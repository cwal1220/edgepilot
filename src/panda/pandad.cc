// pandad: Panda USB 브리지. 10 ms마다 Panda에 쌓인 CAN을 읽어 한 묶음으로 올리고(kCanTopic), controlsd가 만든
// 송신 묶음(kSendCanTopic)은 송신 스레드가 마지막 관문(나이·길이 확인)을 거쳐 보낸다. 하트비트와 상태 조회(PandaState),
// 웹 콘솔이 요청한 Panda 펌웨어 쓰기도 한다. 차량 제어 메시지는 만들지 않는다.

#include "common/build_version.h"
#include "common/ipc_channels.h"
#include "common/utils_file.h"
#include "common/utils_process.h"
#include "common/utils_time.h"
#include "panda/panda_can_codec.h"
#include "panda/panda_client.h"
#include "panda/panda_firmware.h"
#include "panda/panda_flasher.h"

#include <pthread.h>
#include <signal.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

volatile sig_atomic_t g_stop = 0;
// 수신 루프 주기. 상류 pandad처럼 10 ms마다 Panda에 쌓인 CAN을 한 번에 읽어 한 묶음으로 올린다.
constexpr auto kRxCycle = std::chrono::milliseconds(10);
constexpr uint64_t kMaxSendCanAgeNs = 100'000'000ULL;
// 송신 스레드가 sendcan을 기다리는 최대 시간(멈춤 확인 간격). 상류 can_send_thread의 100 ms와 같다.
constexpr uint64_t kTxWaitNs = 100'000'000ULL;
constexpr uint64_t kFirmwareIntervalNs = 1'000'000'000ULL;
constexpr uint64_t kHeartbeatIntervalNs = 500'000'000ULL;  // 상류 pandad 하트비트 0.5초
constexpr uint64_t kHealthIntervalNs = 500'000'000ULL;     // 상태 조회(USB 제어 전송)
constexpr uint64_t kStatsWindowNs = 1'000'000'000ULL;      // 통계 로그 한 줄
// 웹 콘솔(web_console)과 주고받는 파일. 요청 파일에는 쓸 이미지의 버전 문자열이 들어 있다.
constexpr char kFirmwareStatusPath[] = "/dev/shm/edgepilot_panda_status.json";
constexpr char kFlashRequestPath[] = "/dev/shm/edgepilot_panda_flash";
constexpr char kDefaultFirmwarePath[] = "firmware/panda.bin.signed";

// controlsd가 결합하는 모드는 hyundaiCommunity뿐이다(control_holds.h). 그 밖에는 송신하지 않는 nooutput.
uint16_t parse_safety_model(const char *name) {
    const char *value = std::getenv(name);
    const std::string mode = value && value[0] ? value : "nooutput";
    if (mode == "hyundaiCommunity") return kPandaSafetyHyundaiCommunity;
    return kPandaSafetyNoOutput;
}

void fill_rx_batch(CanBatch *batch, const PandaCanFrame *frames, size_t size) {
    fill_can_batch(batch, frames, size, [](IpcCanFrame *dst, const PandaCanFrame &src) {
        dst->address = src.address;
        dst->src = src.bus;
        dst->data_len = src.data_len;
        dst->flags = (src.returned ? kCanFrameReturned : 0U) | (src.rejected ? kCanFrameRejected : 0U);
        std::memcpy(dst->data, src.data, std::min<size_t>(src.data_len, sizeof(dst->data)));
    });
}

// 보낼 묶음은 controlsd가 만든 LKAS11·CLU11·MDPS12뿐이다(버스 0~2, 4·8바이트). 길이는 복사할 버퍼를 넘지 않게만 본다.
std::vector<PandaCanFrame> frames_from_batch(const CanBatch &batch) {
    std::vector<PandaCanFrame> frames;
    frames.reserve(batch.count);
    for (uint32_t i = 0; i < batch.count; ++i) {
        const IpcCanFrame &src = batch.frames[i];
        if (src.data_len > kPandaCanMaxDataLen) continue;
        PandaCanFrame frame;
        frame.address = src.address;
        frame.bus = static_cast<uint8_t>(src.src);
        frame.data_len = static_cast<uint8_t>(src.data_len);
        std::memcpy(frame.data, src.data, frame.data_len);
        frames.push_back(frame);
    }
    return frames;
}

void publish_disconnected(LatestChannel &state_pub, bool tx_enabled) {
    PandaState state;
    state.timestamp_ns = monotonic_now_ns();
    state.tx_enabled = tx_enabled ? 1 : 0;
    state_pub.publish(&state, sizeof(state));
}

// 상태를 조회해 PandaState로 올린다. 조회한 값은 health_out에 남겨 1초 통계 줄이 다시 묻지 않게 한다.
bool publish_health(LatestChannel &state_pub, PandaClient &panda, bool tx_enabled, PandaHealthPacket *health_out) {
    PandaHealthPacket health{};
    PandaState state;
    state.timestamp_ns = monotonic_now_ns();
    state.connected = panda.connected() ? 1 : 0;
    state.comms_healthy = panda.comms_healthy() ? 1 : 0;
    state.tx_enabled = tx_enabled ? 1 : 0;
    state.panda_type = panda.hw_type();
    const bool got = panda.get_health(&health);
    if (got) {
        state.controls_allowed = health.controls_allowed_pkt;
        state.ignition_line = health.ignition_line_pkt;
        state.ignition_can = health.ignition_can_pkt;
        state.safety_mode = health.safety_mode_pkt;
        state.safety_param = health.safety_param_pkt;
        state.can_rx_errs = health.can_rx_errs_pkt;
        state.can_send_errs = health.can_send_errs_pkt;
        state.can_fwd_errs = health.can_fwd_errs_pkt;
        state.blocked_msg_cnt = health.blocked_msg_cnt_pkt;
        state.heartbeat_lost = health.heartbeat_lost_pkt;
        state.usb_tx_timeouts = panda.usb_tx_timeouts();
        state.usb_tx_retries = panda.usb_tx_retries();
        state.malformed_rx_batches = panda.malformed_rx_batches();
        state.faults = health.faults_pkt;
        state.fault_status = health.fault_status_pkt;
        state.voltage = health.voltage_pkt;
        state.current = health.current_pkt;
    }
    state_pub.publish(&state, sizeof(state));
    if (got && health_out) *health_out = health;
    return got;
}

// 송신 스레드의 1초 창 통계. 메인 스레드가 로그를 찍으며 비운다.
struct TxStats {
    std::atomic<unsigned> frames{0};
    std::atomic<unsigned> batches{0};
    std::atomic<unsigned> stale{0};
    std::atomic<unsigned> blocked{0};  // EDGEPILOT_PANDA_TX가 꺼져 보내지 않은 프레임
    std::atomic<unsigned> unsent{0};   // 접속·safety 설정 전, 재접속·펌웨어 쓰기 중이라 버린 묶음
    std::atomic<unsigned> log_full{0};
    std::atomic<unsigned> errors{0};
    std::atomic<uint64_t> lag_sum_us{0};  // controlsd가 묶음을 만든 뒤 USB로 보내기 시작할 때까지
    std::atomic<unsigned> lag_n{0};
    std::atomic<unsigned> lag_max_us{0};

    void add_lag(uint64_t lag_ns) {
        const unsigned us = static_cast<unsigned>(std::min<uint64_t>(lag_ns / 1000, UINT32_MAX));
        lag_sum_us += us;
        ++lag_n;
        unsigned seen = lag_max_us.load();
        while (us > seen && !lag_max_us.compare_exchange_weak(seen, us)) {
        }
    }
};

// 1초 창의 브리지 통계. 창이 끝나면 panda 헬스와 함께 한 줄로 찍고 비운다.
struct BridgeStats {
    unsigned rx_frames = 0;
    unsigned rx_queue_full = 0;
    unsigned rx_log_queue_full = 0;
    unsigned rx_rejected = 0;
    unsigned rx_late = 0;  // 10 ms 주기를 넘긴 수신 주기
    unsigned errors = 0;
    std::map<std::pair<uint32_t, uint8_t>, unsigned> rejected_frames;

    // last_health는 직전 publish_health가 조회한 값(없으면 nullptr). 여기서 다시 묻지 않는다.
    void log(PandaClient &panda, TxStats &tx, const PandaHealthPacket *last_health, unsigned long long tx_depth,
             unsigned long long rx_depth) {
        const PandaHealthPacket health = last_health ? *last_health : PandaHealthPacket{};  // 없으면 0
        const unsigned lag_n = tx.lag_n.exchange(0);
        const uint64_t lag_sum = tx.lag_sum_us.exchange(0);
        std::fprintf(stderr,
                     "pandad: rx=%u tx=%u batches=%u stale=%u unsent=%u txLag=%llu/%uus rxLate=%u "
                     "queue=%llu/%llu rxFull=%u logFull=%u/%u "
                     "blocked=%u rejected=%u errors=%u/%u "
                     "canerr=%u/%u/%u pandaBlocked=%u "
                     "heartbeatLost=%u controls=%u usb=%u/%u malformed=%u "
                     "safety=%u:%u ign=%u/%u voltage=%umV current=%umA faults=0x%x\n",
                     rx_frames, tx.frames.exchange(0), tx.batches.exchange(0), tx.stale.exchange(0),
                     tx.unsent.exchange(0), static_cast<unsigned long long>(lag_n ? lag_sum / lag_n : 0),
                     tx.lag_max_us.exchange(0), rx_late, tx_depth, rx_depth, rx_queue_full, rx_log_queue_full,
                     tx.log_full.exchange(0), tx.blocked.exchange(0), rx_rejected, errors, tx.errors.exchange(0),
                     health.can_rx_errs_pkt, health.can_send_errs_pkt, health.can_fwd_errs_pkt,
                     health.blocked_msg_cnt_pkt, health.heartbeat_lost_pkt, health.controls_allowed_pkt,
                     panda.usb_tx_timeouts(), panda.usb_tx_retries(), panda.malformed_rx_batches(),
                     health.safety_mode_pkt, health.safety_param_pkt, health.ignition_line_pkt, health.ignition_can_pkt,
                     health.voltage_pkt, health.current_pkt, health.faults_pkt);
        for (const auto &[key, count] : rejected_frames) {
            std::fprintf(stderr, "pandad: rejected addr=0x%x bus=%u count=%u\n", key.first, key.second, count);
        }
        *this = BridgeStats{};
    }
};

// 이번 주기에 받은 프레임을 CAN 토픽과 로그 토픽에 올린다. 256개를 넘으면(밀린 뒤 한꺼번에 왔을 때) 나눠 올린다.
// batch는 호출하는 쪽이 다시 쓰는 묶음이다(fill_can_batch).
void publish_rx(const std::vector<PandaCanFrame> &frames, CanQueue &can_pub, CanQueue &can_log_pub, CanBatch *batch,
                BridgeStats *stats) {
    for (size_t start = 0; start < frames.size(); start += kCanBatchMaxFrames) {
        fill_rx_batch(batch, frames.data() + start, std::min<size_t>(frames.size() - start, kCanBatchMaxFrames));
        if (!can_pub.push(*batch)) {
            ++stats->rx_queue_full;
            ++stats->errors;
        }
        if (!can_log_pub.push(*batch)) ++stats->rx_log_queue_full;
    }
}

// USB 연결과 safety 모델 설정. 실패하면 false를 돌려주고 호출자가 다음 루프에서
// 다시 시도한다. 연결이 안 되거나 safety 설정이 맞지 않으면 1초 쉰다.
bool connect_and_configure(PandaClient &panda, uint16_t safety_model, uint16_t safety_param, BridgeStats *stats) {
    // 기다리기 시작할 때 한 번만 알린다(1초마다 찍으면 Panda 없는 시간 동안 로그의 1/5을 차지했다)
    static bool waiting_logged = false;
    if (!panda.connect()) {
        if (!waiting_logged) std::fprintf(stderr, "pandad: waiting for panda\n");
        waiting_logged = true;
        sleep(1);
        return false;
    }
    waiting_logged = false;
    std::fprintf(stderr, "pandad: connected serial=%s hw_type=%u health_v=%u can_v=%u\n", panda.usb_serial().c_str(),
                 panda.hw_type(), panda.health_packet_version(), panda.can_packet_version());
    PandaHealthPacket configured_health{};
    if (!panda.set_safety_model(safety_model, safety_param) || !panda.get_health(&configured_health) ||
        configured_health.safety_mode_pkt != safety_model || configured_health.safety_param_pkt != safety_param) {
        // 같은 실패가 이어지면 처음 한 번만 알리고, 1초 쉬고 다시 잇는다(쉬지 않으면 재접속만 되풀이했다)
        static std::string last_failure;
        const std::string failure = std::to_string(configured_health.safety_mode_pkt) + ":" +
                                    std::to_string(configured_health.safety_param_pkt);
        if (failure != last_failure)
            std::fprintf(stderr, "pandad: safety setup failed expected=%u:%u actual=%s\n", safety_model, safety_param,
                         failure.c_str());
        last_failure = failure;
        ++stats->errors;
        panda.close();
        sleep(1);
        return false;
    }
    panda.set_configured();  // 이제부터 송신 스레드가 보낸다
    return true;
}

// 웹 콘솔의 판다 펌웨어 카드: 1초마다 연결·버전·플래싱 진행을 상태 파일에 쓰고, 요청 파일이 있으면
// 설치된 이미지와 차 상태를 확인한 뒤 판다에 펌웨어를 쓴다. 쓰는 동안(약 10초) CAN은 멈추고, 끝나면
// 루프가 판다에 다시 잇는다.
class FirmwareService {
public:
    explicit FirmwareService(std::string image_path) : image_path_(std::move(image_path)) {}

    void write_status(const PandaClient &panda) {
        PandaLinkStatus link;
        if (panda.connected()) {
            link.mode = "app";
            link.serial = panda.usb_serial();
            link.hw_type = panda.hw_type();
            link.firmware_version = panda.firmware_version();
        } else if (flash_.state != PandaFlashState::Flashing && panda_usb_mode() == PandaUsbMode::Bootstub) {
            link.mode = "bootstub";  // 앱이 서지 않았다: 다시 쓰면 된다
        }
        write_file_atomic(kFirmwareStatusPath, panda_status_json(link, flash_, monotonic_now_ns()));
    }

    // 요청이 있으면 처리한다. 펌웨어를 썼으면 true이고, 그때 판다 연결은 닫혀 있다.
    bool service_request(PandaClient &panda) {
        if (access(kFlashRequestPath, F_OK) != 0) return false;
        std::string requested = read_text_file(kFlashRequestPath);
        std::remove(kFlashRequestPath);
        requested.erase(std::min(requested.size(), requested.find_last_not_of(" \t\r\n") + 1));
        flash_ = PandaFlashStatus{};
        flash_.version = requested;
        flash_.stamp_ns = monotonic_now_ns();

        PandaImage image;
        std::string detail;
        if (!load_panda_image(image_path_, &image, &detail)) return refuse(panda, "image_invalid", detail);
        if (image.version != requested) return refuse(panda, "image_changed", "installed image is " + image.version);
        control_.attach(kControlStateTopic);
        control_.poll();
        std::string reason;
        const bool bootstub = !panda.connected() && panda_usb_mode() == PandaUsbMode::Bootstub;
        if (!panda_flash_allowed(control_.latest(), monotonic_now_ns(), bootstub, &reason))
            return refuse(panda, reason, "car state does not allow flashing");
        if (panda.connected() && !panda_hw_type_flashable(panda.hw_type()))
            return refuse(panda, "hw_unsupported", panda_hw_type_name(panda.hw_type()));
        if (!panda.connected() && panda_usb_mode() == PandaUsbMode::None)
            return refuse(panda, "no_panda", "no panda on USB");

        std::fprintf(stderr, "pandad: flashing panda firmware %s (%zu bytes) from %s\n", image.version.c_str(),
                     image.bytes.size(), image_path_.c_str());
        panda.close();
        flash_.state = PandaFlashState::Flashing;
        write_status(panda);
        const PandaFlashResult result = flash_panda(image, [&](const char *step, int percent) {
            flash_.step = step;
            flash_.percent = percent;
            write_status(panda);
        });
        flash_.state = result.ok ? PandaFlashState::Done : PandaFlashState::Failed;
        flash_.error = result.error;
        flash_.detail = result.detail;
        flash_.stamp_ns = monotonic_now_ns();
        if (result.ok) {
            std::fprintf(stderr, "pandad: panda flash done, firmware %s\n", result.version.c_str());
        } else {
            std::fprintf(stderr, "pandad: panda flash failed: %s (%s)\n", result.error.c_str(), result.detail.c_str());
        }
        write_status(panda);
        return true;
    }

private:
    bool refuse(const PandaClient &panda, const std::string &error, const std::string &detail) {
        flash_.state = PandaFlashState::Failed;
        flash_.error = error;
        flash_.detail = detail;
        std::fprintf(stderr, "pandad: panda flash refused: %s (%s)\n", error.c_str(), detail.c_str());
        write_status(panda);
        return false;
    }

    std::string image_path_;
    PandaFlashStatus flash_;
    Subscription<ControlState> control_;
};

// USB 수신 한 번. 실패하면 false이고 호출자가 연결을 끊고 다시 잇는다. Panda는 쌓인 것이 없으면 빈 응답을
// 바로 돌려준다(펌웨어 EP1 IN).
bool service_rx(PandaClient &panda, bool log_can, std::vector<PandaCanFrame> *frames, BridgeStats *stats) {
    if (!panda.receive(frames, 10)) {
        ++stats->errors;
        return false;
    }
    for (const PandaCanFrame &frame : *frames) {
        if (frame.rejected) {
            ++stats->rx_rejected;
            ++stats->rejected_frames[{frame.address, frame.bus}];
        }
    }
    stats->rx_frames += static_cast<unsigned>(frames->size());
    if (log_can) {
        for (const PandaCanFrame &frame : *frames) {
            std::fprintf(stderr, "can rx bus=%u addr=0x%x len=%u returned=%u rejected=%u\n", frame.bus, frame.address,
                         frame.data_len, frame.returned ? 1 : 0, frame.rejected ? 1 : 0);
        }
    }
    // Panda가 버스에 낸 프레임(버스 사이 전달과 우리 송신)은 returned로 다시 온다. 받은 프레임의 약 2/3(경로 하나에
    // 700 MB 넘게)이고 읽는 쪽이 모두 버리므로 올리지도 녹화하지도 않는다. 거부된 프레임(rejected)은 둔다.
    frames->erase(std::remove_if(frames->begin(), frames->end(), [](const PandaCanFrame &f) { return f.returned; }),
                  frames->end());
    return true;
}

// 송신 스레드. 상류 can_send_thread처럼 sendcan이 오면 바로 Panda로 보낸다. 예전에는 수신 루프가 빈 수신 뒤
// 5 ms씩 자는 동안 sendcan도 기다려, 보내기까지 평균 약 2 ms, 길게는 5 ms가 걸렸다(10-12 주행 기록).
// 보낼 수 있는지(접속 + safety 설정 확인)와 100 ms 신선도는 PandaClient::send가 USB 잠금을 얻은 뒤 다시 본다:
// 메인 스레드의 USB 작업(수신, 하트비트, 상태 조회, 재접속)을 기다린 묶음이 낡았거나 재접속 직후 설정 전이면 보내지 않는다.
void run_tx(CanQueue &sendcan_sub, CanQueue &sendcan_log_pub, PandaClient &panda, bool tx_enabled,
            const std::atomic<bool> &stop, TxStats *stats) {
    pthread_setname_np(pthread_self(), "pandad_tx");
    // 묶음 하나가 21.5 KB라 한 번만 잡는다. pop은 count까지 채우고 그 뒤는 읽지 않는다.
    const std::unique_ptr<CanBatch> batch(new CanBatch);
    while (!stop.load()) {
        if (!sendcan_sub.pop_wait(batch.get(), kTxWaitNs)) continue;
        ++stats->batches;
        // 생산자는 이 스레드가 시계를 읽은 뒤에 발행할 수 있다. 꺼낸 뒤 시각을 새로 읽어, 새 묶음을 미래나 낡은
        // 묶음으로 잘못 보고 건너뛰는 일이 없게 한다.
        if (!can_batch_is_fresh(*batch, monotonic_now_ns(), kMaxSendCanAgeNs)) {
            ++stats->stale;
            continue;
        }
        const std::vector<PandaCanFrame> tx = frames_from_batch(*batch);
        if (!tx_enabled) {
            if (!panda.ready()) {
                ++stats->unsent;
                continue;
            }
            if (!sendcan_log_pub.push(*batch)) ++stats->log_full;
            stats->blocked += static_cast<unsigned>(tx.size());
            continue;
        }
        uint64_t started_ns = 0;
        switch (panda.send(tx, batch->timestamp_ns + kMaxSendCanAgeNs, &started_ns)) {
            case PandaSendResult::NotReady:
                ++stats->unsent;
                continue;
            case PandaSendResult::Expired:
                ++stats->stale;
                continue;
            case PandaSendResult::Sent:
                stats->frames += static_cast<unsigned>(tx.size());
                stats->add_lag(started_ns - batch->timestamp_ns);
                break;
            case PandaSendResult::Failed:
                ++stats->errors;
                break;
        }
        // 기록(recordd의 CanTx)에는 보낸(보내려 한) 묶음만 남긴다.
        if (!sendcan_log_pub.push(*batch)) ++stats->log_full;
    }
}

// 송신 스레드와 멈춤 신호. 예외로 main을 빠져나가도 세우고 기다린다(joinable한 std::thread를 그대로 지우면
// terminate). 스레드가 쓰는 큐와 PandaClient보다 뒤에 만들어 먼저 지워지게 한다.
struct TxThread {
    std::atomic<bool> stop{false};
    std::thread thread;

    ~TxThread() {
        stop = true;
        if (thread.joinable()) thread.join();
    }
};

}  // namespace

int main() {
    install_stop_signal_handlers(&g_stop);
    log_build_version("pandad");

    try {
        CanQueue can_pub;
        CanQueue sendcan_sub;
        CanQueue can_log_pub;
        CanQueue sendcan_log_pub;
        LatestChannel panda_state_pub;
        if (!can_pub.open(kCanTopic, kCanQueueSlots, true)) throw std::runtime_error("open can ipc failed");
        if (!sendcan_sub.open(kSendCanTopic, kCanQueueSlots, true)) throw std::runtime_error("open sendcan ipc failed");
        if (!can_log_pub.open(kCanLogTopic, kCanQueueSlots, true)) throw std::runtime_error("open CAN log ipc failed");
        if (!sendcan_log_pub.open(kSendCanLogTopic, kCanQueueSlots, true))
            throw std::runtime_error("open sendcan log ipc failed");
        if (!panda_state_pub.open(kPandaStateTopic, sizeof(PandaState), true))
            throw std::runtime_error("open pandaState ipc failed");
        can_pub.reset();
        can_log_pub.reset();
        sendcan_log_pub.reset();

        const bool tx_enabled = env_flag("EDGEPILOT_PANDA_TX", false);
        // 동봉한 펌웨어(firmware/panda)는 하트비트의 결합 값을 읽지 않고 타이머만 되돌린다. 새 펌웨어가 읽는다
        const bool heartbeat_engaged = env_flag("EDGEPILOT_PANDA_ENGAGED", false);
        const bool log_can = env_flag("EDGEPILOT_PANDA_LOG_CAN", false);
        const uint16_t safety_model = parse_safety_model("EDGEPILOT_PANDA_SAFETY");
        const uint16_t safety_param = static_cast<uint16_t>(env_unsigned("EDGEPILOT_PANDA_SAFETY_PARAM", 0));

        if (tx_enabled) {
            std::fprintf(stderr, "pandad: TX enabled safety=%u param=%u engaged=%u\n", safety_model, safety_param,
                         heartbeat_engaged ? 1 : 0);
        } else {
            std::fprintf(stderr, "pandad: shadow mode TX disabled safety=%u param=%u\n", safety_model, safety_param);
        }

        PandaClient panda;
        BridgeStats stats;
        TxStats tx_stats;
        FirmwareService firmware(env_string("EDGEPILOT_PANDA_FIRMWARE", kDefaultFirmwarePath));
        uint64_t last_health_ns = 0;
        uint64_t last_heartbeat_ns = 0;
        uint64_t last_log_ns = 0;
        uint64_t last_firmware_ns = 0;
        std::vector<PandaCanFrame> frames;
        frames.reserve(kCanBatchMaxFrames);
        PandaHealthPacket last_health{};
        bool have_health = false;

        TxThread tx;
        tx.thread = std::thread(run_tx, std::ref(sendcan_sub), std::ref(sendcan_log_pub), std::ref(panda), tx_enabled,
                                std::cref(tx.stop), &tx_stats);

        using Clock = std::chrono::steady_clock;
        auto next_cycle = Clock::now();
        CanBatch rx_batch;  // 주기마다 21.5 KB를 0으로 채우지 않게 다시 쓴다
        while (!g_stop) {
            if (!panda.connected()) {
                have_health = false;
                publish_disconnected(panda_state_pub, tx_enabled);
                // 판다가 없거나 bootstub에 머물러도 웹 콘솔에서 펌웨어를 다시 쓸 수 있게 여기서도 본다
                if (monotonic_now_ns() - last_firmware_ns >= kFirmwareIntervalNs) {
                    firmware.service_request(panda);
                    firmware.write_status(panda);
                    last_firmware_ns = monotonic_now_ns();
                }
                if (!connect_and_configure(panda, safety_model, safety_param, &stats)) continue;
                last_health_ns = 0;
                last_heartbeat_ns = 0;
                next_cycle = Clock::now();
            }

            if (!service_rx(panda, log_can, &frames, &stats)) {
                panda.close();
                continue;
            }
            publish_rx(frames, can_pub, can_log_pub, &rx_batch, &stats);

            const uint64_t now = monotonic_now_ns();
            // 하트비트와 상태 조회(USB 제어 전송, 각각 최대 100 ms)를 한 주기에 몰지 않는다. 송신 스레드가 USB 잠금을
            // 두 번 연달아 기다리지 않게 한다.
            bool control_transfer = false;
            if (now - last_heartbeat_ns >= kHeartbeatIntervalNs) {
                panda.send_heartbeat(heartbeat_engaged && tx_enabled);
                last_heartbeat_ns = now;
                control_transfer = true;
            }
            if (!control_transfer && now - last_health_ns >= kHealthIntervalNs) {
                have_health = publish_health(panda_state_pub, panda, tx_enabled, &last_health);
                last_health_ns = now;
                // 접속 뒤에도 safety 모드를 본다. 하트비트가 5초 끊기면 펌웨어는 NOOUTPUT으로 바꾸고 다음 하트비트에도
                // 그대로 둔다. 그대로 두면 재접속할 때까지 조향이 막힌다. 이때 controls_allowed는 이미 0이라 다시
                // 설정해도 잃는 것이 없다(운전자 버튼으로 다시 얻는다). 상태 조회 주기(0.5초)마다만 다시 보낸다.
                if (have_health &&
                    (last_health.safety_mode_pkt != safety_model || last_health.safety_param_pkt != safety_param)) {
                    std::fprintf(stderr, "pandad: safety is %u:%u, setting %u:%u again\n", last_health.safety_mode_pkt,
                                 last_health.safety_param_pkt, safety_model, safety_param);
                    panda.set_safety_model(safety_model, safety_param);
                }
            }
            if (now - last_log_ns >= kStatsWindowNs) {
                stats.log(panda, tx_stats, have_health ? &last_health : nullptr,
                          static_cast<unsigned long long>(sendcan_sub.depth()),
                          static_cast<unsigned long long>(can_pub.depth()));
                last_log_ns = now;
            }
            if (now - last_firmware_ns >= kFirmwareIntervalNs) {
                last_firmware_ns = now;
                // 펌웨어를 쓰면 판다 연결이 닫혀 송신 스레드의 send는 NotReady가 되고, 다음 루프가 다시 잇는다
                if (firmware.service_request(panda)) continue;
                firmware.write_status(panda);
            }

            // 다음 주기까지 잔다. 넘겼으면 밀린 만큼 몰아서 읽지 않고 지금부터 다시 센다.
            next_cycle += kRxCycle;
            const auto cycle_end = Clock::now();
            if (next_cycle > cycle_end) {
                std::this_thread::sleep_until(next_cycle);
            } else {
                ++stats.rx_late;
                next_cycle = cycle_end;
            }
        }

        std::fprintf(stderr, "\npandad: stopping\n");
        return 0;
    } catch (const std::exception &e) {
        std::fprintf(stderr, "pandad error: %s\n", e.what());
        return 1;
    }
}
