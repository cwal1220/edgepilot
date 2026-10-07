#include "common/utils_file.h"
#include "common/utils_process.h"
#include "common/utils_time.h"
#include "common/ipc_channels.h"
#include "panda/panda_can_codec.h"
#include "panda/panda_client.h"
#include "panda/panda_firmware.h"
#include "panda/panda_flasher.h"

#include <signal.h>
#include <unistd.h>

#include <algorithm>
#include <map>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

volatile sig_atomic_t g_stop = 0;
constexpr uint64_t kCanPublishIntervalNs = 10000000ULL;
constexpr uint64_t kMaxSendCanAgeNs = 100000000ULL;
constexpr uint64_t kFirmwareIntervalNs = 1000000000ULL;
// 웹 콘솔(param_server.py)과 주고받는 파일. 요청 파일에는 쓸 이미지의 버전 문자열이 들어 있다.
constexpr char kFirmwareStatusPath[] = "/dev/shm/edgepilot_panda_status.json";
constexpr char kFlashRequestPath[] = "/dev/shm/edgepilot_panda_flash";
constexpr char kDefaultFirmwarePath[] = "firmware/panda.bin.signed";

uint16_t parse_safety_model(const char *name, uint16_t *default_param)
{
    const char *value = std::getenv(name);
    const std::string mode = value && value[0] ? value : "nooutput";
    *default_param = 0;
    if (mode == "silent") return kPandaSafetySilent;
    if (mode == "elm327") return kPandaSafetyElm327;
    if (mode == "hyundai") {
        *default_param = 2;  // KIA K7 YG HEV is a Hyundai/Kia hybrid safety-param path.
        return kPandaSafetyHyundai;
    }
    if (mode == "hyundaiCommunity") return kPandaSafetyHyundaiCommunity;
    if (mode == "allOutput") return kPandaSafetyAllOutput;
    return kPandaSafetyNoOutput;
}

CanBatch rx_can_batch(const std::vector<PandaCanFrame> &frames)
{
    return make_can_batch(frames, [](IpcCanFrame *dst, const PandaCanFrame &src) {
        dst->address = src.address;
        dst->src = src.bus;
        dst->data_len = src.data_len;
        dst->flags = (src.returned ? 0x1U : 0U) | (src.rejected ? 0x2U : 0U);
        std::memcpy(dst->data, src.data, std::min<size_t>(src.data_len, sizeof(dst->data)));
    });
}

std::vector<PandaCanFrame> frames_from_batch(const CanBatch &batch)
{
    std::vector<PandaCanFrame> frames;
    if (!batch.valid) return frames;
    const uint32_t count = std::min<uint32_t>(batch.count, kCanBatchMaxFrames);
    frames.reserve(count);
    for (uint32_t i = 0; i < count; ++i) {
        const IpcCanFrame &src = batch.frames[i];
        if (src.flags != 0) continue;
        if (src.address > kPandaCanMaxAddress) continue;
        if (src.src > kPandaCanMaxTxBus) continue;
        if (src.data_len > kPandaCanMaxDataLen) continue;
        if (!panda_can_is_valid_data_len(static_cast<uint8_t>(src.data_len))) continue;
        PandaCanFrame frame;
        frame.address = src.address;
        frame.bus = static_cast<uint8_t>(src.src);
        frame.data_len = static_cast<uint8_t>(src.data_len);
        std::memcpy(frame.data, src.data, frame.data_len);
        frames.push_back(frame);
    }
    return frames;
}

void publish_disconnected(LatestChannel &state_pub, bool tx_enabled)
{
    PandaState state;
    state.timestamp_ns = monotonic_now_ns();
    state.tx_enabled = tx_enabled ? 1 : 0;
    state_pub.publish(&state, sizeof(state));
}

void publish_health(LatestChannel &state_pub, PandaClient &panda, bool tx_enabled)
{
    PandaHealth health;
    PandaState state;
    state.timestamp_ns = monotonic_now_ns();
    state.connected = panda.connected() ? 1 : 0;
    state.comms_healthy = panda.comms_healthy() ? 1 : 0;
    state.tx_enabled = tx_enabled ? 1 : 0;
    state.panda_type = panda.hw_type();
    if (panda.get_health(&health)) {
        state.controls_allowed = health.controls_allowed;
        state.ignition_line = health.ignition_line;
        state.ignition_can = health.ignition_can;
        state.safety_mode = health.safety_mode;
        state.safety_param = health.safety_param;
        state.can_rx_errs = health.can_rx_errs;
        state.can_send_errs = health.can_send_errs;
        state.can_fwd_errs = health.can_fwd_errs;
        state.blocked_msg_cnt = health.blocked_msg_cnt;
        state.heartbeat_lost = health.heartbeat_lost;
        state.usb_tx_timeouts = panda.usb_tx_timeouts();
        state.usb_tx_retries = panda.usb_tx_retries();
        state.malformed_rx_batches = panda.malformed_rx_batches();
        state.faults = health.faults;
        state.fault_status = health.fault_status;
        state.voltage = health.voltage;
        state.current = health.current;
    }
    state_pub.publish(&state, sizeof(state));
}

/* 1초 창의 브리지 통계. 창이 끝나면 panda 헬스와 함께 한 줄로 찍고 비운다. */
struct BridgeStats {
    unsigned rx_frames = 0;
    unsigned tx_frames = 0;
    unsigned tx_batches = 0;
    unsigned rx_queue_full = 0;
    unsigned rx_log_queue_full = 0;
    unsigned tx_log_queue_full = 0;
    unsigned tx_stale = 0;
    unsigned tx_blocked = 0;
    unsigned rx_rejected = 0;
    unsigned errors = 0;
    std::map<std::pair<uint32_t, uint8_t>, unsigned> rejected_frames;

    void log(PandaClient &panda, unsigned long long tx_depth, unsigned long long rx_depth)
    {
        PandaHealth health;
        const bool got_health = panda.get_health(&health);
        std::fprintf(stderr,
                     "pandad: rx=%u tx=%u batches=%u stale=%u "
                     "queue=%llu/%llu rxFull=%u logFull=%u/%u "
                     "blocked=%u rejected=%u errors=%u "
                     "canerr=%u/%u/%u pandaBlocked=%u "
                     "heartbeatLost=%u controls=%u usb=%u/%u malformed=%u "
                     "safety=%u:%u ign=%u/%u voltage=%umV current=%umA faults=0x%x\n",
                     rx_frames, tx_frames, tx_batches, tx_stale,
                     tx_depth, rx_depth,
                     rx_queue_full,
                     rx_log_queue_full, tx_log_queue_full,
                     tx_blocked, rx_rejected, errors,
                     got_health ? health.can_rx_errs : 0,
                     got_health ? health.can_send_errs : 0,
                     got_health ? health.can_fwd_errs : 0,
                     got_health ? health.blocked_msg_cnt : 0,
                     got_health ? health.heartbeat_lost : 0,
                     got_health ? health.controls_allowed : 0,
                     panda.usb_tx_timeouts(), panda.usb_tx_retries(),
                     panda.malformed_rx_batches(),
                     got_health ? health.safety_mode : 0,
                     got_health ? health.safety_param : 0,
                     got_health ? health.ignition_line : 0,
                     got_health ? health.ignition_can : 0,
                     got_health ? health.voltage : 0,
                     got_health ? health.current : 0,
                     got_health ? health.faults : 0);
        for (const auto &[key, count] : rejected_frames) {
            std::fprintf(stderr,
                         "pandad: rejected addr=0x%x bus=%u count=%u\n",
                         key.first, key.second, count);
        }
        *this = BridgeStats{};
    }
};

/* 수신 프레임을 10 ms 또는 256개 단위로 모아 CAN 토픽과 로그 토픽에 올린다. */
class RxBatcher {
public:
    RxBatcher() { pending_.reserve(kCanBatchMaxFrames); }

    void clear()
    {
        pending_.clear();
        dropped_ = 0;
    }

    void reset(uint64_t now_ns)
    {
        clear();
        last_publish_ns_ = now_ns;
    }

    void add(const std::vector<PandaCanFrame> &frames)
    {
        pending_.insert(pending_.end(), frames.begin(), frames.end());
    }

    bool due(uint64_t now_ns) const
    {
        return !pending_.empty() &&
               (now_ns - last_publish_ns_ >= kCanPublishIntervalNs ||
                pending_.size() >= kCanBatchMaxFrames);
    }

    void publish(uint64_t now_ns, CanQueue &can_pub, CanQueue &can_log_pub,
                 BridgeStats *stats)
    {
        if (pending_.size() > kCanBatchMaxFrames) {
            const size_t overflow = pending_.size() - kCanBatchMaxFrames;
            pending_.erase(pending_.begin(), pending_.begin() + overflow);
            dropped_ += static_cast<unsigned>(overflow);
        }
        CanBatch batch = rx_can_batch(pending_);
        batch.dropped += dropped_;
        if (!can_pub.push(batch)) {
            ++stats->rx_queue_full;
            ++stats->errors;
        }
        if (!can_log_pub.push(batch)) ++stats->rx_log_queue_full;
        reset(now_ns);
    }

private:
    std::vector<PandaCanFrame> pending_;
    unsigned dropped_ = 0;
    uint64_t last_publish_ns_ = 0;
};

/* USB 연결과 safety 모델 설정. 실패하면 false를 돌려주고 호출자가 다음 루프에서
 * 다시 시도한다. 연결 자체가 안 되면 1초 쉰다. */
bool connect_and_configure(PandaClient &panda, uint16_t safety_model, uint16_t safety_param,
                           BridgeStats *stats)
{
    if (!panda.connect()) {
        std::fprintf(stderr, "pandad: waiting for panda\n");
        sleep(1);
        return false;
    }
    std::fprintf(stderr,
                 "pandad: connected serial=%s hw_type=%u health_v=%u can_v=%u\n",
                 panda.usb_serial().c_str(), panda.hw_type(),
                 panda.health_packet_version(), panda.can_packet_version());
    PandaHealth configured_health;
    if (!panda.set_safety_model(safety_model, safety_param) ||
        !panda.get_health(&configured_health) ||
        configured_health.safety_mode != safety_model ||
        configured_health.safety_param != safety_param) {
        std::fprintf(stderr,
                     "pandad: safety setup failed expected=%u:%u actual=%u:%u\n",
                     safety_model, safety_param,
                     configured_health.safety_mode,
                     configured_health.safety_param);
        ++stats->errors;
        panda.close();
        return false;
    }
    return true;
}

/* 웹 콘솔의 판다 펌웨어 카드: 1초마다 연결·버전·플래싱 진행을 상태 파일에 쓰고, 요청 파일이 있으면
 * 설치된 이미지와 차 상태를 확인한 뒤 판다에 펌웨어를 쓴다. 쓰는 동안(약 10초) CAN은 멈추고, 끝나면
 * 루프가 판다에 다시 잇는다. */
class FirmwareService {
public:
    explicit FirmwareService(std::string image_path) : image_path_(std::move(image_path)) {}

    void write_status(const PandaClient &panda)
    {
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
    bool service_request(PandaClient &panda)
    {
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
        if (image.version != requested)
            return refuse(panda, "image_changed", "installed image is " + image.version);
        control_.attach(kControlStateTopic);
        control_.poll();
        std::string reason;
        if (!panda_flash_allowed(control_.latest(), monotonic_now_ns(), &reason))
            return refuse(panda, reason, "car state does not allow flashing");
        if (panda.connected() && !panda_hw_type_flashable(panda.hw_type()))
            return refuse(panda, "hw_unsupported", panda_hw_type_name(panda.hw_type()));
        if (!panda.connected() && panda_usb_mode() == PandaUsbMode::None)
            return refuse(panda, "no_panda", "no panda on USB");

        std::fprintf(stderr, "pandad: flashing panda firmware %s (%zu bytes) from %s\n",
                     image.version.c_str(), image.bytes.size(), image_path_.c_str());
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
    bool refuse(const PandaClient &panda, const std::string &error, const std::string &detail)
    {
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

enum class RxResult { Idle, Frames, Error };

// USB 수신 한 번. Error면 호출자가 연결을 끊고 다시 잇는다.
RxResult service_rx(PandaClient &panda, bool log_can, RxBatcher *rx, BridgeStats *stats)
{
    std::vector<PandaCanFrame> frames;
    if (!panda.receive(&frames, 10)) {
        ++stats->errors;
        return RxResult::Error;
    }
    if (frames.empty()) return RxResult::Idle;
    for (const PandaCanFrame &frame : frames) {
        if (frame.rejected) {
            ++stats->rx_rejected;
            ++stats->rejected_frames[{frame.address, frame.bus}];
        }
    }
    rx->add(frames);
    stats->rx_frames += static_cast<unsigned>(frames.size());
    if (log_can) {
        for (const PandaCanFrame &frame : frames) {
            std::fprintf(stderr,
                         "can rx bus=%u addr=0x%x len=%u returned=%u rejected=%u\n",
                         frame.bus, frame.address, frame.data_len,
                         frame.returned ? 1 : 0, frame.rejected ? 1 : 0);
        }
    }
    return RxResult::Frames;
}

// sendcan 큐를 비워 panda로 보낸다. 배치가 하나라도 있었으면 true.
bool service_tx(CanQueue &sendcan_sub, CanQueue &sendcan_log_pub, PandaClient &panda,
                bool tx_enabled, BridgeStats *stats)
{
    bool had_sendcan = false;
    CanBatch send_batch;
    while (sendcan_sub.pop(&send_batch)) {
        had_sendcan = true;
        ++stats->tx_batches;
        // The producer can publish after the loop sampled its clock. Use a
        // fresh timestamp here so a new batch is never mistaken for a
        // future/stale batch and skipped from the torque sequence.
        const uint64_t tx_now = monotonic_now_ns();
        if (!can_batch_is_fresh(send_batch, tx_now, kMaxSendCanAgeNs)) {
            ++stats->tx_stale;
            continue;
        }
        const std::vector<PandaCanFrame> tx = frames_from_batch(send_batch);
        if (!sendcan_log_pub.push(send_batch)) ++stats->tx_log_queue_full;
        if (tx_enabled) {
            if (panda.send(tx)) {
                stats->tx_frames += static_cast<unsigned>(tx.size());
            } else {
                ++stats->errors;
            }
        } else {
            stats->tx_blocked += static_cast<unsigned>(tx.size());
        }
    }
    return had_sendcan;
}

} // namespace

int main()
{
    install_stop_signal_handlers(&g_stop);

    try {
        CanQueue can_pub;
        CanQueue sendcan_sub;
        CanQueue can_log_pub;
        CanQueue sendcan_log_pub;
        LatestChannel panda_state_pub;
        if (!can_pub.open(kCanTopic, kCanQueueSlots, true))
            throw std::runtime_error("open can ipc failed");
        if (!sendcan_sub.open(kSendCanTopic, kCanQueueSlots, true))
            throw std::runtime_error("open sendcan ipc failed");
        if (!can_log_pub.open(kCanLogTopic, kCanQueueSlots, true))
            throw std::runtime_error("open CAN log ipc failed");
        if (!sendcan_log_pub.open(kSendCanLogTopic, kCanQueueSlots, true))
            throw std::runtime_error("open sendcan log ipc failed");
        if (!panda_state_pub.open(kPandaStateTopic, sizeof(PandaState), true))
            throw std::runtime_error("open pandaState ipc failed");
        can_pub.reset();
        can_log_pub.reset();
        sendcan_log_pub.reset();

        const bool tx_enabled = env_flag("EDGEPILOT_PANDA_TX", false);
        const bool heartbeat_engaged = env_flag("EDGEPILOT_PANDA_ENGAGED", false);
        const bool log_can = env_flag("EDGEPILOT_PANDA_LOG_CAN", false);
        const uint16_t idle_us = static_cast<uint16_t>(env_unsigned("EDGEPILOT_PANDA_IDLE_US", 5000));
        uint16_t default_safety_param = 0;
        const uint16_t safety_model = parse_safety_model("EDGEPILOT_PANDA_SAFETY", &default_safety_param);
        const uint16_t safety_param = static_cast<uint16_t>(env_unsigned("EDGEPILOT_PANDA_SAFETY_PARAM", default_safety_param));

        if (tx_enabled) {
            std::fprintf(stderr,
                         "pandad: TX enabled safety=%u param=%u engaged=%u\n",
                         safety_model, safety_param, heartbeat_engaged ? 1 : 0);
        } else {
            std::fprintf(stderr,
                         "pandad: shadow mode TX disabled safety=%u param=%u\n",
                         safety_model, safety_param);
        }

        PandaClient panda;
        RxBatcher rx;
        BridgeStats stats;
        FirmwareService firmware(env_string("EDGEPILOT_PANDA_FIRMWARE", kDefaultFirmwarePath));
        uint64_t last_health_ns = 0;
        uint64_t last_heartbeat_ns = 0;
        uint64_t last_log_ns = 0;
        uint64_t last_firmware_ns = 0;

        while (!g_stop) {
            if (!panda.connected()) {
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
                rx.reset(monotonic_now_ns());
            }

            const RxResult received = service_rx(panda, log_can, &rx, &stats);
            if (received == RxResult::Error) {
                rx.clear();
                panda.close();
                continue;
            }

            const uint64_t now = monotonic_now_ns();
            if (rx.due(now)) rx.publish(now, can_pub, can_log_pub, &stats);
            const bool had_sendcan =
                service_tx(sendcan_sub, sendcan_log_pub, panda, tx_enabled, &stats);

            if (now - last_heartbeat_ns >= 500000000ULL) {
                panda.send_heartbeat(heartbeat_engaged && tx_enabled);
                last_heartbeat_ns = now;
            }
            if (now - last_health_ns >= 500000000ULL) {
                publish_health(panda_state_pub, panda, tx_enabled);
                last_health_ns = now;
            }
            if (now - last_log_ns >= 1000000000ULL) {
                stats.log(panda, static_cast<unsigned long long>(sendcan_sub.depth()),
                          static_cast<unsigned long long>(can_pub.depth()));
                last_log_ns = now;
            }
            if (now - last_firmware_ns >= kFirmwareIntervalNs) {
                last_firmware_ns = now;
                if (firmware.service_request(panda)) {
                    rx.clear();
                    continue;  // 펌웨어를 썼다: 다음 루프가 다시 잇는다
                }
                firmware.write_status(panda);
            }
            if (received == RxResult::Idle && !had_sendcan && idle_us > 0) {
                usleep(idle_us);
            }
        }

        std::fprintf(stderr, "\npandad: stopping\n");
        return 0;
    } catch (const std::exception &e) {
        std::fprintf(stderr, "pandad error: %s\n", e.what());
        return 1;
    }
}
