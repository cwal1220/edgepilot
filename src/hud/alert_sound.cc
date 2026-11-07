#include "hud/alert_sound.h"

#include "common/utils_process.h"

#include <fcntl.h>
#include <sound/asound.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace {

constexpr int kRate = 48000;
constexpr int kChannels = 2;
constexpr int kChunkFrames = kRate / 50;  // 20 ms씩 흘린다
constexpr int kPeriods = 3;               // ALSA 버퍼 60 ms: 지연을 짧게 두면서 스레드가 늦어도 끊기지 않을 만큼
// 장치를 열면 드라이버가 앰프를 켠다. 켜지는 동안 첫 음이 잘리지 않게 알림 앞에 무음을 흘린다.
constexpr int kWarmupFrames = kRate * 150 / 1000;  // 150 ms(80·120 ms에서는 첫 음이 조금 잘렸다)

// 모노 알림음을 인터리브 스테레오로.
std::vector<int16_t> stereo(const std::vector<int16_t> &mono) {
    std::vector<int16_t> pcm(mono.size() * kChannels);
    for (size_t i = 0; i < mono.size(); ++i) pcm[2 * i] = pcm[2 * i + 1] = mono[i];
    return pcm;
}

void set_mask(snd_pcm_hw_params &params, int param, unsigned int bit) {
    snd_mask &mask = params.masks[param - SNDRV_PCM_HW_PARAM_FIRST_MASK];
    std::fill(std::begin(mask.bits), std::end(mask.bits), 0U);
    mask.bits[bit / 32] = 1U << (bit % 32);
}

void set_integer(snd_pcm_hw_params &params, int param, unsigned int value) {
    snd_interval &interval = params.intervals[param - SNDRV_PCM_HW_PARAM_FIRST_INTERVAL];
    interval.min = interval.max = value;
    interval.integer = 1;
}

// 재생 장치를 열어 형식을 정하고 쓸 준비까지 한다(tinyalsa pcm_open과 같은 순서: 모든 마스크·구간을 열어 둔 채
// 원하는 값만 고정해 HW_PARAMS, 그다음 SW_PARAMS, PREPARE). 실패하면 -1.
int open_pcm(const std::string &path) {
    const int fd = ::open(path.c_str(), O_WRONLY | O_CLOEXEC);
    if (fd < 0) return -1;
    snd_pcm_hw_params hw{};
    for (snd_mask &mask : hw.masks) std::fill(std::begin(mask.bits), std::end(mask.bits), ~0U);
    for (snd_interval &interval : hw.intervals) interval.max = ~0U;
    hw.rmask = ~0U;
    hw.info = ~0U;
    set_mask(hw, SNDRV_PCM_HW_PARAM_ACCESS, SNDRV_PCM_ACCESS_RW_INTERLEAVED);
    set_mask(hw, SNDRV_PCM_HW_PARAM_FORMAT, SNDRV_PCM_FORMAT_S16_LE);
    set_mask(hw, SNDRV_PCM_HW_PARAM_SUBFORMAT, SNDRV_PCM_SUBFORMAT_STD);
    set_integer(hw, SNDRV_PCM_HW_PARAM_CHANNELS, kChannels);
    set_integer(hw, SNDRV_PCM_HW_PARAM_RATE, kRate);
    set_integer(hw, SNDRV_PCM_HW_PARAM_PERIOD_SIZE, kChunkFrames);
    set_integer(hw, SNDRV_PCM_HW_PARAM_PERIODS, kPeriods);
    snd_pcm_sw_params sw{};
    sw.period_step = 1;
    sw.avail_min = kChunkFrames;
    sw.start_threshold = kChunkFrames * kPeriods;  // 버퍼가 차면 시작하고(aplay와 같다)
    sw.stop_threshold = kChunkFrames * kPeriods;   // 비면 언더런으로 멈춘다
    if (ioctl(fd, SNDRV_PCM_IOCTL_HW_PARAMS, &hw) < 0 || ioctl(fd, SNDRV_PCM_IOCTL_SW_PARAMS, &sw) < 0 ||
        ioctl(fd, SNDRV_PCM_IOCTL_PREPARE) < 0) {
        std::fprintf(stderr, "alert sound: %s setup failed: %s\n", path.c_str(), std::strerror(errno));
        ::close(fd);
        return -1;
    }
    return fd;
}

// 버퍼에 자리가 날 때까지 막히므로 이 쓰기가 소리 스레드의 박자를 맞춘다.
bool write_frames(int fd, const int16_t *data, size_t frames) {
    while (frames > 0) {
        snd_xferi transfer{};
        transfer.buf = const_cast<int16_t *>(data);
        transfer.frames = frames;
        if (ioctl(fd, SNDRV_PCM_IOCTL_WRITEI_FRAMES, &transfer) < 0) {
            if (errno == EINTR) continue;
            // 스레드가 60 ms 넘게 늦어 버퍼가 비었다(aplay도 이렇게 이어 갔다)
            if (errno == EPIPE && ioctl(fd, SNDRV_PCM_IOCTL_PREPARE) == 0) continue;
            return false;
        }
        data += transfer.result * kChannels;
        frames -= static_cast<size_t>(transfer.result);
    }
    return true;
}

}  // namespace

AlertSound::AlertSound() : pcm_(env_string("EDGEPILOT_ALERT_PCM", "/dev/snd/pcmC0D1p")) {
    if (!env_flag("EDGEPILOT_ALERT_SOUND", true)) return;
    set_volume_percent(env_float("EDGEPILOT_ALERT_VOLUME", 70.0f));
    // 100% 크기로 한 번 만들어 두고 재생할 때 volume_을 곱한다.
    for (int id = 0; id < static_cast<int>(AlertSoundId::Count); ++id)
        clips_.push_back(stereo(render_alert_tone(static_cast<AlertSoundId>(id), kRate)));
    enabled_ = true;
    thread_ = std::thread(&AlertSound::loop, this);
}

AlertSound::~AlertSound() {
    stop_ = true;
    if (thread_.joinable()) thread_.join();
    close_pcm();
}

void AlertSound::set_volume_percent(float percent) { volume_ = std::clamp(percent, 0.0f, 100.0f) / 100.0f; }

void AlertSound::play(AlertSoundId id) {
    if (enabled_) pending_ = static_cast<int>(id);
}

void AlertSound::close_pcm() {
    if (pcm_fd_ >= 0) ::close(pcm_fd_);  // 남은 버퍼는 무음이라 비우지 않고 닫는다
    pcm_fd_ = -1;
}

void AlertSound::loop() {
    const std::vector<int16_t> silence(kChunkFrames * kChannels, 0);
    const std::vector<int16_t> warmup(kWarmupFrames * kChannels, 0);
    std::vector<int16_t> scaled(silence.size());
    const std::vector<int16_t> *clip = nullptr;
    size_t position = 0;
    int quiet_chunks = 0;  // 알림이 끝난 뒤 흘린 무음 조각
    while (!stop_) {
        const int next = pending_.exchange(-1);
        if (next >= 0) {  // 새 알림은 재생 중인 알림을 끊는다
            clip = &clips_[static_cast<size_t>(next)];
            position = 0;
        }
        if (pcm_fd_ < 0) {
            if (!clip) {  // 낼 알림이 없으면 앰프를 끈 채 기다린다
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }
            pcm_fd_ = open_pcm(pcm_);
            if (pcm_fd_ < 0) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }
            quiet_chunks = 0;
            if (!write_frames(pcm_fd_, warmup.data(), kWarmupFrames)) {
                std::fprintf(stderr, "alert sound: write failed: %s\n", std::strerror(errno));
                close_pcm();
                continue;
            }
        }
        const int16_t *data = silence.data();
        size_t samples = silence.size();
        if (clip) {
            quiet_chunks = 0;
            samples = std::min(silence.size(), clip->size() - position);
            data = clip->data() + position;
            position += samples;
            if (position >= clip->size()) clip = nullptr;
            const float volume = volume_.load();
            if (volume < 1.0f) {
                for (size_t i = 0; i < samples; ++i)
                    scaled[i] = static_cast<int16_t>(std::lround(static_cast<float>(data[i]) * volume));
                data = scaled.data();
            }
        } else if (++quiet_chunks > kPeriods) {  // 알림의 마지막 조각까지 버퍼에서 다 나갔다: 앰프를 끈다
            close_pcm();
            continue;
        }
        if (!write_frames(pcm_fd_, data, samples / kChannels)) {
            std::fprintf(stderr, "alert sound: write failed: %s\n", std::strerror(errno));
            close_pcm();
        }
    }
}
