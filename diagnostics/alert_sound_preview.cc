// 알림음 미리 듣기. overlayd가 보드 스피커로 내는 소리(alert_tones.h)를 100% 크기 48 kHz 모노
// WAV로 쓴다. 보드에서는 pkill -USR1 overlayd 할 때마다 하나씩 차례로 울린다.
// 사용: alert_sound_preview [--out PREFIX]  →  PREFIX_<이름>.wav
#include "cli_args.h"
#include "hud/alert_tones.h"

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace {

constexpr int kRate = 48000;

void put_u32(std::vector<uint8_t> &out, uint32_t v) {
    for (int i = 0; i < 4; ++i) out.push_back(static_cast<uint8_t>(v >> (8 * i)));
}

void put_u16(std::vector<uint8_t> &out, uint16_t v) {
    out.push_back(static_cast<uint8_t>(v));
    out.push_back(static_cast<uint8_t>(v >> 8));
}

bool write_wav(const std::string &path, const std::vector<int16_t> &pcm) {
    const uint32_t data_bytes = static_cast<uint32_t>(pcm.size() * 2);
    std::vector<uint8_t> out;
    out.insert(out.end(), {'R', 'I', 'F', 'F'});
    put_u32(out, 36 + data_bytes);
    out.insert(out.end(), {'W', 'A', 'V', 'E', 'f', 'm', 't', ' '});
    put_u32(out, 16);
    put_u16(out, 1);  // PCM
    put_u16(out, 1);  // 모노
    put_u32(out, kRate);
    put_u32(out, kRate * 2);
    put_u16(out, 2);
    put_u16(out, 16);
    out.insert(out.end(), {'d', 'a', 't', 'a'});
    put_u32(out, data_bytes);
    for (int16_t s : pcm) put_u16(out, static_cast<uint16_t>(s));
    FILE *file = std::fopen(path.c_str(), "wb");
    if (!file) return false;
    const bool ok = std::fwrite(out.data(), 1, out.size(), file) == out.size();
    std::fclose(file);
    return ok;
}

}  // namespace

int main(int argc, char **argv) {
    std::string prefix = "alert";
    ArgReader args(argc, argv);
    while (args.next()) {
        if (args.is("--out"))
            prefix = args.value();
        else
            args.fail();
    }
    if (args.error()) {
        std::fprintf(stderr, "usage: %s [--out PREFIX]\n", argv[0]);
        return 2;
    }
    for (int i = 0; i < static_cast<int>(AlertSoundId::Count); ++i) {
        const AlertSoundId id = static_cast<AlertSoundId>(i);
        const std::vector<int16_t> pcm = render_alert_tone(id, kRate);
        const std::string path = prefix + "_" + alert_sound_name(id) + ".wav";
        if (!write_wav(path, pcm)) {
            std::fprintf(stderr, "cannot write %s\n", path.c_str());
            return 1;
        }
        std::printf("%s  %.2f s\n", path.c_str(), static_cast<double>(pcm.size()) / kRate);
    }
    return 0;
}
