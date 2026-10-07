/* panda_flash: 런타임을 멈춘 채 판다에 펌웨어를 쓴다(웹 콘솔을 쓸 수 없을 때, 복구용).
 * 런타임이 돌 때는 웹 콘솔의 기기 설정 탭에서 쓴다(pandad가 차 상태를 확인하고 쓴다).
 *
 * 사용: panda_flash [--yes] [이미지]   (기본 firmware/panda.bin.signed)
 *   --yes 없이는 이미지와 판다를 보여 주기만 한다. 판다가 두 번 재부팅하는 약 10초 동안 하네스가
 *   순정 카메라 배선으로 돌아가고 조향 제어가 끊기므로 주차한 차에서만 쓴다. */
#include "panda/panda_firmware.h"
#include "panda/panda_flasher.h"

#include <cstdio>
#include <cstring>
#include <string>

namespace {

const char *mode_name(PandaUsbMode mode)
{
    switch (mode) {
    case PandaUsbMode::App: return "application";
    case PandaUsbMode::Bootstub: return "bootstub (no application running)";
    case PandaUsbMode::None: break;
    }
    return "not found";
}

} // namespace

int main(int argc, char **argv)
{
    bool yes = false;
    std::string path = "firmware/panda.bin.signed";
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--yes") == 0) {
            yes = true;
        } else if (argv[i][0] == '-') {
            std::fprintf(stderr, "usage: %s [--yes] [image]\n", argv[0]);
            return 2;
        } else {
            path = argv[i];
        }
    }

    PandaImage image;
    std::string error;
    if (!load_panda_image(path, &image, &error)) {
        std::fprintf(stderr, "panda_flash: %s\n", error.c_str());
        return 1;
    }
    std::printf("image: %s (%s, %zu bytes)\n", path.c_str(), image.version.c_str(), image.bytes.size());
    const PandaUsbMode mode = panda_usb_mode();
    std::printf("panda: %s\n", mode_name(mode));
    if (mode == PandaUsbMode::None) return 1;
    if (!yes) {
        std::printf("Run again with --yes to flash. Do it parked: the Panda reboots twice (about 10 s) and the\n"
                    "harness falls back to the stock camera wiring meanwhile. Stop the runtime first.\n");
        return 2;
    }

    const PandaFlashResult result = flash_panda(image, [](const char *step, int percent) {
        std::printf("  %-8s %3d%%\n", step, percent);
        std::fflush(stdout);
    });
    if (!result.ok) {
        std::fprintf(stderr, "panda_flash: failed: %s (%s)\n", result.error.c_str(), result.detail.c_str());
        return 1;
    }
    std::printf("done: the panda runs %s\n", result.version.c_str());
    return 0;
}
