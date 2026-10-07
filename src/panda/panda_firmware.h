#ifndef PANDA_FIRMWARE_H
#define PANDA_FIRMWARE_H

/* firmware/panda가 만든 앱 이미지(panda.bin.signed)를 검사하고, 플래싱해도 되는 차 상태인지
 * 가리고, pandad가 웹 콘솔에 알리는 상태를 JSON으로 만든다. USB를 쓰지 않아 호스트에서도
 * 빌드한다. 실제로 쓰는 것은 panda_flasher(보드 전용)다. */

#include "common/ipc_messages.h"

#include <cstdint>
#include <string>
#include <vector>

struct PandaImage {
    std::vector<uint8_t> bytes;
    std::string version;  // 이미지 안의 펌웨어 버전 문자열(예: EDGE-f9907afb-DEBUG)
};

/* 앱 이미지 검사: 앱 영역(섹터 1~3) 안에 들고 4바이트 단위이며, 머리의 길이 필드가 서명(128바이트)을
 * 뺀 길이와 같고 그 끝이 "VERS"+버전 번호이며, 버전 문자열이 들어 있어야 한다. 서명은 bootstub이
 * 검사한다(틀리면 앱으로 넘어가지 않고 bootstub에 머물러, 다시 쓸 수 있다). */
bool parse_panda_image(std::vector<uint8_t> bytes, PandaImage *image, std::string *error);
bool load_panda_image(const std::string &path, PandaImage *image, std::string *error);

// 이 이미지와 섹터 배치를 쓰는 F413 보드(black panda, uno, dos)인가
bool panda_hw_type_flashable(uint8_t hw_type);
const char *panda_hw_type_name(uint8_t hw_type);

/* 플래싱해도 되는 차 상태인가. 판다가 두 번 재부팅하는 동안(약 10초) 하네스가 순정 카메라 배선으로
 * 돌아가고 조향 제어가 끊기므로 주차 중에만 쓴다: 제어 상태가 1초 안에 나왔고, 차 상태가 살아
 * 있고, 조향·결합이 꺼져 있고, P단에 서 있어야 한다. 아니면 reason에 이유 코드를 둔다
 * (control_stale, vehicle_stale, engaged, not_park, moving). */
bool panda_flash_allowed(const ControlState &control, uint64_t now_ns, std::string *reason);

enum class PandaFlashState { Idle, Flashing, Done, Failed };

/* 웹 콘솔에 알리는 플래싱 진행. error는 Failed일 때의 이유 코드(panda_flash_allowed의 것과
 * image_invalid, image_changed(요청한 버전이 설치된 이미지와 다르다), no_panda, hw_unsupported,
 * 그리고 panda_flasher가 내는 것), detail은 로그용 설명이다. */
struct PandaFlashStatus {
    PandaFlashState state = PandaFlashState::Idle;
    std::string step;     // bootstub, erase, write, verify, reboot, check
    int percent = 0;
    std::string error;
    std::string detail;
    std::string version;  // 쓰는(쓴) 이미지의 버전
    uint64_t stamp_ns = 0;
};

// 판다 연결. mode: app(펌웨어가 돈다), bootstub(플래셔에 머문다), none
struct PandaLinkStatus {
    std::string mode = "none";
    std::string serial;
    uint8_t hw_type = 0;
    std::string firmware_version;
};

// pandad가 /dev/shm/edgepilot_panda_status.json에 쓰는 한 줄 JSON
std::string panda_status_json(const PandaLinkStatus &link, const PandaFlashStatus &flash, uint64_t now_ns);

#endif
