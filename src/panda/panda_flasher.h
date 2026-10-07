#ifndef PANDA_FLASHER_H
#define PANDA_FLASHER_H

/* 판다 앱 펌웨어를 USB로 쓴다(STM32F413, bootstub 경유, 보드 전용). 앱이 돌고 있으면 bootstub으로
 * 보낸 뒤 섹터 1~3을 지우고 이미지를 쓰고, bootstub이 받은 끝 주소를 확인하고 재부팅시켜, 앱이 이미지의
 * 버전으로 다시 올라오는 것까지 본다. bootstub(섹터 0)은 지우지 않으므로 중간에 끊겨도 판다는
 * bootstub에 머물고, 거기서 다시 쓰면 된다. 판다 USB 인터페이스를 쥔 다른 프로세스가 없어야 한다
 * (pandad는 자기 연결을 닫고 부른다). */

#include "panda/panda_firmware.h"

#include <functional>
#include <string>

enum class PandaUsbMode { None, App, Bootstub };

// 지금 붙어 있는 판다가 어느 쪽으로 열거됐나(장치 목록만 보고 열지는 않는다)
PandaUsbMode panda_usb_mode();

// 단계 이름(bootstub, erase, write, verify, reboot, check)과 그 단계의 진행률(0~100)
using PandaFlashProgress = std::function<void(const char *step, int percent)>;

struct PandaFlashResult {
    bool ok = false;
    std::string error;    // 이유 코드: no_panda, busy, hw_unsupported, usb_error, bootstub_timeout,
                          // flasher_missing, erase_failed, write_failed, verify_failed, app_timeout,
                          // version_mismatch
    std::string detail;   // 설명(로그용)
    std::string version;  // 다시 올라온 앱이 알린 버전
};

PandaFlashResult flash_panda(const PandaImage &image, const PandaFlashProgress &progress);

#endif
