#ifndef PANDA_PROTOCOL_H
#define PANDA_PROTOCOL_H

/* firmware/panda가 USB로 말하는 것: 장치 ID, 요청 코드, 패킷 버전, health 패킷, 앱 플래시 배치.
 * pandad·플래셔가 쓰고, gtest_panda_firmware가 펌웨어 헤더(board/health.h 등)와 대조한다. */

#include <cstddef>
#include <cstdint>

constexpr uint16_t kPandaVendorId = 0xbbaa;
constexpr uint16_t kPandaAppProductId = 0xddcc;       // 앱(펌웨어)이 돌 때
constexpr uint16_t kPandaBootstubProductId = 0xddee;  // bootstub(플래셔)에 머물 때

// 벤더 요청 코드(board/usb_comms.h, board/flasher.h)
constexpr uint8_t kPandaRequestFlasherEcho = 0xb0;   // bootstub: 응답 [4:8]이 DE AD D0 0D, [8:12]가 기록 위치
constexpr uint8_t kPandaRequestFlashUnlock = 0xb1;   // bootstub: 플래시 잠금 해제, 기록 위치를 앱 시작으로
constexpr uint8_t kPandaRequestFlashErase = 0xb2;    // bootstub: wValue 섹터 지우기(0은 거부)
constexpr uint8_t kPandaRequestHwType = 0xc1;
constexpr uint8_t kPandaRequestEnterBootloader = 0xd1;  // wValue 1: bootstub, 0: STM32 DFU
constexpr uint8_t kPandaRequestHealth = 0xd2;
constexpr uint8_t kPandaRequestVersion = 0xd6;       // 버전 문자열(최대 64바이트)
constexpr uint8_t kPandaRequestReset = 0xd8;
constexpr uint8_t kPandaRequestSafetyModel = 0xdc;
constexpr uint8_t kPandaRequestPacketVersions = 0xdd;
constexpr uint8_t kPandaRequestHeartbeat = 0xf3;

constexpr uint8_t kPandaFlashEndpoint = 0x02;        // bootstub: 앱 이미지를 4바이트 단위로 받는 bulk OUT
constexpr uint8_t kPandaFlasherMagic[4] = {0xde, 0xad, 0xd0, 0x0d};
constexpr size_t kPandaVersionMaxBytes = 64;

// board/safety.h의 SAFETY_*
constexpr uint16_t kPandaSafetySilent = 0;
constexpr uint16_t kPandaSafetyElm327 = 3;
constexpr uint16_t kPandaSafetyHyundai = 8;
constexpr uint16_t kPandaSafetyAllOutput = 17;
constexpr uint16_t kPandaSafetyNoOutput = 19;
constexpr uint16_t kPandaSafetyHyundaiCommunity = 24;

constexpr uint8_t kPandaHealthPacketVersion = 7;
constexpr uint8_t kPandaCanPacketVersion = 2;

// board/boards/board_declarations.h의 HW_TYPE_*
constexpr uint8_t kPandaHwBlack = 3;
constexpr uint8_t kPandaHwUno = 5;
constexpr uint8_t kPandaHwDos = 6;
constexpr uint8_t kPandaHwRed = 7;

// STM32F413 앱 영역: bootstub이 섹터 0(0x08000000)에 있고 앱은 섹터 1~3(각 16 KB)에 쓴다.
constexpr uint32_t kPandaAppStart = 0x08004000;
constexpr uint16_t kPandaAppFirstSector = 1;
constexpr uint16_t kPandaAppLastSector = 3;
constexpr size_t kPandaAppMaxBytes = 3 * 16 * 1024;
constexpr size_t kPandaSignatureBytes = 128;

// health 패킷(board/health.h의 health_t, 버전 7)
struct __attribute__((packed)) PandaHealthPacket {
    uint32_t uptime_pkt;
    uint32_t voltage_pkt;
    uint32_t current_pkt;
    uint32_t can_rx_errs_pkt;
    uint32_t can_send_errs_pkt;
    uint32_t can_fwd_errs_pkt;
    uint32_t gmlan_send_errs_pkt;
    uint32_t faults_pkt;
    uint8_t ignition_line_pkt;
    uint8_t ignition_can_pkt;
    uint8_t controls_allowed_pkt;
    uint8_t gas_interceptor_detected_pkt;
    uint8_t car_harness_status_pkt;
    uint8_t usb_power_mode_pkt;
    uint8_t safety_mode_pkt;
    uint16_t safety_param_pkt;
    uint8_t fault_status_pkt;
    uint8_t power_save_enabled_pkt;
    uint8_t heartbeat_lost_pkt;
    uint16_t alternative_experience_pkt;
    uint32_t blocked_msg_cnt_pkt;
    float interrupt_load;
};

#endif
