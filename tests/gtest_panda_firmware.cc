// panda_firmware(이미지 검사, 플래싱 조건, 상태 JSON)와, pandad가 기대하는 프로토콜이 firmware/panda의
// 소스와 맞는지. 펌웨어의 health 패킷은 board/health.h를 그대로 가져와 대조한다.
#include "car/can_frame.h"
#include "panda/panda_firmware.h"
#include "panda/panda_protocol.h"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include "../firmware/panda/board/health.h"

namespace {

constexpr uint64_t kNow = 100000000000ULL;

// sign.py의 꼴: [본문 길이][코드 ... 버전 문자열 ...][VERS][2] 뒤에 서명 128바이트
std::vector<uint8_t> make_image(const std::string &version, size_t body = 1024)
{
    std::vector<uint8_t> bytes(body, 0x11);
    for (int i = 0; i < 4; ++i) bytes[static_cast<size_t>(i)] = static_cast<uint8_t>(body >> (8 * i));
    bytes[63] = 0;
    std::memcpy(bytes.data() + 64, version.c_str(), version.size() + 1);
    std::memcpy(bytes.data() + body - 8, "VERS", 4);
    bytes[body - 4] = 2;
    bytes[body - 3] = bytes[body - 2] = bytes[body - 1] = 0;
    bytes.insert(bytes.end(), kPandaSignatureBytes, 0xab);
    return bytes;
}

ControlState parked()
{
    ControlState control;
    control.timestamp_ns = kNow - 50000000ULL;
    control.vehicle_fresh = 1;
    control.gear = kGearPark;
    return control;
}

std::string source(const std::string &path)
{
    std::ifstream file("firmware/panda/" + path);
    std::stringstream text;
    text << file.rdbuf();
    return text.str();
}

long define_value(const std::string &path, const std::string &name)
{
    std::smatch match;
    const std::string text = source(path);
    if (!std::regex_search(text, match, std::regex("#define " + name + " (0x[0-9A-Fa-f]+|[0-9]+)U?\\b"))) return -1;
    return std::stol(match[1].str(), nullptr, 0);
}

} // namespace

TEST(PandaFirmware, ParsesASignedImage)
{
    PandaImage image;
    std::string error;
    ASSERT_TRUE(parse_panda_image(make_image("EDGE-f9907afb-DEBUG"), &image, &error)) << error;
    EXPECT_EQ(image.version, "EDGE-f9907afb-DEBUG");
    EXPECT_EQ(image.bytes.size(), 1024 + kPandaSignatureBytes);
}

TEST(PandaFirmware, ReadsTheOldFixedVersion)
{
    PandaImage image;
    ASSERT_TRUE(parse_panda_image(make_image("DEV-23456789-DEBUG"), &image, nullptr));
    EXPECT_EQ(image.version, "DEV-23456789-DEBUG");
}

TEST(PandaFirmware, RejectsImagesTheBootstubWouldNot)
{
    std::string error;
    EXPECT_FALSE(parse_panda_image(make_image("EDGE-f9907afb-DEBUG", kPandaAppMaxBytes), nullptr, &error));
    EXPECT_NE(error.find("does not fit"), std::string::npos) << error;

    std::vector<uint8_t> unaligned = make_image("EDGE-f9907afb-DEBUG");
    unaligned.push_back(0);
    EXPECT_FALSE(parse_panda_image(unaligned, nullptr, &error));

    std::vector<uint8_t> unsigned_image = make_image("EDGE-f9907afb-DEBUG");
    unsigned_image[0] ^= 4;
    EXPECT_FALSE(parse_panda_image(unsigned_image, nullptr, &error));
    EXPECT_NE(error.find("length field"), std::string::npos) << error;

    std::vector<uint8_t> no_trailer = make_image("EDGE-f9907afb-DEBUG");
    no_trailer[1024 - 8] = 'X';
    EXPECT_FALSE(parse_panda_image(no_trailer, nullptr, &error));

    EXPECT_FALSE(parse_panda_image(make_image("not a version"), nullptr, &error));
    EXPECT_FALSE(parse_panda_image(make_image("EDGE-f9907afb-DEBUGX"), nullptr, &error));
    EXPECT_FALSE(parse_panda_image(std::vector<uint8_t>(64, 0), nullptr, &error));
}

TEST(PandaFirmware, FlashesOnlyAParkedCar)
{
    std::string reason;
    EXPECT_TRUE(panda_flash_allowed(parked(), kNow, &reason));
    EXPECT_EQ(reason, "");

    ControlState control = parked();
    control.timestamp_ns = 0;
    EXPECT_FALSE(panda_flash_allowed(control, kNow, &reason));
    EXPECT_EQ(reason, "control_stale");
    control.timestamp_ns = kNow - 1500000000ULL;
    EXPECT_FALSE(panda_flash_allowed(control, kNow, &reason));
    EXPECT_EQ(reason, "control_stale");

    control = parked();
    control.vehicle_fresh = 0;  // gear 0은 P이기도 하고 모름이기도 하다
    EXPECT_FALSE(panda_flash_allowed(control, kNow, &reason));
    EXPECT_EQ(reason, "vehicle_stale");

    control = parked();
    control.engaged = 1;
    EXPECT_FALSE(panda_flash_allowed(control, kNow, &reason));
    EXPECT_EQ(reason, "engaged");
    control = parked();
    control.active = 1;
    EXPECT_FALSE(panda_flash_allowed(control, kNow, &reason));
    EXPECT_EQ(reason, "engaged");

    control = parked();
    control.gear = kGearDrive;
    EXPECT_FALSE(panda_flash_allowed(control, kNow, &reason));
    EXPECT_EQ(reason, "not_park");

    control = parked();
    control.ego_speed_kph = 3.0f;
    EXPECT_FALSE(panda_flash_allowed(control, kNow, &reason));
    EXPECT_EQ(reason, "moving");
    control = parked();
    control.cluster_speed_kph = 1.0f;
    EXPECT_FALSE(panda_flash_allowed(control, kNow, &reason));
    EXPECT_EQ(reason, "moving");
}

TEST(PandaFirmware, FlashesOnlyF413Boards)
{
    for (uint8_t hw : {kPandaHwBlack, kPandaHwUno, kPandaHwDos}) EXPECT_TRUE(panda_hw_type_flashable(hw)) << int(hw);
    for (uint8_t hw : {0, 1, 2, 4, 7, 8}) EXPECT_FALSE(panda_hw_type_flashable(static_cast<uint8_t>(hw))) << hw;
    EXPECT_STREQ(panda_hw_type_name(kPandaHwBlack), "black panda");
}

TEST(PandaFirmware, StatusJsonEscapesStrings)
{
    PandaLinkStatus link;
    link.mode = "app";
    link.serial = "a\"b\\c\n";
    link.hw_type = kPandaHwBlack;
    link.firmware_version = "EDGE-f9907afb-DEBUG";
    PandaFlashStatus flash;
    flash.state = PandaFlashState::Failed;
    flash.error = "not_park";
    const std::string json = panda_status_json(link, flash, 42);
    EXPECT_NE(json.find("\"serial\":\"a\\\"b\\\\c\\u000a\""), std::string::npos) << json;
    EXPECT_NE(json.find("\"hw_name\":\"black panda\""), std::string::npos) << json;
    EXPECT_NE(json.find("\"state\":\"failed\""), std::string::npos) << json;
    EXPECT_NE(json.find("\"error\":\"not_park\""), std::string::npos) << json;
    EXPECT_EQ(json.rfind("{\"stamp_ns\":42,", 0), 0U) << json;
}

#define EXPECT_SAME_OFFSET(field) EXPECT_EQ(offsetof(PandaHealthPacket, field), offsetof(health_t, field)) << #field

TEST(PandaProtocol, HealthPacketIsTheFirmwares)
{
    static_assert(sizeof(PandaHealthPacket) == sizeof(health_t), "pandad's health packet differs from health_t");
    EXPECT_EQ(HEALTH_PACKET_VERSION, kPandaHealthPacketVersion);
    EXPECT_SAME_OFFSET(uptime_pkt);
    EXPECT_SAME_OFFSET(voltage_pkt);
    EXPECT_SAME_OFFSET(faults_pkt);
    EXPECT_SAME_OFFSET(ignition_line_pkt);
    EXPECT_SAME_OFFSET(controls_allowed_pkt);
    EXPECT_SAME_OFFSET(safety_mode_pkt);
    EXPECT_SAME_OFFSET(safety_param_pkt);
    EXPECT_SAME_OFFSET(fault_status_pkt);
    EXPECT_SAME_OFFSET(heartbeat_lost_pkt);
    EXPECT_SAME_OFFSET(alternative_experience_pkt);
    EXPECT_SAME_OFFSET(blocked_msg_cnt_pkt);
    EXPECT_SAME_OFFSET(interrupt_load);
}

TEST(PandaProtocol, ConstantsMatchTheFirmwareSources)
{
    EXPECT_EQ(define_value("board/can_definitions.h", "CAN_PACKET_VERSION"), kPandaCanPacketVersion);
    EXPECT_EQ(define_value("board/panda.h", "USB_VID"), kPandaVendorId);
    const std::string usb = source("board/panda.h");
    EXPECT_NE(usb.find("#define USB_PID 0xDDEEU"), std::string::npos);  // bootstub
    EXPECT_NE(usb.find("#define USB_PID 0xDDCCU"), std::string::npos);  // 앱
    EXPECT_EQ(kPandaBootstubProductId, 0xddee);
    EXPECT_EQ(kPandaAppProductId, 0xddcc);
    EXPECT_EQ(define_value("board/stm32fx/stm32fx_config.h", "APP_START_ADDRESS"), static_cast<long>(kPandaAppStart));
    EXPECT_EQ(define_value("board/boards/board_declarations.h", "HW_TYPE_BLACK_PANDA"), kPandaHwBlack);
    EXPECT_EQ(define_value("board/boards/board_declarations.h", "HW_TYPE_UNO"), kPandaHwUno);
    EXPECT_EQ(define_value("board/boards/board_declarations.h", "HW_TYPE_DOS"), kPandaHwDos);
    EXPECT_EQ(define_value("board/safety.h", "SAFETY_SILENT"), kPandaSafetySilent);
    EXPECT_EQ(define_value("board/safety.h", "SAFETY_ELM327"), kPandaSafetyElm327);
    EXPECT_EQ(define_value("board/safety.h", "SAFETY_HYUNDAI"), kPandaSafetyHyundai);
    EXPECT_EQ(define_value("board/safety.h", "SAFETY_ALLOUTPUT"), kPandaSafetyAllOutput);
    EXPECT_EQ(define_value("board/safety.h", "SAFETY_NOOUTPUT"), kPandaSafetyNoOutput);
    EXPECT_EQ(define_value("board/safety.h", "SAFETY_HYUNDAI_COMMUNITY"), kPandaSafetyHyundaiCommunity);
    // bootstub의 flasher: 응답 매직과, 섹터 0(bootstub 자신)은 지우지 않는다는 것
    EXPECT_NE(source("board/flasher.h").find("\\xde\\xad\\xd0\\x0d"), std::string::npos);
    EXPECT_NE(source("board/stm32fx/llflash.h").find("if (sector != 0 && sector < 12 && unlocked)"), std::string::npos);
}
