#ifndef DEVICE_SETTINGS_H
#define DEVICE_SETTINGS_H

/* 웹 기기 설정(params/display.json) 중 런타임이 읽는 값. modeld(카메라 장착)와 overlayd(알림음)가
 * 각자 메인 루프에서 poll()을 부른다. 1초에 한 번만 stat하고 파일이 바뀌었을 때만 다시 읽는다. */

#include "common/model_output.h"
#include "common/utils_file.h"
#include "common/utils_json.h"
#include "common/utils_process.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <utility>

struct DeviceSettings {
    float alert_volume_percent = NAN;  // 없으면 NAN(시작 크기 유지)
    float camera_offset_m = 0.0f;      // 가상 카메라를 오른쪽(+)으로: 차가 왼쪽으로 간다
    float camera_height_m = kModelHeight;
    bool hud_debug = false;            // HUD 진단 카드(예전 패널의 수치)
};

/* 허용 범위(벗어나면 클램프). 웹 편집기의 min/max와 같아야 한다(check_web_console.py가 대조).
 * 카메라 오프셋 ±0.35 m는 sunnypilot과 같은 한계(넘으면 물체가 기운다). */
inline constexpr JsonFloatField<DeviceSettings> kDeviceSettingsFloats[] = {
    {"alert_volume_percent", 0.0f, 100.0f, &DeviceSettings::alert_volume_percent},
    {"camera_offset_m", -0.35f, 0.35f, &DeviceSettings::camera_offset_m},
    {"camera_height_m", 0.8f, 2.0f, &DeviceSettings::camera_height_m},
};

inline constexpr JsonBoolField<DeviceSettings> kDeviceSettingsBools[] = {
    {"hud_debug", &DeviceSettings::hud_debug},
};

/* 표의 허용 범위로 자른다(환경 변수로 받은 값 등). */
inline float clamp_device_setting(float DeviceSettings::*member, float value)
{
    for (const auto &field : kDeviceSettingsFloats)
        if (field.member == member) return value < field.lo ? field.lo : value > field.hi ? field.hi : value;
    return value;
}

/* 다른 파라미터 파일과 같은 로더: 값이 잘못되면(숫자가 아님, NaN) 예외 대신 false와 error. */
inline bool load_device_settings(const std::string &path, DeviceSettings *out, std::string *error)
{
    DeviceSettings settings;
    if (!load_json_param_file(path, [&settings](const std::string &text) {
            parse_json_fields(text, kDeviceSettingsFloats, &settings);
            parse_json_fields(text, kDeviceSettingsBools, &settings);
        }, error))
        return false;
    *out = settings;
    return true;
}

class DeviceSettingsFile {
public:
    explicit DeviceSettingsFile(std::string path = param_path("display.json")) : path_(std::move(path)) {}

    /* 새로 읽었으면 true. 첫 호출은 파일이 있으면 항상 읽는다. 잘못된 파일은 한 번 알리고
     * 이전 값을 그대로 둔다(다시 바뀔 때 읽는다). */
    bool poll(uint64_t now_ns, DeviceSettings *out)
    {
        if (now_ns < next_check_ns_) return false;
        next_check_ns_ = now_ns + 1'000'000'000ULL;
        const FileStamp stamp = file_stamp(path_);
        if (!stamp.valid || (read_once_ && !(stamp != stamp_))) return false;
        stamp_ = stamp;
        read_once_ = true;
        std::string error;
        if (!load_device_settings(path_, out, &error)) {
            std::fprintf(stderr, "device settings %s: %s (keeping previous values)\n", path_.c_str(),
                         error.c_str());
            return false;
        }
        return true;
    }

private:
    std::string path_;
    FileStamp stamp_;
    bool read_once_ = false;
    uint64_t next_check_ns_ = 0;
};

#endif
