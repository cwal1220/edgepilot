#include "common/app_config.h"
#include "common/utils_math.h"
#include "common/utils_process.h"

#include <cstdio>
#include <stdexcept>

std::string AppConfig::usage(const char *program_name) {
    return std::string("Usage: ") + (program_name ? program_name : "modeld") + " <supercombo.axmodel>";
}

void AppConfig::set_warp_source(unsigned width, unsigned height) {
    const float w = static_cast<float>(width), h = static_cast<float>(height);
    input_warp_fx = camera_fx * w / static_cast<float>(kDefaultSensorWidth);
    input_warp_fy = camera_fy * h / static_cast<float>(kDefaultSensorHeight);
    input_warp_cx = camera_cx * w / static_cast<float>(kDefaultSensorWidth);
    input_warp_cy = camera_cy * h / static_cast<float>(kDefaultSensorHeight);
}

AppConfig AppConfig::from_env_defaults() {
    AppConfig config;

    const std::string intrinsics = env_string("EDGEPILOT_CAMERA_INTRINSICS");
    if (!intrinsics.empty()) {
        float v[4];
        if (std::sscanf(intrinsics.c_str(), "%f,%f,%f,%f", &v[0], &v[1], &v[2], &v[3]) != 4 || v[0] <= 0.0f ||
            v[1] <= 0.0f)
            throw std::runtime_error("EDGEPILOT_CAMERA_INTRINSICS must be fx,fy,cx,cy at 1920x1080");
        config.camera_fx = v[0];
        config.camera_fy = v[1];
        config.camera_cx = v[2];
        config.camera_cy = v[3];
    }
    config.set_warp_source(config.nv12_width, config.nv12_height);
    config.calibration_auto = env_flag("EDGEPILOT_CALIB_AUTO", true);
    config.profile = env_flag("EDGEPILOT_PROFILE");
    config.flow_calibration = env_flag("EDGEPILOT_FLOW_CALIB", true);
    return config;
}

AppConfig AppConfig::from_env(int argc, char *argv[]) {
    if (argc != 2) throw std::invalid_argument(usage(argc > 0 ? argv[0] : "modeld"));

    AppConfig config = from_env_defaults();
    config.axmodel_path = argv[1];
    return config;
}
