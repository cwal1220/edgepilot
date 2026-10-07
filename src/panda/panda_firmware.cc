#include "panda/panda_firmware.h"

#include "car/can_frame.h"
#include "panda/panda_protocol.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>

namespace {

constexpr uint64_t kControlMaxAgeNs = 1000000000ULL;
constexpr float kParkedMaxSpeedKph = 1.0f;

uint32_t read_le32(const uint8_t *p)
{
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

/* 버전 문자열 <대문자 3~8>-<영숫자 8>-<DEBUG|RELEASE>를 찾는다(펌웨어의 gitversion). 앞뒤가
 * 같은 꼴의 글자와 붙어 있으면 아니다. */
std::string find_version(const std::vector<uint8_t> &bytes)
{
    for (const char *suffix : {"-DEBUG", "-RELEASE"}) {
        const size_t suffix_len = std::strlen(suffix);
        auto it = bytes.begin();
        while ((it = std::search(it, bytes.end(), suffix, suffix + suffix_len)) != bytes.end()) {
            const size_t end = static_cast<size_t>(it - bytes.begin()) + suffix_len;
            ++it;
            const size_t hash_end = end - suffix_len;
            if (hash_end < 9 || (end < bytes.size() && std::isalnum(bytes[end]))) continue;
            const size_t hash_start = hash_end - 8;
            if (bytes[hash_start - 1] != '-') continue;
            if (!std::all_of(bytes.begin() + static_cast<long>(hash_start), bytes.begin() + static_cast<long>(hash_end),
                             [](uint8_t c) { return std::isalnum(c) != 0; }))
                continue;
            size_t name_start = hash_start - 1;
            while (name_start > 0 && std::isupper(bytes[name_start - 1]) && hash_start - 1 - name_start < 8) --name_start;
            const size_t name_len = hash_start - 1 - name_start;
            if (name_len < 3 || (name_start > 0 && std::isalnum(bytes[name_start - 1]))) continue;
            return std::string(bytes.begin() + static_cast<long>(name_start), bytes.begin() + static_cast<long>(end));
        }
    }
    return std::string();
}

void append_json_string(std::string *out, const std::string &value)
{
    out->push_back('"');
    for (const unsigned char c : value) {
        if (c == '"' || c == '\\') {
            out->push_back('\\');
            out->push_back(static_cast<char>(c));
        } else if (c < 0x20) {
            char escaped[8];
            std::snprintf(escaped, sizeof(escaped), "\\u%04x", c);
            out->append(escaped);
        } else {
            out->push_back(static_cast<char>(c));
        }
    }
    out->push_back('"');
}

const char *flash_state_name(PandaFlashState state)
{
    switch (state) {
    case PandaFlashState::Flashing: return "flashing";
    case PandaFlashState::Done: return "done";
    case PandaFlashState::Failed: return "failed";
    case PandaFlashState::Idle: break;
    }
    return "idle";
}

} // namespace

bool parse_panda_image(std::vector<uint8_t> bytes, PandaImage *image, std::string *error)
{
    auto fail = [&](const std::string &message) {
        if (error) *error = message;
        return false;
    };
    const size_t size = bytes.size();
    if (size <= kPandaSignatureBytes + 16) return fail("image too small (" + std::to_string(size) + " bytes)");
    if (size > kPandaAppMaxBytes)
        return fail("image of " + std::to_string(size) + " bytes does not fit the " +
                    std::to_string(kPandaAppMaxBytes) + "-byte application area");
    if (size % 4 != 0) return fail("image size " + std::to_string(size) + " is not a multiple of 4");
    const size_t body = size - kPandaSignatureBytes;
    if (read_le32(bytes.data()) != body)
        return fail("length field " + std::to_string(read_le32(bytes.data())) + " does not match " +
                    std::to_string(body) + " (not a signed panda image)");
    if (std::memcmp(bytes.data() + body - 8, "VERS", 4) != 0) return fail("no VERS trailer before the signature");
    std::string version = find_version(bytes);
    if (version.empty()) return fail("no firmware version string in the image");
    if (image) {
        image->bytes = std::move(bytes);
        image->version = std::move(version);
    }
    return true;
}

bool load_panda_image(const std::string &path, PandaImage *image, std::string *error)
{
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        if (error) *error = "cannot open " + path;
        return false;
    }
    std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    return parse_panda_image(std::move(bytes), image, error);
}

bool panda_hw_type_flashable(uint8_t hw_type)
{
    return hw_type == kPandaHwBlack || hw_type == kPandaHwUno || hw_type == kPandaHwDos;
}

const char *panda_hw_type_name(uint8_t hw_type)
{
    switch (hw_type) {
    case 1: return "white panda";
    case 2: return "grey panda";
    case kPandaHwBlack: return "black panda";
    case 4: return "pedal";
    case kPandaHwUno: return "uno";
    case kPandaHwDos: return "dos";
    case kPandaHwRed: return "red panda";
    default: return "unknown";
    }
}

bool panda_flash_allowed(const ControlState &control, uint64_t now_ns, std::string *reason)
{
    const char *code = nullptr;
    const uint64_t age = now_ns >= control.timestamp_ns ? now_ns - control.timestamp_ns : 0;
    if (control.timestamp_ns == 0 || age > kControlMaxAgeNs) {
        code = "control_stale";
    } else if (!control.vehicle_fresh) {
        code = "vehicle_stale";
    } else if (control.engaged || control.active) {
        code = "engaged";
    } else if (control.gear != kGearPark) {
        code = "not_park";
    } else if (std::max(control.ego_speed_kph, control.cluster_speed_kph) >= kParkedMaxSpeedKph) {
        code = "moving";
    }
    if (reason) *reason = code ? code : "";
    return code == nullptr;
}

std::string panda_status_json(const PandaLinkStatus &link, const PandaFlashStatus &flash, uint64_t now_ns)
{
    std::string out = "{\"stamp_ns\":" + std::to_string(now_ns) + ",\"mode\":";
    append_json_string(&out, link.mode);
    out += ",\"serial\":";
    append_json_string(&out, link.serial);
    out += ",\"hw_type\":" + std::to_string(link.hw_type) + ",\"hw_name\":";
    append_json_string(&out, link.mode == "app" ? panda_hw_type_name(link.hw_type) : "");
    out += ",\"firmware_version\":";
    append_json_string(&out, link.firmware_version);
    out += ",\"flash\":{\"state\":";
    append_json_string(&out, flash_state_name(flash.state));
    out += ",\"step\":";
    append_json_string(&out, flash.step);
    out += ",\"percent\":" + std::to_string(flash.percent) + ",\"error\":";
    append_json_string(&out, flash.error);
    out += ",\"detail\":";
    append_json_string(&out, flash.detail);
    out += ",\"version\":";
    append_json_string(&out, flash.version);
    out += ",\"stamp_ns\":" + std::to_string(flash.stamp_ns) + "}}\n";
    return out;
}
