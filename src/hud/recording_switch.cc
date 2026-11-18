#include "hud/recording_switch.h"

#include "common/json_value.h"
#include "common/utils_file.h"
#include "common/utils_json.h"

#include <cctype>
#include <cerrno>
#include <cstring>

namespace {

// text에서 키 "enabled"의 값(true/false 글자)이 시작하는 자리와 길이. 없으면 false. 키는 따옴표로 감싼 이름 뒤에
// 공백과 ':'이 오는 것만 친다(값 문자열 안의 같은 글자는 건너뛴다).
bool find_enabled_literal(const std::string &text, size_t *begin, size_t *length) {
    static const std::string kKey = "\"enabled\"";
    for (size_t at = text.find(kKey); at != std::string::npos; at = text.find(kKey, at + 1)) {
        size_t i = at + kKey.size();
        while (i < text.size() && std::isspace(static_cast<unsigned char>(text[i]))) ++i;
        if (i >= text.size() || text[i] != ':') continue;
        ++i;
        while (i < text.size() && std::isspace(static_cast<unsigned char>(text[i]))) ++i;
        for (const char *literal : {"true", "false"}) {
            const size_t n = std::strlen(literal);
            if (text.compare(i, n, literal) == 0) {
                *begin = i;
                *length = n;
                return true;
            }
        }
        return false;
    }
    return false;
}

bool enabled_of(const std::string &text, bool *enabled, std::string *error) {
    JsonValue root;
    if (!parse_json_object(text, &root, error)) return false;
    const JsonValue *value = root.find("enabled");
    if (value == nullptr || !value->is_bool()) {
        if (error) *error = "enabled is not a boolean";
        return false;
    }
    *enabled = value->boolean();
    return true;
}

}  // namespace

bool toggle_recording_enabled(const std::string &path, bool *enabled, std::string *error) {
    const std::string text = read_text_file(path);
    if (text.empty()) {
        if (error) *error = "cannot read " + path;
        return false;
    }
    bool current = false;
    if (!enabled_of(text, &current, error)) return false;
    size_t begin = 0, length = 0;
    if (!find_enabled_literal(text, &begin, &length)) {
        if (error) *error = "enabled value not found";
        return false;
    }
    std::string next = text;
    next.replace(begin, length, current ? "false" : "true");
    bool flipped = current;
    if (!enabled_of(next, &flipped, error)) return false;
    if (flipped == current) {
        if (error) *error = "enabled did not change";
        return false;
    }
    if (!write_file_atomic(path, next)) {
        if (error) *error = "write " + path + ": " + std::strerror(errno);
        return false;
    }
    *enabled = flipped;
    return true;
}
