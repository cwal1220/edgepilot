#pragma once

// 진단 도구 공용 옵션 값 읽기. std::atof·atoi·sscanf는 "abc"나 "0.3s", 모자란 목록을 말없이 0이나 앞부분으로 읽어,
// 오타가 있어도 기본값 대신 0으로 돌았다. 값 전체가 유한한 수(목록이면 정한 개수)이고 [lo, hi] 안일 때만 담고 true를
// 돌려준다. 도구는 false면 사용법 오류로 끝낸다.

#include <cerrno>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>

inline bool parse_double_arg(const char *text, double *out, double lo = -std::numeric_limits<double>::max(),
                             double hi = std::numeric_limits<double>::max()) {
    if (text == nullptr || *text == '\0') return false;
    char *end = nullptr;
    errno = 0;
    const double value = std::strtod(text, &end);
    if (errno != 0 || *end != '\0' || !std::isfinite(value) || value < lo || value > hi) return false;
    *out = value;
    return true;
}

inline bool parse_float_arg(const char *text, float *out, double lo = -std::numeric_limits<float>::max(),
                            double hi = std::numeric_limits<float>::max()) {
    double value = 0.0;
    if (!parse_double_arg(text, &value, lo, hi)) return false;
    *out = static_cast<float>(value);
    return true;
}

inline bool parse_long_arg(const char *text, long *out, long lo = LONG_MIN, long hi = LONG_MAX) {
    if (text == nullptr || *text == '\0') return false;
    char *end = nullptr;
    errno = 0;
    const long value = std::strtol(text, &end, 10);
    if (errno != 0 || *end != '\0' || value < lo || value > hi) return false;
    *out = value;
    return true;
}

inline bool parse_int_arg(const char *text, int *out, int lo = INT_MIN, int hi = INT_MAX) {
    long value = 0;
    if (!parse_long_arg(text, &value, lo, hi)) return false;
    *out = static_cast<int>(value);
    return true;
}

// "a,b,c"처럼 쉼표로 나눈 유한한 수 정확히 n개(8개까지).
inline bool parse_float_list_arg(const char *text, float *out, int n) {
    constexpr int kMaxValues = 8;
    if (text == nullptr || n < 1 || n > kMaxValues) return false;
    float values[kMaxValues];
    const char *p = text;
    for (int i = 0; i < n; ++i) {
        char *end = nullptr;
        errno = 0;
        const double value = std::strtod(p, &end);
        if (end == p || errno != 0 || !std::isfinite(value) || std::fabs(value) > std::numeric_limits<float>::max())
            return false;
        values[i] = static_cast<float>(value);
        const bool last = i + 1 == n;
        if (*end != (last ? '\0' : ',')) return false;
        p = end + 1;
    }
    for (int i = 0; i < n; ++i) out[i] = values[i];
    return true;
}

// 도구들의 옵션 읽기. while (args.next())로 인자를 하나씩 보고, is()로 이름을 맞춘 뒤 value()나 read()로 값을 받는다.
// 값이 없거나(마지막 인자) 수가 틀리면 error()가 켜진다. 모르는 --옵션은 도구가 fail()을 부르고, 나머지는 위치
// 인자(arg())다. error()면 도구는 사용법을 찍고 2로 끝낸다.
class ArgReader {
public:
    ArgReader(int argc, char **argv) : argc_(argc), argv_(argv) {}

    bool next() { return ++index_ < argc_; }
    const char *arg() const { return argv_[index_]; }
    bool is(const char *name) const { return std::strcmp(argv_[index_], name) == 0; }
    bool is_option() const { return std::strncmp(argv_[index_], "--", 2) == 0; }
    void fail() { error_ = true; }
    bool error() const { return error_; }

    // 옵션 값(다음 인자). 없으면 오류를 켜고 ""를 돌려준다.
    const char *value() { return has_value() ? argv_[++index_] : ""; }
    void read(double *out, double lo = -std::numeric_limits<double>::max(),
              double hi = std::numeric_limits<double>::max()) {
        if (has_value()) check(parse_double_arg(argv_[++index_], out, lo, hi));
    }
    void read(float *out, double lo = -std::numeric_limits<float>::max(),
              double hi = std::numeric_limits<float>::max()) {
        if (has_value()) check(parse_float_arg(argv_[++index_], out, lo, hi));
    }
    void read(long *out, long lo = LONG_MIN, long hi = LONG_MAX) {
        if (has_value()) check(parse_long_arg(argv_[++index_], out, lo, hi));
    }
    void read(int *out, int lo = INT_MIN, int hi = INT_MAX) {
        if (has_value()) check(parse_int_arg(argv_[++index_], out, lo, hi));
    }
    void read_list(float *out, int n) {
        if (has_value()) check(parse_float_list_arg(argv_[++index_], out, n));
    }

private:
    bool has_value() {
        if (index_ + 1 < argc_) return true;
        error_ = true;
        return false;
    }
    void check(bool ok) {
        if (ok) return;
        std::fprintf(stderr, "bad value for %s: %s\n", argv_[index_ - 1], argv_[index_]);
        error_ = true;
    }

    int argc_;
    char **argv_;
    int index_ = 0;
    bool error_ = false;
};
