#include "controls/lateral_nn.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "common/json_value.h"
#include "common/utils_file.h"
#include "common/utils_time.h"  // kDtCtrlF

namespace {

constexpr const char *kFormat = "edgepilot-lateral-nn";
constexpr int kVersion = 1;
// 파일의 시험 벡터와 이 계산의 차이 한계(학습 쪽 float32 계산과의 차이는 1e-7 안팎이다)
constexpr float kTestVectorTolerance = 2e-4f;

std::string offset_name(const std::string &prefix, double offset, int decimals) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), decimals == 1 ? "%+.1f" : "%+.2f", offset);
    return prefix + buf;
}

bool fail(std::string *error, const std::string &message) {
    if (error) *error = message;
    return false;
}

// 숫자 배열. float로 읽을 때는 float로도 유한해야 한다(1e39는 double로는 유한하지만 float로는 무한대다).
template <class T>
bool read_numbers(const JsonValue *value, std::vector<T> *out) {
    if (!value || !value->is_array()) return false;
    out->clear();
    for (const JsonValue &v : value->array()) {
        if (!v.is_number() || !std::isfinite(static_cast<T>(v.number()))) return false;
        out->push_back(static_cast<T>(v.number()));
    }
    return true;
}

// tanh(x) = 1 - 2/(e^2x + 1). 보드(A53, glibc 2.35)에서 tanhf는 한 번에 약 200사이클이라 은닉 192개(망 셋)에
// 32 µs가 드는데, expf로는 12 µs다. 오차는 1에 대한 반올림(약 1e-7)이고 |x|가 커도 ±1로 끝난다.
float tanh_via_exp(float x) { return 1.0f - 2.0f / (std::exp(2.0f * x) + 1.0f); }

}  // namespace

std::vector<std::string> LateralNnSpec::names() const {
    std::vector<std::string> out = {"v", "la", "jerk", "roll"};
    for (double o : la_offsets) out.push_back(offset_name("la", o, 1));
    for (double o : roll_offsets) out.push_back(offset_name("roll", o, 1));
    for (Extra e : extra) out.push_back(e == Extra::Angle ? "angle" : "long");
    for (double o : angle_offsets) out.push_back(offset_name("angle", o, 2));
    return out;
}

bool LateralNnModel::load(const std::string &path, std::string *error) {
    const std::string text = read_text_file(path);
    if (text.empty()) return fail(error, "cannot open " + path);
    if (!load_text(text, error)) {
        if (error) *error = path + ": " + *error;
        return false;
    }
    return true;
}

bool LateralNnModel::load_text(const std::string &text, std::string *error) {
    nets_.clear();
    JsonValue root;
    std::string parse_error;
    if (!JsonValue::parse(text, &root, &parse_error)) return fail(error, "bad JSON: " + parse_error);
    const JsonValue *format = root.find("format");
    const JsonValue *version = root.find("version");
    if (!format || !format->is_string() || format->string() != kFormat || !version || !version->is_number() ||
        version->number() != kVersion)
        return fail(error, std::string("not an ") + kFormat + " v" + std::to_string(kVersion) + " file");
    const JsonValue *name = root.find("name");
    std::string model_name = name && name->is_string() ? name->string() : std::string();

    // 창 정의. 학습 쪽 Spec 필드 이름 그대로다.
    const JsonValue *spec = root.find("spec");
    if (!spec || !spec->is_object()) return fail(error, "no spec");
    LateralNnSpec s;
    std::vector<double> past, future;
    const JsonValue *lag = spec->find("lag_s");
    if (!lag || !lag->is_number() || !read_numbers(spec->find("la_past"), &past) ||
        !read_numbers(spec->find("la_future"), &future))
        return fail(error, "spec: lag_s, la_past, la_future");
    s.lag_s = lag->number();
    s.la_offsets = past;
    s.la_offsets.insert(s.la_offsets.end(), future.begin(), future.end());
    if (!read_numbers(spec->find("roll_past"), &past) || !read_numbers(spec->find("roll_future"), &future))
        return fail(error, "spec: roll_past, roll_future");
    s.roll_offsets = past;
    s.roll_offsets.insert(s.roll_offsets.end(), future.begin(), future.end());
    const JsonValue *extra = spec->find("extra");
    if (!extra || !extra->is_array()) return fail(error, "spec: extra");
    for (const JsonValue &e : extra->array()) {
        if (!e.is_string() || (e.string() != "angle" && e.string() != "long"))
            return fail(error, "spec: unsupported extra input");
        s.extra.push_back(e.string() == "angle" ? LateralNnSpec::Extra::Angle : LateralNnSpec::Extra::Long);
    }
    const JsonValue *windows = spec->find("extra_windows");
    if (!windows || !windows->is_array()) return fail(error, "spec: extra_windows");
    for (const JsonValue &w : windows->array()) {
        std::vector<double> offsets;
        if (!w.is_array() || w.array().size() != 2 || !w.array()[0].is_string() || w.array()[0].string() != "angle" ||
            !read_numbers(&w.array()[1], &offsets))
            return fail(error, "spec: unsupported window (only angle)");
        s.angle_offsets.insert(s.angle_offsets.end(), offsets.begin(), offsets.end());
    }
    // 마찰 몫은 v3에서 더한 항목이라 없으면 거짓이다(그 전 모델은 보낸 토크를 배웠다)
    const JsonValue *friction_free = spec->find("friction_free");
    if (friction_free && !friction_free->is_bool()) return fail(error, "spec: friction_free");
    s.friction_free = friction_free && friction_free->boolean();
    // 입력이 읽는 시각(지금부터): la와 조향각은 응답 시각 + 오프셋, 저크는 응답 시각 ±0.05 s. 롤은 지금 값이라 상관없다.
    std::vector<double> times = {s.lag_s - 0.05, s.lag_s + 0.05};
    for (double o : s.la_offsets) times.push_back(s.lag_s + o);
    for (double o : s.angle_offsets) times.push_back(s.lag_s + o);
    for (double t : times) {
        if (!std::isfinite(t) || t < kLateralNnEarliestS - 1e-6 || t > kLateralNnLatestS + 1e-6)
            return fail(error, "spec: an input reads " + std::to_string(t) + " s, outside the history and the plan");
    }

    // 입력 이름이 창 정의와 같아야 한다(학습 쪽과 이름 규칙이 어긋나면 여기서 막힌다)
    const JsonValue *features = root.find("features");
    const std::vector<std::string> expected = s.names();
    if (expected.size() > static_cast<size_t>(kLateralNnMaxWidth)) return fail(error, "too many inputs");
    if (!features || !features->is_array() || features->array().size() != expected.size())
        return fail(error, "features do not match spec");
    for (size_t i = 0; i < expected.size(); ++i) {
        const JsonValue &f = features->array()[i];
        if (!f.is_string() || f.string() != expected[i])
            return fail(error, "feature " + std::to_string(i) + " is not " + expected[i]);
    }

    std::vector<float> mean, stdev, clip;
    if (!read_numbers(root.find("input_mean"), &mean) || !read_numbers(root.find("input_std"), &stdev) ||
        mean.size() != expected.size() || stdev.size() != expected.size())
        return fail(error, "input_mean / input_std");
    for (float v : stdev) {
        if (!(v > 0.0f)) return fail(error, "input_std must be positive");
    }
    if (!read_numbers(root.find("output_clip"), &clip) || clip.size() != 2 || !(clip[0] < clip[1]) || clip[0] < -1.0f ||
        clip[1] > 1.0f)
        return fail(error, "output_clip must be inside [-1, 1]");

    const JsonValue *nets = root.find("nets");
    if (!nets || !nets->is_array() || nets->array().empty()) return fail(error, "no nets");
    std::vector<std::vector<Layer>> parsed;
    for (const JsonValue &net : nets->array()) {
        const JsonValue *layers = net.find("layers");
        if (!layers || !layers->is_array() || layers->array().empty()) return fail(error, "net without layers");
        std::vector<Layer> out;
        int width = static_cast<int>(expected.size());
        for (const JsonValue &l : layers->array()) {
            Layer layer;
            const JsonValue *weight = l.find("weight");
            if (!weight || !weight->is_array() || weight->array().empty() ||
                weight->array().size() > static_cast<size_t>(kLateralNnMaxWidth))
                return fail(error, "layer weight");
            layer.out = static_cast<int>(weight->array().size());
            layer.in = width;
            layer.weight.resize(static_cast<size_t>(layer.out) * static_cast<size_t>(layer.in));
            std::vector<float> r;
            for (int o = 0; o < layer.out; ++o) {
                if (!read_numbers(&weight->array()[static_cast<size_t>(o)], &r) || static_cast<int>(r.size()) != width)
                    return fail(error, "layer weight shape");
                for (int i = 0; i < width; ++i)
                    layer.weight[static_cast<size_t>(i) * static_cast<size_t>(layer.out) + static_cast<size_t>(o)] =
                        r[static_cast<size_t>(i)];
            }
            if (!read_numbers(l.find("bias"), &layer.bias) || static_cast<int>(layer.bias.size()) != layer.out)
                return fail(error, "layer bias shape");
            const JsonValue *act = l.find("activation");
            if (!act || !act->is_string()) return fail(error, "layer activation");
            if (act->string() == "tanh")
                layer.activation = Activation::Tanh;
            else if (act->string() == "linear")
                layer.activation = Activation::Linear;
            else
                return fail(error, "unsupported activation " + act->string());
            width = layer.out;
            out.push_back(std::move(layer));
        }
        if (width != 1) return fail(error, "net output must be 1");
        parsed.push_back(std::move(out));
    }

    // 시험 벡터: 학습 쪽이 같은 입력으로 낸 출력. 이 계산이 그대로 내야 쓴다(형식이나 값이 어긋난 파일을 막는다).
    const JsonValue *vectors = root.find("test_vectors");
    if (!vectors || !vectors->is_array() || vectors->array().empty()) return fail(error, "no test_vectors");
    name_ = std::move(model_name);
    spec_ = std::move(s);
    mean_ = std::move(mean);
    std_ = std::move(stdev);
    clip_lo_ = clip[0];
    clip_hi_ = clip[1];
    nets_ = std::move(parsed);
    for (size_t k = 0; k < vectors->array().size(); ++k) {
        const JsonValue &v = vectors->array()[k];
        std::vector<float> in;
        const JsonValue *expected_out = v.find("output");
        const bool shaped = read_numbers(v.find("input"), &in) && in.size() == mean_.size() && expected_out &&
                            expected_out->is_number();
        const float got = shaped ? evaluate(in.data()) : NAN;
        if (!shaped || !(std::fabs(got - static_cast<float>(expected_out->number())) <= kTestVectorTolerance)) {
            nets_.clear();
            return fail(error, "test vector " + std::to_string(k) + " does not match");
        }
    }
    return true;
}

float LateralNnModel::evaluate(const float *inputs) const {
    if (!loaded()) return NAN;
    const int n = static_cast<int>(mean_.size());
    float x[kLateralNnMaxWidth], a[kLateralNnMaxWidth], b[kLateralNnMaxWidth];
    for (int i = 0; i < n; ++i) {
        if (!std::isfinite(inputs[i])) return NAN;
        x[i] = (inputs[i] - mean_[static_cast<size_t>(i)]) / std_[static_cast<size_t>(i)];
    }
    float sum = 0.0f;
    for (const std::vector<Layer> &net : nets_) {
        const float *in = x;
        for (const Layer &layer : net) {
            float *next = in == a ? b : a;
            // 출력마다 편향에서 시작해 입력 순서대로 더한다(행마다 내적하는 것과 같은 순서)
            std::copy(layer.bias.begin(), layer.bias.end(), next);
            for (int i = 0; i < layer.in; ++i) {
                const float v = in[i];
                const float *w = layer.weight.data() + static_cast<size_t>(i) * static_cast<size_t>(layer.out);
                for (int o = 0; o < layer.out; ++o) next[o] += w[o] * v;
            }
            if (layer.activation == Activation::Tanh) {
                for (int o = 0; o < layer.out; ++o) next[o] = tanh_via_exp(next[o]);
            }
            in = next;
        }
        sum += in[0];
    }
    const float out = sum / static_cast<float>(nets_.size());
    // fmin/fmax는 NaN을 버리고 한계를 낸다. 망이 무너지면 한계 토크 대신 NaN으로 알린다.
    if (!std::isfinite(out)) return NAN;
    return std::fmin(std::fmax(out, clip_lo_), clip_hi_);
}

void LateralNnInputs::push(float la_now, float angle_now) {
    head_ = (head_ + 1) % kHistory;
    la_[head_] = la_now;
    angle_[head_] = angle_now;
    if (count_ < kHistory) ++count_;
}

float LateralNnInputs::history(const float *buffer, float delta_s) const {
    // delta_s < 0: |delta_s|초 전 틱의 값(load가 이력 안만 읽는 모델만 받는다). 막 시작해 모자라면 가장 오래된 값.
    if (count_ == 0) return NAN;
    const int back = std::min(static_cast<int>(std::lround(-delta_s / kDtCtrlF)), count_ - 1);
    return buffer[(head_ - back + kHistory) % kHistory];
}
