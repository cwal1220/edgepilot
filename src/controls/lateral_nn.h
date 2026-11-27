#pragma once

// 학습한 조향 토크 모델(피드포워드). 학습은 별도 저장소 nnlc가 한다(docs/lateral-nn.md).
// - 모델 파일: 형식 "edgepilot-lateral-nn" 버전 1(JSON). 입력 이름·창 정의·정규화·망 가중치·시험 벡터를 담는다.
// - 입력은 응답 시각 t + lag 기준 궤적(원래 단위). 운용은 목표 궤적을 넣는다. 학습은 모델 파일의 training.inputs가
//   말한다: v3은 실제로 일어난 궤적, v4는 그 틱에 컨트롤러가 만들었을 운용 입력이다.
// - 출력은 지금 보낼 토크(제어 부호: 오른쪽이 양수, steer_max로 나눈 값)이고 output_clip 안이다.

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

// 입력이 읽을 수 있는 시각(지금부터 초). 과거는 LateralNnInputs의 이력(지금 + 99틱)이, 미래는 계획 궤적(2.5 s)에서
// 계획 나이 보정(최대 0.25 s)을 뺀 만큼이 채운다(lateral_controller.cc가 계획 격자와 맞는지 컴파일 때 확인한다).
// 모델이 이 밖을 읽으면 load가 거부한다.
constexpr float kLateralNnEarliestS = -0.99f;
constexpr float kLateralNnLatestS = 2.25f;
// 입력 수와 층 폭의 한계. 계산 버퍼를 스택에 두어 evaluate가 할당 없이, 여러 스레드에서 같이 쓸 수 있게 한다.
constexpr int kLateralNnMaxWidth = 64;

// 학습 쪽 dataset.Spec과 같다. 오프셋은 응답 시각 기준 초다. 입력 이름을 학습 쪽(파이썬)과 똑같이 만들려고 double로
// 둔다: 0.45 s는 double로 0.4500…01이라 "%+.1f"가 "+0.5"를 내지만 float로는 "+0.4"가 된다.
struct LateralNnSpec {
    enum class Extra { Angle, Long };  // 응답 시각의 조향각, 종가속
    double lag_s = 0.3;
    std::vector<double> la_offsets;  // 과거와 미래, 파일 순서 그대로
    std::vector<double> roll_offsets;
    std::vector<Extra> extra;
    std::vector<double> angle_offsets;  // 조향각 창(파일의 창을 순서대로 이어 붙인 것)
    // 학습 목표에서 운용이 곡선에 더하는 마찰 몫을 뺐다(nnlc Spec.friction_free, 파일에 없으면 거짓). 이 모델에만 곡선
    // 마찰을 더한다: 보낸 토크를 그대로 배운 모델에 더하면 마찰이 두 번 들어간다.
    bool friction_free = false;

    // 학습 쪽 Spec.names()와 같은 입력 이름(예: "la-0.5", "roll+1.5", "angle+0.30").
    std::vector<std::string> names() const;
};

class LateralNnModel {
public:
    // 파일을 읽어 검사한다(형식, 입력 이름, 읽을 수 있는 시각, 유한한 값, 파일의 시험 벡터를 이 계산으로 다시 내는지).
    // 실패하면 false와 이유. 성공해야 evaluate를 쓸 수 있다.
    bool load(const std::string &path, std::string *error);
    bool load_text(const std::string &text, std::string *error);

    bool loaded() const { return !nets_.empty(); }
    const std::string &name() const { return name_; }
    const LateralNnSpec &spec() const { return spec_; }
    size_t input_count() const { return mean_.size(); }

    // inputs: input_count()개, 원래 단위. 입력이나 망 출력이 NaN·무한대면 NaN을 돌려준다(호출자는 선형 피드포워드를
    // 쓴다).
    float evaluate(const float *inputs) const;

private:
    enum class Activation { Linear, Tanh };
    struct Layer {
        int out = 0;
        int in = 0;
        std::vector<float> weight;  // [in][out] 순서(파일은 [out][in]). 안쪽 반복이 출력끼리 독립이라 벡터화된다
        std::vector<float> bias;
        Activation activation = Activation::Linear;
    };

    std::string name_;
    LateralNnSpec spec_;
    std::vector<float> mean_;
    std::vector<float> std_;
    std::vector<std::vector<Layer>> nets_;
    float clip_lo_ = -1.0f;
    float clip_hi_ = 1.0f;
};

// 운용 때 입력을 만든다. 호출자는 매 제어 틱(100 Hz)마다 push로 지금의 목표 값을 쌓고, 조향할 때 evaluate를 부른다.
// - la(δ), 조향각(δ): 지금부터 δ초 뒤의 목표 값. δ ≥ 0이면 호출자가 준 함수(계획 궤적)로, δ < 0이면 쌓아 둔 지난 틱의
//   지금 값으로 채운다(막 시작해 이력이 모자라면 가장 오래된 값).
// - jerk: 응답 시각 앞뒤 ±0.05 s 목표 la의 중앙차분(학습 쪽 series.py와 같다).
// - v: 지금 속도(학습도 표본 시각의 속도). roll: 편경사 추정의 횡가속 / g(학습 열 roll = bank / g. paramsd 롤과
//   부호가 반대다). 모든 창이 지금 값이다(미래 롤은 모른다).
// - long: 지금 종가속(v3 학습은 응답 시각의 값, v4는 지금 값이다).
class LateralNnInputs {
public:
    void push(float la_now, float angle_now);
    void reset() { count_ = 0; }

    // la_at, angle_at: float(float delta_s)로 부를 수 있는 것(람다). 템플릿이라 매 틱 std::function 할당이 없다.
    template <class LaAt, class AngleAt>
    float evaluate(const LateralNnModel &model, float speed_mps, float bank_over_g, float long_accel_mps2,
                   const LaAt &la_at, const AngleAt &angle_at);

    // 마지막 evaluate가 만든 입력(시험용)
    std::vector<float> last_inputs() const { return {inputs_.begin(), inputs_.begin() + input_count_}; }

private:
    static constexpr int kHistory = 100;  // 1 s, 100 Hz
    float history(const float *buffer, float delta_s) const;

    float la_[kHistory] = {};
    float angle_[kHistory] = {};
    int head_ = 0;
    int count_ = 0;
    std::array<float, kLateralNnMaxWidth> inputs_{};
    int input_count_ = 0;
};

template <class LaAt, class AngleAt>
float LateralNnInputs::evaluate(const LateralNnModel &model, float speed_mps, float bank_over_g, float long_accel_mps2,
                                const LaAt &la_at, const AngleAt &angle_at) {
    const LateralNnSpec &s = model.spec();
    const auto la = [&](float delta) { return delta >= 0.0f ? la_at(delta) : history(la_, delta); };
    const auto angle = [&](float delta) { return delta >= 0.0f ? angle_at(delta) : history(angle_, delta); };
    const float t = static_cast<float>(s.lag_s);
    int n = 0;
    const auto put = [&](float value) {
        if (n < kLateralNnMaxWidth) inputs_[static_cast<size_t>(n)] = value;
        ++n;
    };
    put(speed_mps);
    put(la(t));
    put((la(t + 0.05f) - la(t - 0.05f)) / 0.1f);
    put(bank_over_g);
    for (double o : s.la_offsets) put(la(t + static_cast<float>(o)));
    for (size_t i = 0; i < s.roll_offsets.size(); ++i) put(bank_over_g);
    for (LateralNnSpec::Extra e : s.extra) put(e == LateralNnSpec::Extra::Angle ? angle(t) : long_accel_mps2);
    for (double o : s.angle_offsets) put(angle(t + static_cast<float>(o)));
    input_count_ = std::min(n, kLateralNnMaxWidth);
    if (n != static_cast<int>(model.input_count())) return NAN;
    return model.evaluate(inputs_.data());
}
