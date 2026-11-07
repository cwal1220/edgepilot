#pragma once

// HUD 렌더러 내부 공용: 디자인 토큰(색·간격·카드 크기), 글자 줄 배치와 색 섞기, 상태 → 색, 앞차
// 정보, 그리고 렌더러가 나눈 영역의 그리기 함수(hud_scene.cc의 도로 장면, hud_cards.cc의
// 카드). hud_renderer.cc·hud_scene.cc·hud_cards.cc만 쓴다.

#include "common/model_output.h"
#include "common/projection.h"
#include "hud/hud_canvas.h"
#include "hud/hud_state.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

namespace hud_draw {

// ---- 디자인 토큰 ----

constexpr uint32_t kText = hud_argb(255, 255, 255, 255);
constexpr uint32_t kTextSecondary = hud_argb(235, 190, 196, 205);
constexpr uint32_t kCard = hud_argb(150, 12, 14, 18);
constexpr uint32_t kCardStrong = hud_argb(205, 12, 14, 18);
constexpr uint32_t kTrack = hud_argb(90, 255, 255, 255);
constexpr uint32_t kShadow = hud_argb(110, 0, 0, 0);
constexpr uint32_t kCarBody = hud_argb(70, 255, 255, 255);
constexpr uint32_t kDriverTorque = hud_argb(255, 90, 200, 250);  // 토크 바의 운전자 토크 눈금
constexpr uint32_t kSignalHousing = hud_argb(255, 58, 62, 70);   // 신호등 아이콘 몸체
constexpr uint32_t kGreen = hud_argb(255, 48, 209, 88);
constexpr uint32_t kYellow = hud_argb(255, 255, 204, 0);  // 깜빡이·비상등
constexpr uint32_t kBlue = hud_argb(255, 64, 156, 255);
constexpr uint32_t kGray = hud_argb(255, 150, 156, 165);
constexpr uint32_t kAmber = hud_argb(255, 255, 176, 32);
constexpr uint32_t kRed = hud_argb(255, 255, 69, 58);

constexpr int kMargin = 14;
constexpr int kGap = 8;
constexpr float kRadius = 10.0f;
constexpr int kBorder = 4;
constexpr int kCardPad = 10;
constexpr int kChipH = 30;
constexpr int kChipPadX = 12;
constexpr int kDot = 10;
constexpr int kTopCardH = 80;  // 왼쪽 위 설정 속도·기어 카드
constexpr int kSetCardW = 90;
constexpr int kGearCardW = 56;
constexpr int kNetworkCardMinW = 200;
constexpr int kTorqueBarW = 240;
constexpr int kTorqueBarH = 6;
constexpr int kAlertH = 76;
constexpr int kAlertMinW = 320;
constexpr int kAlertTextPad = 56;        // 알림 카드 폭 = 제목·설명 중 넓은 것 + 이 값(왼쪽 색 막대와 양쪽 여백)
constexpr float kAlertArrowGap = 18.0f;  // 알림 제목 끝에서 방향 화살표 가운데까지
constexpr float kAlertArrowW = 14.0f;
constexpr int kRowH = 19;  // 카드 안 이름·값 줄 간격
// 아래 모서리 카드(TPMS, 카메라 보정)와 보정 위 학습 카드. 모서리 카드는 위가 알림 카드, 아래가
// 토크 바와 맞는다. 알림 카드는 둘 사이(640 − 2·(14 + 126 + 8) = 344)에 들어가므로 제목·설명은 288 px까지다
// (gtest_hud_canvas가 모든 문구를 잰다).
constexpr int kCornerCardW = 126;
constexpr int kCornerCardH = kAlertH + kGap + kTorqueBarH;
constexpr int kStatusTouchW = 190;  // 상태 알약을 누른 것으로 보는 오른쪽 위 영역(손가락만큼 넉넉하게)
constexpr int kStatusTouchH = 76;
constexpr int kLeftColumnTouchW = 250;  // 왼쪽 열을 누른 것으로 보는 폭(진단 카드 대부분)
constexpr int kWeakWifiDbm = -75;

// ---- 글자와 색 ----

template <typename... Args>
std::string format_text(const char *format, Args... args) {
    char text[96];
    std::snprintf(text, sizeof(text), format, args...);
    return text;
}

inline uint32_t mix(uint32_t a, uint32_t b, float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    uint32_t out = 0;
    for (int shift = 0; shift < 32; shift += 8) {
        const float ca = static_cast<float>((a >> shift) & 0xff);
        const float cb = static_cast<float>((b >> shift) & 0xff);
        out |= static_cast<uint32_t>(std::lround(ca + (cb - ca) * t)) << shift;
    }
    return out;
}

// 줄 배치 기준 글리프: 대문자 H, 숫자만 있는 글꼴은 0.
inline const HudGlyph &cap_glyph(const HudFont &font) {
    const HudGlyph *cap = font.glyph('H');
    return cap && cap->h ? *cap : *font.glyph('0');
}

// 대문자 윗선을 cap_top에 두는 줄 위 y.
inline int cap_line(const HudFont &font, int cap_top) { return cap_top - cap_glyph(font).top; }

// 대문자 아랫선(기준선)을 baseline에 두는 줄 위 y. 크기가 다른 글꼴을 한 줄에 맞출 때 쓴다.
inline int base_line(const HudFont &font, int baseline) { return cap_line(font, baseline - cap_glyph(font).h); }

// 대문자 높이를 box_h 안 가운데에 두는 줄 위 y.
inline int centered_line_top(const HudFont &font, int box_y, int box_h) {
    return cap_line(font, box_y + (box_h - cap_glyph(font).h) / 2);
}

// ---- 상태 → 색 ----

// 지금 조향하는가: 활성이고 85도 위에서 쉬는 중이 아니다.
inline bool steering_now(const HudState &hud) { return hud.controller_active && !hud.steer_paused; }

// 화면 테두리와 모드 점의 색. 결합 전이면 0(테두리 없음). 결합했지만 조향을 쉬면 회색.
inline uint32_t state_color(const HudState &hud) {
    if (hud.steering_fault || hud.panda_faults != 0 || hud.soft_disabling) return kRed;
    if (!hud.services_healthy) return kAmber;
    if (steering_now(hud)) return hud.laneless_mode ? kBlue : kGreen;
    return hud.controller_engaged ? kGray : 0;
}

// ---- 앞차(hud_scene.cc) ----

struct LeadInfo {
    bool vision = false;
    bool radar = false;
    ParsedLeadPoint point{};
    float probability = 0.0f;
    float distance_m = 0.0f;
    float relative_speed_kph = 0.0f;
};

LeadInfo lead_info(const HudState &hud, const ParsedModelOutput &output);
// 앞차 거리(다가오면 상대속도도) 글자
std::string lead_text(const LeadInfo &lead);

// ---- 도로 장면(hud_scene.cc) ----

void draw_scene(HudCanvas &canvas, const ParsedModelOutput &output, const ProjectionState &projection,
                const HudState &hud, const LeadInfo &lead);
void draw_lane_position(HudCanvas &canvas, const ParsedModelOutput &output, const ProjectionState &projection,
                        float lane_center_offset_m, uint32_t car_color);

// ---- 카드(hud_cards.cc) ----

int draw_network_card(HudCanvas &canvas, int y, const HudState &hud);
void draw_tpms(HudCanvas &canvas, const HudState &hud);
void draw_calibration(HudCanvas &canvas, const HudState &hud);
void draw_learned(HudCanvas &canvas, const HudState &hud);
void draw_system(HudCanvas &canvas, const HudState &hud);
void draw_debug_card(HudCanvas &canvas, int y, const HudState &hud);

}  // namespace hud_draw
