#include "hud/hud_renderer.h"

#include "car/can_frame.h"
#include "hud/hud_draw.h"

#include <algorithm>
#include <cmath>
#include <string>

/* MaixCAM2 HUD(640x480). 주행 화면에는 필요한 것만 둔다: 상태 테두리, 현재 속도와 그 양옆
 * 깜빡이·비상등(노랑), 설정 속도(비전 크루즈가 낮게 잡으면 SET도)와 기어, 조향 모드와 하고 있는
 * 조작(회전, 차선 변경), 경로·차선, 앞차, 토크 바(운전자 토크 눈금 포함), 알림, 오토 홀드(속도
 * 아래 큰 배지). 아래 두 모서리에는 TPMS와 카메라 보정을, 그 위에는 보드 상태와 주행에 쓰는
 * 학습값을 좌우 짝으로 늘 두고, 오른쪽 위 상태 알약은 녹화와 와이파이를 보이며 누르면 네트워크
 * 카드를 연다. panda·저장 공간은 문제가 있을 때 칩으로도 띄우고(온도는 보드 상태 카드가 색으로),
 * 다른 카드에 없는 수치 진단은 웹 기기 설정의 HUD 진단을 켜면 왼쪽 카드에 모은다. 색·간격·글꼴
 * 크기는 hud_draw.h의 토큰 한 곳에서 정한다. 도로 장면은 hud_scene.cc, 카드는
 * hud_cards.cc가 그린다. 크기는 2.4" 패널(333 ppi)을 차 안에서 읽을 만큼(2026-10-04
 * 실차에서 한 번 키움). */

namespace hud_draw {
namespace {

constexpr int kTurnLitSteps = 15;
constexpr int kTurnChevronStartStep[] = {0, 4, 8};
constexpr float kTurnChevronAlpha[] = {0.35f, 0.65f, 1.0f};

// ---- 상태 → 문구 ----

std::string mode_text(const HudState &hud)
{
    if (hud.controller_active)
        return hud.laneless_mode ? "LANELESS" : "LANE";
    if (!hud.controller_engaged) return hud.controller_enabled ? "READY" : "OFF";
    if (hud.active_block[0] == '\0') return "READY";
    if (const char *label = engage_block_label(hud.active_block)) return label;
    std::string fallback = hud.active_block;
    std::replace(fallback.begin(), fallback.end(), '_', ' ');
    return fallback;
}

const char *gear_text(int gear)
{
    switch (gear) {
    case kGearPark: return "P";
    case kGearDrive: return "D";
    case kGearNeutral: return "N";
    case kGearReverse: return "R";
    case kGearSport: return "S";
    default: return "-";
    }
}

// 기어 카드의 색: 후진은 주황, 주차·중립은 흐리게.
uint32_t gear_color(int gear)
{
    if (gear == kGearReverse) return kAmber;
    return gear == kGearDrive || gear == kGearSport ? kText : kTextSecondary;
}

bool speed_valid(float kph) { return std::isfinite(kph) && kph > 0.0f; }

// ---- 알림 ----

uint32_t alert_color(HudAlertLevel level)
{
    switch (level) {
    case HudAlertLevel::proceed: return kGreen;
    case HudAlertLevel::caution: return kAmber;
    case HudAlertLevel::critical: return kRed;
    case HudAlertLevel::notice: break;
    }
    return kText;
}

// ---- 공통 부품 ----

// 칩 앞 표시: 색 점 또는 방향 화살표.
enum class ChipMark { dot, left, right };

// 방향 화살표: 가운데 (cx, cy), 폭 w, 높이 h인 삼각형. direction은 -1(왼쪽) 또는 1.
void draw_arrow(HudCanvas &canvas, float cx, float cy, float w, float h, int direction, uint32_t color)
{
    const float tip = cx + direction * w / 2.0f, back = cx - direction * w / 2.0f;
    const HudPoint points[] = {{tip, cy}, {back, cy - h / 2.0f}, {back, cy + h / 2.0f}};
    canvas.fill_polygon(points, 3, color);
}

/* 신호 대기: 한국식 가로 신호등(빨강·노랑·초록 중 빨강만 켜짐), 글 없이 아이콘만. 오른쪽 끝
 * right, 위 y, 높이 h. */
void draw_traffic_light(HudCanvas &canvas, int right, int y, int h)
{
    const int lamp = h - 4, w = 3 * lamp + 8, x = right - w;
    canvas.fill_round_rect(x, y, w, h, h / 2.0f, kSignalHousing);
    const uint32_t lamps[] = {kRed, hud_fade(kAmber, 0.25f), hud_fade(kGreen, 0.25f)};
    for (int i = 0; i < 3; ++i)
        canvas.fill_round_rect(x + 2 + i * (lamp + 2), y + 2, lamp, lamp, lamp / 2.0f, lamps[i]);
}

// 둥근 칩: 앞 표시와 글. x는 align 기준점. 그린 폭을 돌려준다.
int chip(HudCanvas &canvas, int x, int y, const std::string &text, uint32_t color,
         HudAlign align = HudAlign::left, ChipMark mark = ChipMark::dot)
{
    constexpr int kArrowW = 10;
    const int mark_w = mark == ChipMark::dot ? kDot : kArrowW;
    const int w = 2 * kChipPadX + mark_w + 6 + kHudBodyFont.width(text);
    const int left = align == HudAlign::right ? x - w : align == HudAlign::center ? x - w / 2 : x;
    canvas.fill_round_rect(left, y, w, kChipH, kChipH / 2.0f, kCard);
    const int mark_x = left + kChipPadX;
    if (mark == ChipMark::dot)
        canvas.fill_round_rect(mark_x, y + (kChipH - kDot) / 2, kDot, kDot, kDot / 2.0f, color);
    else
        draw_arrow(canvas, mark_x + kArrowW / 2.0f, y + kChipH / 2.0f, kArrowW, 13.0f,
                   mark == ChipMark::left ? -1 : 1, color);
    canvas.text(mark_x + mark_w + 6, centered_line_top(kHudBodyFont, y, kChipH), text, kHudBodyFont, kText);
    return w;
}

// ---- 위쪽과 아래 가장자리 ----

void draw_border(HudCanvas &canvas, uint32_t color)
{
    if (!color) return;
    const int w = canvas.width(), h = canvas.height();
    canvas.fill_rect(0, 0, w, kBorder, color);
    canvas.fill_rect(0, h - kBorder, w, kBorder, color);
    canvas.fill_rect(0, kBorder, kBorder, h - 2 * kBorder, color);
    canvas.fill_rect(w - kBorder, kBorder, kBorder, h - 2 * kBorder, color);
}

// 현재 속도 아래 "km/h" 줄의 위 y.
int speed_unit_line() { return kMargin + kHudSpeedFont.glyph('0')->h + 4; }

// 가운데 현재 속도와 그 양옆 깜빡이 화살표. 화살표 기준으로 쓸 숫자 폭을 돌려준다.
int draw_speed(HudCanvas &canvas, const HudState &hud)
{
    const std::string speed = format_text("%.0f", std::max(0.0f, hud.cluster_speed_kph));
    const int center = canvas.width() / 2;
    const int top = kMargin - kHudSpeedFont.glyph('0')->top;
    const int width = canvas.text(center, top, speed, kHudSpeedFont, kText, HudAlign::center, true);
    canvas.text(center, speed_unit_line(), "km/h", kHudCaptionFont, kTextSecondary, HudAlign::center, true);
    return width;
}

/* 오토 홀드: 현재 속도 바로 아래 가운데의 큰 배지. 차 계기판처럼 초록 글씨로, 정차해 브레이크를
 * 잡고 있는 동안 멀리서도 보이게. */
void draw_brake_hold(HudCanvas &canvas, const HudState &hud)
{
    if (!hud.brake_hold) return;
    constexpr int kH = 40, kPadX = 20;
    const char *text = "AUTO HOLD";
    const int w = 2 * kPadX + kHudTitleFont.width(text);
    const int y = speed_unit_line() + kHudCaptionFont.ascent + kGap;
    canvas.fill_round_rect((canvas.width() - w) / 2, y, w, kH, kH / 2.0f, kCardStrong);
    canvas.text(canvas.width() / 2, centered_line_top(kHudTitleFont, y, kH), text, kHudTitleFont, kGreen,
                HudAlign::center);
}

/* 현재 속도 양옆의 깜빡이 화살표(노랑, 비상등이면 양쪽). 세 개가 차례로 켜진다. 숫자에서 18 px
 * 띄우되, 세 자리 속도에서도 왼쪽 위 카드에 닿지 않게 안쪽으로 당긴다(오른쪽은 대칭). */
void draw_turn_signals(HudCanvas &canvas, const HudState &hud, int speed_width)
{
    if (!hud.left_blinker && !hud.right_blinker) return;
    const int step = std::clamp(hud.turn_signal_step, 0, kTurnSignalSteps - 1);
    if (step >= kTurnLitSteps) return;
    const float center_y = kMargin + kHudSpeedFont.glyph('0')->h / 2.0f;
    constexpr float kHalfH = 18.0f, kWidth = 18.0f, kThickness = 9.0f, kStep = 17.0f;
    constexpr float kReach = 2.0f * kStep + kThickness + kWidth;  // 안쪽 끝에서 바깥 화살표 끝까지
    constexpr float kCardsRight = kMargin + kSetCardW + kGap + kGearCardW + kGap;
    const float offset = std::min(speed_width / 2.0f + 18.0f, canvas.width() / 2.0f - kCardsRight - kReach);

    auto side = [&](bool active, float direction) {
        if (!active) return;
        const float inner = canvas.width() / 2.0f + direction * offset;
        for (int i = 0; i < 3 && step >= kTurnChevronStartStep[i]; ++i) {
            const float x = inner + direction * i * kStep;
            const HudPoint points[] = {{x, center_y - kHalfH},
                                       {x + direction * kThickness, center_y - kHalfH},
                                       {x + direction * (kThickness + kWidth), center_y},
                                       {x + direction * kThickness, center_y + kHalfH},
                                       {x, center_y + kHalfH},
                                       {x + direction * kWidth, center_y}};
            canvas.fill_polygon(points, 6, hud_fade(kYellow, kTurnChevronAlpha[i]));
        }
    };
    side(hud.left_blinker, -1.0f);
    side(hud.right_blinker, 1.0f);
}

/* 왼쪽 위 값 카드 하나: 위에 작은 이름, 그 아래 큰 값. sub가 있으면 큰 값을 이름 바로 아래로
 * 올리고 카드 아래쪽에 sub를 작게 쓴다. */
void top_card(HudCanvas &canvas, int x, int w, const char *title, const std::string &value, uint32_t color,
              const std::string &sub = {}, uint32_t sub_color = kTextSecondary)
{
    const int y = kMargin, header_bottom = y + kCardPad + cap_glyph(kHudCaptionFont).h;
    canvas.fill_round_rect(x, y, w, kTopCardH, kRadius, kCard);
    canvas.text(x + w / 2, cap_line(kHudCaptionFont, y + kCardPad), title, kHudCaptionFont, kTextSecondary,
                HudAlign::center);
    if (sub.empty()) {
        canvas.text(x + w / 2, centered_line_top(kHudValueFont, header_bottom, y + kTopCardH - 4 - header_bottom),
                    value, kHudValueFont, color, HudAlign::center);
        return;
    }
    canvas.text(x + w / 2, cap_line(kHudValueFont, header_bottom + 7), value, kHudValueFont, color, HudAlign::center);
    canvas.text(x + w / 2, base_line(kHudBodyFont, y + kTopCardH - kCardPad), sub, kHudBodyFont, sub_color,
                HudAlign::center);
}

/* 왼쪽 위: 설정 속도 카드와 기어 카드, 그 아래 조향 모드 칩. 비전 크루즈가 설정보다 낮게 잡고
 * 있으면 그 속도를 설정 속도 카드 아래쪽에 SET으로. 다음 칩이 올 y를 돌려준다. */
int draw_cruise(HudCanvas &canvas, const HudState &hud)
{
    const int x = kMargin, y = kMargin;
    const bool max_valid = speed_valid(hud.cruise_max_speed_kph);
    const bool limited = max_valid && speed_valid(hud.cruise_command_speed_kph) &&
                         hud.cruise_command_speed_kph < hud.cruise_max_speed_kph - 2.0f;
    top_card(canvas, x, kSetCardW, "MAX", max_valid ? format_text("%.0f", hud.cruise_max_speed_kph) : "-",
             max_valid && hud.cruise_active ? kText : kTextSecondary,
             limited ? format_text("SET %.0f", hud.cruise_command_speed_kph) : std::string(), kAmber);
    // 차 상태가 없으면 기어도 모른다(gear 0은 P와 같다)
    top_card(canvas, x + kSetCardW + kGap, kGearCardW, "GEAR", hud.vehicle_fresh ? gear_text(hud.gear) : "-",
             hud.vehicle_fresh ? gear_color(hud.gear) : kTextSecondary);

    const int chip_y = y + kTopCardH + kGap;
    chip(canvas, x, chip_y, mode_text(hud), state_color(hud) ? state_color(hud) : kGray);
    return chip_y + kChipH + kGap;
}

constexpr float kWifiRadius = 20.5f;  // 와이파이 부채의 바깥 반지름(아이콘 폭 = 2·r·sin45°)

/* 와이파이 부채: 아래 꼭짓점의 점과 그 위 호 셋(45°~135°). 세기만큼 점부터 켜고 나머지 호는
 * 흐리게, 가장 약하면 점만 주황. 무선이 아니면(dbm 0) 다 켠다. (cx, apex)는 꼭짓점. */
void draw_wifi_icon(HudCanvas &canvas, float cx, float apex, int dbm)
{
    constexpr int kSegments = 8;
    constexpr float kBands[3][2] = {{7.0f, 9.5f}, {12.5f, 15.0f}, {18.0f, kWifiRadius}};
    const int level = dbm == 0 || dbm >= -55 ? 4 : dbm >= -65 ? 3 : dbm > kWeakWifiDbm ? 2 : 1;
    const uint32_t lit = level <= 1 ? kAmber : kText;
    canvas.fill_round_rect(cx - 2.5f, apex - 5.0f, 5.0f, 5.0f, 2.5f, lit);
    for (int band = 0; band < 3; ++band) {
        HudPoint points[2 * (kSegments + 1)];
        for (int i = 0; i <= kSegments; ++i) {
            const float angle = (0.25f + 0.5f * static_cast<float>(i) / kSegments) * 3.14159265f;
            const float c = std::cos(angle), s = std::sin(angle);
            points[i] = {cx + kBands[band][1] * c, apex - kBands[band][1] * s};
            points[2 * kSegments + 1 - i] = {cx + kBands[band][0] * c, apex - kBands[band][0] * s};
        }
        canvas.fill_polygon(points, 2 * (kSegments + 1), band + 1 < level ? lit : kTrack);
    }
}

/* 오른쪽 위 상태 알약: 녹화 중이면 빨간 점과 REC, 그리고 와이파이 부채(끊기면 OFFLINE).
 * 누르면 네트워크 카드가 열린다(hud_status_touch). 다음 줄 y를 돌려준다. */
int draw_status_pill(HudCanvas &canvas, const HudState &hud)
{
    constexpr int kWifiW = 30, kItemGap = 12;  // kWifiW ≈ 2·kWifiRadius·sin45°
    const int y = kMargin;
    const int rec_w = hud.recording ? kDot + 6 + kHudBodyFont.width("REC") + kItemGap : 0;
    const int network_w = hud.network_connected ? kWifiW : kHudBodyFont.width("OFFLINE");
    const int w = 2 * kChipPadX + rec_w + network_w;
    int x = canvas.width() - kMargin - w;
    canvas.fill_round_rect(x, y, w, kChipH, kChipH / 2.0f, kCard);
    x += kChipPadX;
    const int line = centered_line_top(kHudBodyFont, y, kChipH);
    if (hud.recording) {
        canvas.fill_round_rect(x, y + (kChipH - kDot) / 2, kDot, kDot, kDot / 2.0f, kRed);
        canvas.text(x + kDot + 6, line, "REC", kHudBodyFont, kText);
        x += rec_w;
    }
    // 부채 높이(바깥 반지름)를 알약 높이 가운데에
    if (hud.network_connected)
        draw_wifi_icon(canvas, x + kWifiW / 2.0f, y + (kChipH + kWifiRadius) / 2.0f, hud.wifi_signal_dbm);
    else canvas.text(x, line, "OFFLINE", kHudBodyFont, kAmber);
    return y + kChipH + kGap;
}

/* 오른쪽 위: 상태 알약, 열려 있으면 네트워크 카드, 그 아래 문제 칩(panda, 저장 공간, 레이더 앞차)과
 * 신호 대기 신호등. CPU 온도는 보드 상태 카드가 색으로 알린다. */
void draw_status(HudCanvas &canvas, const HudState &hud, const LeadInfo &lead)
{
    const int right = canvas.width() - kMargin;
    int y = draw_status_pill(canvas, hud);
    if (hud.network_card) y = draw_network_card(canvas, y, hud);
    auto warn = [&](const std::string &text, uint32_t color) {
        chip(canvas, right, y, text, color, HudAlign::right);
        y += kChipH + kGap;
    };
    if (!hud.panda_connected || !hud.panda_healthy) warn("PANDA", kAmber);
    if (hud.storage_full) warn("STORAGE FULL", kAmber);
    if (lead.radar && !lead.vision) warn(format_text("LEAD %s", lead_text(lead).c_str()), kText);
    if (hud.green_light_alert_armed) draw_traffic_light(canvas, right, y, kChipH);
}

/* 아래 가장자리 토크 바: 보낸 토크를 가운데에서 회전 쪽으로 채우고, 운전자 토크를 같은 눈금에
 * 하늘색 세로 눈금으로 얹는다. 시스템과 운전자가 서로 미는지 바로 보인다. + = 왼쪽. */
void draw_torque_bar(HudCanvas &canvas, const HudState &hud)
{
    if (!hud.controller_engaged) return;
    constexpr float kDriverTickMin = 0.05f;  // 이보다 작은 운전자 토크(손을 얹은 정도)는 그리지 않는다
    const float cx = canvas.width() / 2.0f, half = kTorqueBarW / 2.0f;
    const float y = static_cast<float>(canvas.height() - kMargin - kTorqueBarH);
    const bool steering = steering_now(hud);
    canvas.fill_round_rect(cx - half, y, kTorqueBarW, kTorqueBarH, kTorqueBarH / 2.0f,
                           steering ? kTrack : hud_fade(kTrack, 0.5f));
    const float fraction = steering ? std::clamp(hud.steer_torque_fraction, -1.0f, 1.0f) : 0.0f;
    const float length = std::fabs(fraction) * half;
    if (length >= 1.0f) {
        // openpilot mici 토크 바처럼 75%를 넘으면 흰색에서 주황으로. + = 왼쪽 조향은 왼쪽으로 찬다.
        const uint32_t color = mix(kText, kAmber, (std::fabs(fraction) - 0.75f) * 4.0f);
        canvas.fill_round_rect(fraction > 0.0f ? cx - length : cx, y, length, kTorqueBarH, kTorqueBarH / 2.0f, color);
    }
    const float driver = std::clamp(hud.driver_torque_fraction, -1.0f, 1.0f);
    if (std::fabs(driver) >= kDriverTickMin)
        canvas.fill_round_rect(cx - driver * half - 1.5f, y - 5.0f, 3.0f, kTorqueBarH + 10.0f, 1.5f, kDriverTorque);
}

// 아래 가운데 알림 카드(hud_select_alert). 아래 모서리 카드 사이에 들어간다.
void draw_alert(HudCanvas &canvas, const HudAlertCard &alert)
{
    if (alert.empty()) return;
    const int max_w = canvas.width() - 2 * (kMargin + kCornerCardW + kGap);
    const int text_w = std::max(kHudTitleFont.width(alert.title), kHudBodyFont.width(alert.detail));
    const int w = std::clamp(text_w + 56, kAlertMinW, max_w);
    const int x = (canvas.width() - w) / 2, center = canvas.width() / 2;
    const int y = canvas.height() - kMargin - kTorqueBarH - kGap - kAlertH;
    constexpr int kInset = 16;  // 제목 윗선·설명 기준선과 색 막대 끝
    canvas.fill_round_rect(x, y, w, kAlertH, kRadius + 4, kCardStrong);
    canvas.fill_round_rect(x + 12, y + kInset, 4, kAlertH - 2 * kInset, 2.0f, alert_color(alert.level));
    const int title_w = canvas.text(center, cap_line(kHudTitleFont, y + kInset), alert.title, kHudTitleFont, kText,
                                    HudAlign::center);
    if (alert.arrow)
        draw_arrow(canvas, center + alert.arrow * (title_w / 2.0f + 18.0f),
                   y + kInset + cap_glyph(kHudTitleFont).h / 2.0f, 14.0f, 16.0f, alert.arrow, kText);
    canvas.text(center, base_line(kHudBodyFont, y + kAlertH - kInset), alert.detail, kHudBodyFont, kTextSecondary,
                HudAlign::center);
}

// 왼쪽 열: 조향 중에 하고 있는 조작(차선 변경). 다음 칩이 올 y를 돌려준다.
int draw_maneuver(HudCanvas &canvas, int y, const HudState &hud)
{
    if (!steering_now(hud) || hud.lane_change != 2) return y;
    chip(canvas, kMargin, y, "CHANGING LANES", state_color(hud), HudAlign::left,
         hud.lane_change_direction < 0 ? ChipMark::left : ChipMark::right);
    return y + kChipH + kGap;
}

}  // namespace
}  // namespace hud_draw

void HudRenderer::draw(const HudTarget &target, const ParsedModelOutput &output,
                           const ProjectionState &projection, const HudState &hud)
{
    using namespace hud_draw;
    const uint32_t buffer_h = target.orientation.transpose ? target.width : target.height;
    BufferDamage *damage = nullptr;
    for (BufferDamage &known : damage_)
        if (known.map == target.map && known.width == target.width && known.rows.size() == buffer_h)
            damage = &known;
    const bool known = damage != nullptr;
    if (!known) {
        damage = &damage_[next_slot_++ % damage_.size()];
        *damage = BufferDamage{target.map, target.width, std::vector<uint16_t>(buffer_h, 0)};
    }
    HudCanvas canvas(target.map, static_cast<int>(target.width), static_cast<int>(target.height),
                         static_cast<int>(target.stride), coverage_, damage->rows, target.orientation);
    canvas.clear(known);
    last_damage_ = &damage->rows;
    last_tile_shift_ = canvas.tile_shift();
    const LeadInfo lead = lead_info(hud, output);
    draw_scene(canvas, output, projection, hud, lead);
    draw_lane_position(canvas, output, projection, hud.lane_center_offset_m,
                       steering_now(hud) ? state_color(hud) : kText);
    draw_border(canvas, state_color(hud));
    draw_turn_signals(canvas, hud, draw_speed(canvas, hud));
    draw_brake_hold(canvas, hud);
    const int left_y = draw_maneuver(canvas, draw_cruise(canvas, hud), hud);
    draw_system(canvas, hud);
    // 진단 카드는 보드 상태 카드 뒤에: 조작 칩까지 있어 길어지면 위에 겹쳐 보인다
    if (hud.debug_card) draw_debug_card(canvas, left_y, hud);
    // 학습 카드는 오른쪽 열 칩보다 먼저: 칩이 많아 겹치면 경고가 위에 보인다
    draw_learned(canvas, hud);
    draw_status(canvas, hud, lead);
    draw_tpms(canvas, hud);
    draw_calibration(canvas, hud);
    draw_torque_bar(canvas, hud);
    draw_alert(canvas, hud_select_alert(hud, output.valid));
}

bool hud_status_touch(int x, int y, int width)
{
    using namespace hud_draw;
    return x >= width - kStatusTouchW && y < kStatusTouchH;
}

bool hud_left_column_touch(int x, int y, int height)
{
    using namespace hud_draw;
    return x < kLeftColumnTouchW && y < height - kMargin - kCornerCardH;
}
