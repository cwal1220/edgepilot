// HUD 캔버스와 렌더러: 스트레이트 알파 합성을 부동소수 기준식과, 다각형 커버리지 합을 기하
// 넓이와 대조한다. 먼 쪽 흐림, 글자 폭, 둥근 사각형의 곧은 행 지름길, 그린 칸만 지우기,
// 세로 패널 버퍼(transpose·뒤집기)가 가로 그림을 옮긴 것과 같은지, 상태 테두리와 알림 카드,
// 아래 모서리 카드, 상태 알약 터치 영역도 본다.
#include "controls/control_block.h"
#include "hud/hud_canvas.h"
#include "hud/hud_draw.h"
#include "hud/hud_renderer.h"
#include "hud/hud_state.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

namespace {

struct Surface {
    int width;
    int height;
    std::vector<uint32_t> pixels;
    std::vector<uint16_t> coverage;
    std::vector<uint32_t> damage;

    Surface(int w, int h) : width(w), height(h), pixels(static_cast<size_t>(w) * h, 0) {}
    HudCanvas canvas() { return HudCanvas(pixels.data(), width, height, width * 4, coverage, damage); }
    uint32_t at(int x, int y) const { return pixels[static_cast<size_t>(y) * width + x]; }
    double ink() const  // 알파 합 / 255 = 덮인 넓이(px²)
    {
        double sum = 0.0;
        for (uint32_t p : pixels) sum += (p >> 24) / 255.0;
        return sum;
    }
};

int channel(uint32_t color, int shift) { return static_cast<int>((color >> shift) & 0xff); }

TEST(HudCanvas, BlendMatchesStraightAlphaOver) {
    const uint32_t dsts[] = {0, hud_argb(255, 10, 200, 30), hud_argb(120, 250, 40, 90), hud_argb(30, 0, 0, 255)};
    const uint32_t srcs[] = {hud_argb(255, 255, 255, 255), hud_argb(150, 12, 14, 18), hud_argb(40, 255, 176, 32)};
    for (uint32_t dst : dsts) {
        for (uint32_t src : srcs) {
            Surface s(5, 1);  // 넷씩 묶어 칠하는 화소와 남는 화소 하나
            std::fill(s.pixels.begin(), s.pixels.end(), dst);
            s.canvas().fill_rect(0, 0, 5, 1, src);
            const double sa = (src >> 24) / 255.0, da = (dst >> 24) / 255.0;
            const double oa = sa + da * (1.0 - sa);
            // 합성 알파와 색이 부동소수 기준식과 1 안쪽으로 같다
            EXPECT_NEAR(s.pixels[0] >> 24, oa * 255.0, 1.0);
            for (int shift : {0, 8, 16}) {
                const double expected = (channel(src, shift) * sa + channel(dst, shift) * da * (1.0 - sa)) / oa;
                EXPECT_NEAR(channel(s.pixels[0], shift), expected, 1.0) << std::hex << dst << " " << src;
            }
            for (uint32_t p : s.pixels) EXPECT_EQ(p, s.pixels[4]) << "넷씩 묶어 칠해도 하나씩 합성한 것과 같다";
        }
    }
}

TEST(HudCanvas, PolygonCoverageMatchesArea) {
    Surface s(200, 200);
    HudCanvas canvas = s.canvas();
    const HudPoint triangle[] = {{20.3f, 30.7f}, {170.6f, 60.2f}, {60.1f, 180.9f}};
    canvas.fill_polygon(triangle, 3, hud_argb(255, 255, 255, 255));
    const double area = std::fabs((triangle[1].x - triangle[0].x) * (triangle[2].y - triangle[0].y) -
                                  (triangle[2].x - triangle[0].x) * (triangle[1].y - triangle[0].y)) /
                        2.0;
    EXPECT_NEAR(s.ink(), area, area * 0.003) << "커버리지 합이 넓이와 같다";
    EXPECT_EQ(s.at(80, 90) >> 24, 255) << "안쪽은 불투명";
    EXPECT_EQ(s.at(5, 5) >> 24, 0) << "바깥은 비어 있다";

    Surface thin(200, 200);  // 1.5 px 폭 사선(차선처럼 가장자리 띠만 있는 도형)
    const HudPoint line[] = {{50.0f, 10.0f}, {51.5f, 10.0f}, {151.5f, 190.0f}, {150.0f, 190.0f}};
    thin.canvas().fill_polygon(line, 4, hud_argb(255, 255, 255, 255));
    EXPECT_NEAR(thin.ink(), 1.5 * 180.0, 1.5 * 180.0 * 0.01);
}

TEST(HudCanvas, FadedPolygonFadesTowardFar) {
    Surface s(40, 120);
    const HudPoint rect[] = {{0, 10}, {40, 10}, {40, 110}, {0, 110}};
    s.canvas().fill_polygon(rect, 4, hud_argb(200, 48, 209, 88), 110.0f, 10.0f, 0.1f);
    // 가까운 행은 원래 알파, 먼 행은 그 0.1배
    EXPECT_NEAR(s.at(20, 109) >> 24, 200, 3);
    EXPECT_NEAR(s.at(20, 10) >> 24, 20, 3);
    EXPECT_GT(s.at(20, 60) >> 24, s.at(20, 30) >> 24);
}

TEST(HudCanvas, RoundRectStraightRowsMatchPolygon) {
    // 정수 좌표 둥근 사각형은 곧은 행을 fill_rect로 칠한다. 이음매 없이 넓이가 맞아야 한다.
    Surface s(200, 120);
    s.canvas().fill_round_rect(20, 10, 150, 90, 10, hud_argb(255, 255, 255, 255));
    const double area = 150.0 * 90.0 - (4.0 - 3.14159265) * 10.0 * 10.0;
    EXPECT_NEAR(s.ink(), area, area * 0.003);
    for (int y = 20; y < 90; ++y) {
        EXPECT_EQ(s.at(20, y) >> 24, 255) << y;  // 곧은 행 왼쪽 끝 화소
        EXPECT_EQ(s.at(19, y) >> 24, 0) << y;
    }
    EXPECT_EQ(s.at(95, 19) >> 24, 255) << "위 모서리 띠와 곧은 행 사이";
    EXPECT_EQ(s.at(95, 90) >> 24, 255) << "곧은 행과 아래 모서리 띠 사이";
}

TEST(HudCanvas, ClearOnlyDamagedTiles) {
    for (int width : {640, 480}) {  // 32 px 칸 20개, 세로 패널 버퍼 폭은 16 px 칸 30개
        Surface s(width, 60);
        HudCanvas canvas = s.canvas();
        EXPECT_EQ(canvas.tile_shift(), width == 640 ? 5 : 4);
        std::fill(s.pixels.begin(), s.pixels.end(), hud_argb(255, 9, 9, 9));
        canvas.clear(false);
        EXPECT_EQ(s.ink(), 0.0) << "처음에는 마지막 칸까지 전부 지운다";
        canvas.fill_rect(10, 5, 20, 10, hud_argb(255, 255, 0, 0));                      // 첫 칸
        canvas.text(width - 30, 30, "OK", kHudBodyFont, hud_argb(255, 255, 255, 255));  // 마지막 칸
        s.pixels[50 * width + 300] = hud_argb(255, 1, 2, 3);                            // 캔버스 밖에서 쓴 화소
        canvas.clear(true);
        // 그린 칸은 비고, 그리지 않은 칸(직접 쓴 화소)은 그대로다
        for (int y = 0; y < 60; ++y)
            for (int x = 0; x < width; ++x)
                if (!(y == 50 && x == 300)) ASSERT_EQ(s.at(x, y), 0u) << x << "," << y;
        EXPECT_EQ(s.at(300, 50), hud_argb(255, 1, 2, 3));
    }
}

TEST(HudCanvas, TextWidthAndInk) {
    Surface s(200, 40);
    const int width = s.canvas().text(10, 5, "LANE 64", kHudBodyFont, hud_argb(255, 255, 255, 255));
    EXPECT_EQ(width, kHudBodyFont.width("LANE 64"));
    EXPECT_GT(s.ink(), 40.0) << "글자가 그려진다";
    for (int x = 0; x < 9; ++x) EXPECT_EQ(s.at(x, 20) >> 24, 0) << "시작점 왼쪽은 비어 있다";
}

TEST(HudRenderer, StateBorderAndAlert) {
    Surface s(640, 480);
    const HudTarget target{s.pixels.data(), 640, 480, 640 * 4};
    HudRenderer renderer;
    HudState hud;
    hud.services_healthy = true;
    renderer.draw(target, ParsedModelOutput{}, make_projection_state(0, 0, 0), hud);
    EXPECT_EQ(s.at(1, 240) >> 24, 0) << "결합 전에는 테두리가 없다";

    hud.controller_enabled = hud.controller_engaged = hud.controller_active = true;
    renderer.draw(target, ParsedModelOutput{}, make_projection_state(0, 0, 0), hud);
    EXPECT_EQ(s.at(1, 240), hud_argb(255, 48, 209, 88)) << "차선 모드 조향 중은 초록 테두리";

    hud.steering_fault = true;
    renderer.draw(target, ParsedModelOutput{}, make_projection_state(0, 0, 0), hud);
    EXPECT_EQ(channel(s.at(1, 240), 16), 255) << "조향 결함은 빨강 테두리";
    EXPECT_GT(s.at(320, 425) >> 24, 150) << "아래 가운데 알림 카드";
}

// 알림 카드의 모든 문구가 카드 최대 폭(아래 모서리 카드 사이)에 들어간다. 넘치면 카드가 더 넓어지지 못해 양끝
// 글자가 카드 밖으로 나갔다(STEERING PAUSED의 설명).
TEST(HudRenderer, EveryAlertTextFitsTheCard) {
    using namespace hud_draw;
    const int max_text = 640 - 2 * (kMargin + kCornerCardW + kGap) - kAlertTextPad;
    std::vector<HudAlertCard> cards;
    const auto add = [&](const HudState &hud) {
        for (bool model_ok : {false, true}) cards.push_back(hud_select_alert(hud, model_ok));
    };
    for (int r = 1; r < static_cast<int>(BlockReason::Count); ++r) {  // 결합 거부와 해제 예고: 차단 사유마다
        const BlockReason reason = static_cast<BlockReason>(r);
        HudState reject;
        std::snprintf(reject.engage_reject_label, sizeof(reject.engage_reject_label), "%s", block_reason_label(reason));
        add(reject);
        HudState soft;
        soft.soft_disabling = true;
        std::snprintf(soft.active_block, sizeof(soft.active_block), "%s", block_reason_name(reason));
        add(soft);
    }
    HudState hud;
    hud.steering_fault = true;
    add(hud);
    hud = HudState{};
    hud.panda_faults = 1;
    add(hud);
    for (bool by_driver : {false, true}) {
        hud = HudState{};
        hud.controller_active = hud.steer_paused = true;
        hud.steer_paused_by_driver = by_driver;
        add(hud);
    }
    hud = HudState{};
    hud.steer_saturated = true;
    add(hud);
    hud = HudState{};
    add(hud);  // 서비스 대기
    for (int direction : {-1, 1}) {
        hud = HudState{};
        hud.services_healthy = hud.controller_active = true;
        hud.lane_change = HudLaneChange::Pending;
        hud.lane_change_direction = direction;
        add(hud);
    }
    for (DepartureAlertType type : {DepartureAlertType::LeadDeparted, DepartureAlertType::GreenLight}) {
        hud = HudState{};
        hud.services_healthy = true;
        hud.departure_alert_type = type;
        add(hud);
    }
    for (const HudAlertCard &card : cards) {
        ASSERT_FALSE(card.empty());
        const float arrows = card.arrow ? 2.0f * (kAlertArrowGap + kAlertArrowW / 2.0f) : 0.0f;
        EXPECT_LE(kHudTitleFont.width(card.title) + arrows, max_text) << card.title;
        EXPECT_LE(kHudBodyFont.width(card.detail), max_text) << card.title << ": " << card.detail;
    }
}

TEST(HudRenderer, ReusedBufferMatchesFreshBuffer) {
    // 같은 버퍼에 다른 장면을 이어 그려도(그린 칸만 지움) 새 버퍼에 그린 것과 같다
    HudState first;
    first.controller_enabled = first.controller_engaged = first.controller_active = true;
    first.services_healthy = true;
    first.debug_card = true;
    first.network_card = true;
    first.network_connected = true;
    first.brake_hold = true;
    first.cluster_speed_kph = 88.0f;
    HudState second;
    second.services_healthy = true;
    second.cluster_speed_kph = 5.0f;

    ParsedModelOutput road;  // 굽은 길: 경로와 차선 띠(다각형 행 칠하기)
    road.valid = true;
    for (int i = 0; i < kTrajectorySize; ++i) {
        const float x = model_x_idx(i), curve = 0.0006f * x * x;
        road.plan.points[i] = {x, curve, 0.0f};
        for (int lane = 0; lane < 4; ++lane) {
            road.lanes[lane].probability = 0.9f;
            road.lanes[lane].points[i] = {x, -5.4f + 3.6f * lane + curve, kModelHeight};
        }
    }

    Surface reused(640, 480), fresh(640, 480);
    const HudTarget reused_target{reused.pixels.data(), 640, 480, 640 * 4};
    const HudTarget fresh_target{fresh.pixels.data(), 640, 480, 640 * 4};
    HudRenderer renderer, other;
    renderer.draw(reused_target, road, make_projection_state(0, 0, 0), first);
    int road_ink = 0;
    for (int y = 300; y < 380; ++y)
        for (int x = 200; x < 440; ++x) road_ink += (reused.at(x, y) >> 24) != 0;
    ASSERT_GT(road_ink, 2000) << "첫 장면에 경로 띠가 그려진다";
    renderer.draw(reused_target, ParsedModelOutput{}, make_projection_state(0, 0, 0), second);
    other.draw(fresh_target, ParsedModelOutput{}, make_projection_state(0, 0, 0), second);
    EXPECT_TRUE(reused.pixels == fresh.pixels);
}

// 세로 패널 버퍼(480x640)에 화면 좌표로 그린 HUD가 가로 버퍼 그림을 transpose한 것과 같다.
// 글자·사각형은 화소까지 같고, 다각형은 부표본 행 방향이 바뀌어 가장자리만 조금 다르다.
TEST(HudRenderer, PortraitBufferMatchesTransposedLandscape) {
    ParsedModelOutput road;
    road.valid = true;
    for (int i = 0; i < kTrajectorySize; ++i) {
        const float x = model_x_idx(i), curve = 0.0006f * x * x;
        road.plan.points[i] = {x, curve, 0.0f};
    }
    HudState hud;
    hud.services_healthy = hud.network_connected = hud.recording = true;
    hud.controller_enabled = hud.controller_engaged = hud.controller_active = true;
    hud.cluster_speed_kph = 64.0f;
    hud.steer_saturated = true;  // 알림 카드
    hud.debug_card = true;
    const ProjectionState projection = make_projection_state(0, 0, 0);

    Surface landscape(640, 480), portrait(480, 640);
    HudRenderer a, b;
    a.draw({landscape.pixels.data(), 640, 480, 640 * 4}, road, projection, hud);
    b.draw({portrait.pixels.data(), 640, 480, 480 * 4, true}, road, projection, hud);
    int exact = 0, close = 0, worst = 0;
    double ink_l = 0.0, ink_p = 0.0;
    for (int y = 0; y < 480; ++y) {
        for (int x = 0; x < 640; ++x) {
            const uint32_t l = landscape.at(x, y), p = portrait.at(y, x);
            exact += l == p;
            const int d = std::abs(static_cast<int>(l >> 24) - static_cast<int>(p >> 24));
            close += d <= 64;
            worst = std::max(worst, d);
            ink_l += (l >> 24) / 255.0;
            ink_p += (p >> 24) / 255.0;
        }
    }
    EXPECT_GT(exact, 640 * 480 * 99 / 100) << "거의 모든 화소가 같다";
    EXPECT_EQ(close, 640 * 480) << "다른 화소도 가장자리 커버리지 차이뿐(worst " << worst << ")";
    EXPECT_NEAR(ink_p, ink_l, ink_l * 0.002) << "덮인 넓이가 같다";

    // 글자와 사각형은 화소까지 같다
    Surface text_l(200, 60), text_p(60, 200);
    text_l.canvas().text(10, 8, "PITCH -2.30", kHudBodyFont, hud_argb(255, 255, 255, 255), HudAlign::Left, true);
    text_l.canvas().fill_rect(5, 40, 120, 9, hud_argb(150, 12, 14, 18));
    HudCanvas tp(text_p.pixels.data(), 200, 60, 60 * 4, text_p.coverage, text_p.damage, true);
    tp.text(10, 8, "PITCH -2.30", kHudBodyFont, hud_argb(255, 255, 255, 255), HudAlign::Left, true);
    tp.fill_rect(5, 40, 120, 9, hud_argb(150, 12, 14, 18));
    for (int y = 0; y < 60; ++y)
        for (int x = 0; x < 200; ++x) ASSERT_EQ(text_p.at(y, x), text_l.at(x, y)) << x << "," << y;
}

TEST(HudRenderer, CornerCardsAndStatusTouch) {
    Surface s(640, 480);
    const HudTarget target{s.pixels.data(), 640, 480, 640 * 4};
    HudRenderer renderer;
    renderer.draw(target, ParsedModelOutput{}, make_projection_state(0, 0, 0), HudState{});
    // TPMS(왼쪽 아래), 카메라 보정(오른쪽 아래), 그 위 보드 상태·학습값 카드는 값이 없어도 늘 있다
    EXPECT_EQ(s.at(40, 462) >> 24, 150);
    EXPECT_EQ(s.at(600, 462) >> 24, 150);
    EXPECT_EQ(s.at(137, 300) >> 24, 150);
    EXPECT_EQ(s.at(503, 300) >> 24, 150);
    EXPECT_EQ(s.at(320, 300) >> 24, 0) << "가운데는 비어 있다";
    EXPECT_EQ(s.at(320, 120) >> 24, 0) << "오토 홀드가 아니면 속도 아래는 비어 있다";

    HudState hold;
    hold.brake_hold = true;
    renderer.draw(target, ParsedModelOutput{}, make_projection_state(0, 0, 0), hold);
    EXPECT_EQ(s.at(320 - 90, 120) >> 24, 205) << "오토 홀드 배지는 속도 아래 가운데";

    // 세 자리 속도의 왼쪽 깜빡이: 노란 화살표 셋이 다 켜져도 왼쪽 위 기어 카드(오른쪽 끝 168)에 닿지
    // 않는다. 비전 크루즈 SET은 칩이 아니라 설정 속도 카드 안에 있다.
    HudState signal;
    signal.cluster_speed_kph = 120.0f;
    signal.left_blinker = true;
    signal.turn_signal_step = 10;
    signal.cruise_max_speed_kph = 110.0f;
    signal.cruise_command_speed_kph = 90.0f;
    renderer.draw(target, ParsedModelOutput{}, make_projection_state(0, 0, 0), signal);
    EXPECT_EQ(s.at(180, 43), 0xffffcc00u) << "가장 바깥 화살표";
    EXPECT_EQ(s.at(172, 43) >> 24, 0) << "카드와 화살표 사이";
    EXPECT_EQ(s.at(20, 150) >> 24, 0) << "모드 칩 아래에 SET 칩이 없다";

    // 오른쪽 위 상태 알약 둘레만 네트워크 카드를 연다
    EXPECT_TRUE(hud_status_touch(620, 20, 640));
    EXPECT_TRUE(hud_status_touch(500, 60, 640));
    EXPECT_FALSE(hud_status_touch(320, 20, 640));
    EXPECT_FALSE(hud_status_touch(620, 200, 640));
    // 왼쪽 열은 진단 카드를 켜고 끈다. 속도 숫자와 아래 TPMS 카드는 빠진다
    EXPECT_TRUE(hud_left_column_touch(40, 40, 480));
    EXPECT_TRUE(hud_left_column_touch(200, 300, 480));
    EXPECT_FALSE(hud_left_column_touch(320, 40, 480));
    EXPECT_FALSE(hud_left_column_touch(60, 430, 480));
    for (int y = 0; y < 480; y += 10)  // 두 영역은 겹치지 않는다
        for (int x = 0; x < 640; x += 10) EXPECT_FALSE(hud_status_touch(x, y, 640) && hud_left_column_touch(x, y, 480));
}

}  // namespace
