// 보드의 모델과 controlsd가 발행하는 것을 BEV(bev.js)가 그리는 형태로 옮긴다. 보드는 복사만 한다:
// /api/bev는 BEV가 읽는 필드가 ModelState와 ControlState의 어디에 있는지 알려 주고
// (src/common/ipc_messages.h; 둘이 맞는지는 check_web_console.py가 지킨다), /api/bev/stream은 두 페이로드를
// 발행된 그대로, 새 모델 프레임마다 한 프레임씩 보낸다. 프레임은 16바이트 머리와 그 뒤의 페이로드다. 머리는
// 매직 "BEV1"(u32), ModelState와 ControlState 바이트 수(각 u16), 보드의 CLOCK_BOOTTIME ns(u64)다.
// 페이로드는 보드에 그것이 없거나, 모델이면 새것이 없을 때 0바이트다: 모델이 멈춰도 프레임은 그대로 매초 온다.
//
// scene()은 bev.js가 그리는 것으로, 차량 좌표계다: 미터, x는 앞 범퍼에서 앞으로, y는 왼쪽. 모델의 점은
// 카메라 기준이고 y가 오른쪽이다. 카메라는 controlsd와 HUD가 잡는 대로 범퍼에서 RADAR_TO_CAMERA 뒤에 있다.
// 모델은 이미 차의 중심선에서 보므로(modeld가 camera_offset_m만큼 워프한다) y는 옮길 필요가 없다. 점의 높이는
// 따로 노면 하나로 모은다(scene().road, bev_road.js). 선과 경로의 점은 x, y만 갖고 그 노면 위에 그려진다.
//
// 두 자차 차선선은 controlsd의 플래너가 보는 대로 그린다: 같은 차선선 필터(lane_filter.js)에 캡처 시각과
// controlsd의 속도·측정 곡률로 거른다. 모델 자체의 차선선도 함께 남는다(scene().lanes raw).

import {t} from "./i18n.js";
import {clamp} from "./bev_math.js";
import {FLAT, roadProfile} from "./bev_road.js";
import {LaneLineFilter} from "./lane_filter.js";
import {api} from "./ui.js";

const MAGIC = 0x31564542;
const HEADER = 16;
const TRAJECTORY = 33;
// openpilot T_IDXS: 차가 각 계획 점에 있게 되는 시각, s
const T_IDXS = Array.from({length: TRAJECTORY}, (_, i) => 10 * (i / (TRAJECTORY - 1)) ** 2);
// openpilot X_IDXS(float32, model_output.h model_x_idx): 차선·도로 경계 점의 앞 거리, m. ModelState는 그 점의 y, z만 싣는다
const X_IDXS = Array.from({length: TRAJECTORY}, (_, i) => Math.fround(192 * (i / (TRAJECTORY - 1)) ** 2));
const RADAR_TO_CAMERA = 1.52;           // model_output.h kRadarToCameraDistanceM (check_web_console.py)
const LINE_X = X_IDXS.map((x) => x - RADAR_TO_CAMERA);   // 차선·도로 경계 점의 범퍼에서 앞 거리, m
// 노면 높이(bev_road.js)에서 경로의 무게: 확실한 선(확률·신뢰도 1)의 10분의 1이라 선이 다 흐릴 때만 노면을 정한다
const PATH_HEIGHT_WEIGHT = 0.1;
const LEAD_PROBABILITY = 0.5;           // model_output.h kLeadProbabilityThreshold (check_web_console.py)
export const LANE_SURE = 0.5;           // 차로 폭을 재려면 두 자차 차선선 모두 이만큼 확실해야 한다(bev.js도 쓴다)
const LANE_WIDTH_INDEX = 6;             // hud_scene.cc kLaneRulerIndex: 카메라에서 6.75 m
// hud_scene.cc draw_scene: 차선선과 경로는 계획이 닿는 데까지, 10~100 m
const DRAW = {min: 10, max: 100};
const PLAN_NOW_INDEX = 2;               // 계획의 현재 속도: 첫 점의 잡음을 지난 0.04 s 지점
export const GAS_PRESS_HORIZON = 1;     // controls_tick.cc가 출발 알림에 넣는 gas_press_probs[1], 2 s 후
const DRIVE_GEAR = 5;                   // can_frame.h kGearDrive: 출발 알림은 D에서만 본다
export const HISTORY_S = 10;            // 출발 알림 카드가 입력을 얼마나 남겨 두는지
export const DEPARTURE_ALERT = {none: 0, leadDeparted: 1, greenLight: 2};  // departure_alert.h DepartureAlertType
// 앞차 카드: 최근 LEAD_HISTORY_S 동안 앞차의 거리와 자차 대비 상대 속도를 모델, 광류(속도만), modeld의
// 앞차 추적(둘을 융합한 것)에서 가져와 모델 자체 거리의 추세와 견준다. 추세는 앞차 하나의 ±half s에 맞춘
// 직선이다(끊김 없이, jump m보다 큰 튐 없이, 양 끝에서 edge s 안까지 닿는 points 프레임 이상).
// docs/lead-flow.md가 이들을 검증하는 방식대로다. 추세는 half s 늦게 알게 된다.
export const LEAD_HISTORY_S = 20;
const TREND = {half: 2, jump: 4, points: 60, edge: 0.1};
const MODEL_STALE_S = 0.5, CONTROL_STALE_S = 1.0;
const RETRY_MS = [500, 5000];
// 스트림이 이만큼 조용하면 끊긴 것으로 보고 다시 연결한다. 서버는 모델이 멈춰도 1초마다 보내므로
// (bev_stream.BEV_IDLE_S) 조용하다는 것은 보드나 연결이 사라졌다는 뜻이다. 그대로 기다리면 화면이 '정상'으로 멈췄다.
const STREAM_SILENCE_MS = 3000;
// 이 페이지가 읽는 필드. 그중 하나라도 위치를 알려 주지 않는 서버는 페이지보다 오래됐거나 새것이다
const NEEDS = {
  model: ["size", "capture_timestamp_ns", "model_timestamp_ns", "valid", "plan", "lanes", "lane_probabilities", "road_edges", "road_edge_stds",
          "lead", "gas_press_probs", "lead_flow", "lead_track"],
  control: ["size", "timestamp_ns", "enabled", "engaged", "active", "steering_fault", "left_blinker", "right_blinker", "gear",
            "desired_curvature", "actual_curvature",
            "normalized_output", "departure_alert_type", "green_light_alert_armed", "hud_flags", "ego_speed_kph"],
  hud_flags: ["Laneless", "SoftDisabling", "SteerPaused", "BrakeLights"],
};


// 페이로드 하나를 읽는 도구(리틀 엔디안). seconds는 ns(u64) 시각을 초로.
function reader(bytes) {
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  return {view, f32: offset => view.getFloat32(offset, true), u32: offset => view.getUint32(offset, true),
          i8: offset => view.getInt8(offset), seconds: offset => Number(view.getBigUint64(offset, true)) * 1e-9};
}

// 모델 점 xs, ys의 [[x, y(, speed)], ...]. HUD가 그것들로 띠를 그리듯, 유한하고 x가 커지며 reach(모델 x)보다
// 멀지 않은 점만.
function linePoints(xs, ys, reach, speeds = null) {
  const points = [];
  let last = -Infinity;
  for (let i = 0; i < xs.length; i++) {
    const x = xs[i], y = ys[i];
    if (!Number.isFinite(x) || !Number.isFinite(y) || x > reach || x <= last) break;
    points.push(speeds ? [x - RADAR_TO_CAMERA, -y, speeds[i]] : [x - RADAR_TO_CAMERA, -y]);
    last = x;
  }
  return points;
}

// offset에 있는 IpcLanePoint(y, z) 배열을 X_IDXS 위의 x, y, z로.
function lanePointsAt(view, offset) {
  const column = (k) => Array.from({length: TRAJECTORY}, (_, i) => view.getFloat32(offset + 8 * i + 4 * k, true));
  return [X_IDXS, column(0), column(1)];
}

// offset에 있는 IpcPoint 배열의 x, y, z.
function pointsAt(view, offset) {
  const xs = [], ys = [], zs = [];
  for (let i = 0; i < TRAJECTORY; i++) {
    xs.push(view.getFloat32(offset + 12 * i, true));
    ys.push(view.getFloat32(offset + 12 * i + 4, true));
    zs.push(view.getFloat32(offset + 12 * i + 8, true));
  }
  return [xs, ys, zs];
}

// 모델의 노면(bev_road.js), 그려지는 데(reach, 모델 x)까지: 네 차선선은 확률, 두 도로 경계는 신뢰도(1 - std),
// 경로는 PATH_HEIGHT_WEIGHT를 무게로 한다. 경로는 유한하고 앞으로 가는 점까지만 본다(정차 중이면 범퍼를 덮지 않아
// 빠진다).
function roadOf(lanes, lines, edges, edgeLines, [px, , pz], reach) {
  let n = 0;
  while (n < TRAJECTORY && Number.isFinite(px[n]) && Number.isFinite(pz[n]) && (n === 0 || px[n] > px[n - 1])) n++;
  return roadProfile([
    ...lines.map(([, , zs], i) => ({xs: LINE_X, zs, weight: lanes[i].probability})),
    ...edgeLines.map(([, , zs], i) => ({xs: LINE_X, zs, weight: clamp(1 - edges[i].std, 0, 1)})),
    {xs: px.slice(0, n).map((x) => x - RADAR_TO_CAMERA), zs: pz.slice(0, n), weight: PATH_HEIGHT_WEIGHT},
  ], LINE_X.filter((x, i) => x > 0 && X_IDXS[i] <= reach));
}

// 각 점에서 계획의 속도(m/s)를 T_IDXS에 걸친 위치로 구한다. 모델 자체의 속도 출력(ModelState에 없음)과
// 비교하면 2 s에서 0.04 m/s, 4 s에서 0.07 m/s 어긋나고(중앙값, 2026-10-04 주행), 보드의 U16 모델도 이를
// 지킨다(4 s까지 fp32와 상관 0.997).
function planSpeeds(view, offset) {
  const at = (i, k) => view.getFloat32(offset + 12 * i + 4 * k, true);
  return T_IDXS.map((_, i) => {
    const a = Math.max(0, i - 1), b = Math.min(TRAJECTORY - 1, i + 1);
    return Math.hypot(at(b, 0) - at(a, 0), at(b, 1) - at(a, 1)) / (T_IDXS[b] - T_IDXS[a]);
  });
}

// ModelState 페이로드(bytes) 하나를 layout이 알려 준 위치에서 읽는다.
function readModel(at, bytes) {
  const {view, f32, u32, seconds} = reader(bytes);
  const planEnd = f32(at.plan + 12 * (TRAJECTORY - 1));
  const reach = Number.isFinite(planEnd) ? clamp(planEnd, DRAW.min, DRAW.max) : DRAW.max;
  const speeds = planSpeeds(view, at.plan);
  const plan = pointsAt(view, at.plan);
  const lines = [0, 1, 2, 3].map((i) => lanePointsAt(view, at.lanes + 8 * TRAJECTORY * i));
  const edgeLines = [0, 1].map((i) => lanePointsAt(view, at.road_edges + 8 * TRAJECTORY * i));
  const lanes = lines.map(([xs, ys], i) => ({probability: f32(at.lane_probabilities + 4 * i), points: linePoints(xs, ys, reach)}));
  const edges = edgeLines.map(([xs, ys], i) => ({std: f32(at.road_edge_stds + 4 * i), points: linePoints(xs, ys, reach)}));
  const lead = at.lead, flow = at.lead_flow, track = at.lead_track;
  return {
    stamp: view.getBigUint64(at.model_timestamp_ns, true),
    captured: seconds(at.capture_timestamp_ns),
    valid: u32(at.valid) !== 0,
    path: linePoints(plan[0], plan[1], reach, speeds),
    planNow: speeds[PLAN_NOW_INDEX],
    planEnd,
    gas: Array.from({length: 6}, (_, i) => f32(at.gas_press_probs + 4 * i)),
    lanes, reach,
    egoLines: [lines[1], lines[2]],     // 차선선 필터(BevStream)에 넣을 자차 차선선의 모델 점
    edges,
    road: roadOf(lanes, lines, edges, edgeLines, plan, reach),
    laneWidth: laneWidth(lanes, lines[1][1], lines[2][1]),
    lead: u32(lead)
      ? {probability: f32(lead + 4), x: f32(lead + 8), y: f32(lead + 12), speed: f32(lead + 16), accel: f32(lead + 20)} : null,
    // LeadFlowState: 약 0.5 s 동안 앞차의 확대율(1/s, +면 가까워진다)과 그것을 구한 모델 거리
    // (m, 카메라 기준)
    flow: u32(flow + 8) ? {rate: f32(flow + 20), x: f32(flow + 28)} : null,
    // LeadTrackState: 모델 거리와 광류를 융합한 값(카메라에서 m, m/s, -면 가까워진다)
    track: u32(track) ? {x: f32(track + 8), relative: f32(track + 12)} : null,
  };
}

// LANE_WIDTH_INDEX에서 두 자차 차선선 사이의 폭. 둘 다 확실하지 않으면 NaN.
function laneWidth(lanes, left, right) {
  return lanes[1].probability >= LANE_SURE && lanes[2].probability >= LANE_SURE
    ? right[LANE_WIDTH_INDEX] - left[LANE_WIDTH_INDEX] : NaN;
}

// ControlState 페이로드 하나.
function readControl(at, flags, bytes) {
  const {view, f32, u32} = reader(bytes);
  const hud = u32(at.hud_flags), flag = name => (hud >>> flags[name] & 1) === 1;
  return {
    stamp: view.getBigUint64(at.timestamp_ns, true),
    enabled: u32(at.enabled) !== 0, engaged: u32(at.engaged) !== 0, active: u32(at.active) !== 0,
    // HUD가 빨강으로 그리는 상태: 조향 결함, 3초 뒤 해제 예고
    alarm: u32(at.steering_fault) !== 0 || flag("SoftDisabling"),
    laneless: flag("Laneless"), paused: flag("SteerPaused"), braking: flag("BrakeLights"),
    output: f32(at.normalized_output), speedKph: f32(at.ego_speed_kph),
    desired: f32(at.desired_curvature), actual: f32(at.actual_curvature),
    alert: u32(at.departure_alert_type), armed: u32(at.green_light_alert_armed) !== 0,
    drive: view.getInt32(at.gear, true) === DRIVE_GEAR,
    left: u32(at.left_blinker) !== 0, right: u32(at.right_blinker) !== 0,
  };
}

// 스트림 조각 하나를 STREAM_SILENCE_MS까지만 기다린다. 넘으면 오류로 끝내고, loop가 응답을 닫고 다시 연결한다.
function silenceLimited(read) {
  let timer;
  const silence = new Promise((_, reject) => {
    timer = setTimeout(() => reject(new Error(t("bev.silent", {seconds: STREAM_SILENCE_MS / 1000}))), STREAM_SILENCE_MS);
  });
  return Promise.race([read, silence]).finally(() => clearTimeout(timer));
}

const ageOf = (now, stamp) => stamp ? Number(now - stamp) / 1e9 : Infinity;

// controlsd가 없거나 낡았을 때 scene()이 쓰는 제어 상태(readControl의 꼴)
const NO_CONTROL = {enabled: false, engaged: false, active: false, alarm: false, laneless: false, paused: false,
                    braking: false, output: 0, speedKph: NaN, desired: NaN, actual: NaN, alert: 0,
                    armed: false, drive: false};

// 보드 시각 now에서 최신 모델·제어 상태(둘 다 null일 수 있다)의 장면:
//   path [[x, y, 계획 속도 m/s]], planNow (m/s), lanes [{probability, points, raw}] ×4 (왼쪽 바깥, 왼쪽,
//   오른쪽, 오른쪽 바깥. controlsd가 도는 동안 자차 차선선의 points는 플래너가 거르는 대로 거른 것이고, raw는
//   모델 자체의 것으로 자차 차선선에만 있다), edges [{std, points}] ×2, laneWidth (m, 그려진 차선선 사이,
//   두 자차 차선선이 다 확실하지 않으면 NaN), road (노면 높이 bev_road.js RoadProfile, 모델이 없으면 평평)
//   lead {x: 앞차 뒷면, y, gap, speed, accel, relative (m/s, 자차 속도가 없으면 null), ttc (s, 가까워지지
//     않으면 Infinity), tracked (x, gap, relative, speed가 앞차 추적에서 온다), raw {x, relative} (모델 자체의
//     것), flow {relative: -x · rate (m/s, 광류에서)} 또는 null} 또는 null
//   leads [{t s, x, v (모델의 것), flow, fx, fv (앞차 추적의 것), trend, level, segment}, ...]: 앞차 카드의
//     최근 LEAD_HISTORY_S, 거리는 자차 범퍼 기준. BevStream이 채운다(BevStream.addLead)
//   state {enabled, engaged, active, alarm (조향 결함이나 해제 예고), steering, laneless, output}, speedKph
//   (controlsd가 없으면 NaN)
//   curvature {desired, actual} (1/m, controlsd처럼 +가 오른쪽), controlsd가 없으면 null
//   blinkers {left, right} (둘 다면 비상등), controlsd가 없으면 null. controlsd는 램프가 마지막으로 깜박인
//   뒤 0.5 s 동안 각각을 켜 두므로, 램프가 깜박이는 동안 켜진 채로 있다
//   brakeLights: 자차의 브레이크등(페달을 밟았거나 AUTO HOLD: vehicle_can brake_lights_on)
//   departure {drive (D 단), armed, alert (DEPARTURE_ALERT), gas [0, 2, … 10 s],
//   plan10 (m, 10 s의 모델 x), history [[t s, gas 2 s, plan10]]}: departure_alert.cc가 판단에 쓰는 것
//   modelAge, controlAge (s, 없으면 Infinity)
function scene(model, control, now, history = []) {
  const modelAge = model ? ageOf(now, model.stamp) : Infinity;
  const controlAge = control ? ageOf(now, control.stamp) : Infinity;
  const live = control && controlAge <= CONTROL_STALE_S ? control : null;
  const c = live || NO_CONTROL;
  const speed = c.speedKph / 3.6;
  const shown = model && model.valid ? model : null;
  const lead = shown && shown.lead;
  const seen = lead && lead.probability >= LEAD_PROBABILITY && Number.isFinite(lead.x) && Number.isFinite(lead.speed);
  const flow = seen && shown.flow && Number.isFinite(shown.flow.rate) && Number.isFinite(shown.flow.x) ? shown.flow : null;
  const track = seen && shown.track && Number.isFinite(shown.track.x) && Number.isFinite(shown.track.relative) ? shown.track : null;
  const raw = seen ? {x: lead.x - RADAR_TO_CAMERA, relative: Number.isFinite(speed) ? lead.speed - speed : null} : null;
  const x = track ? track.x - RADAR_TO_CAMERA : raw && raw.x, relative = track ? track.relative : raw && raw.relative;
  return {
    modelAge, controlAge, stale: modelAge > MODEL_STALE_S,
    path: shown ? shown.path : [], planNow: shown ? shown.planNow : NaN,
    lanes: shown ? shown.lanes : [], edges: shown ? shown.edges : [],
    laneWidth: shown ? shown.laneWidth : NaN, road: shown ? shown.road : FLAT,
    lead: seen ? {x, y: -lead.y, gap: Math.max(0, x), accel: lead.accel, relative,
                  speed: track ? (Number.isFinite(speed) ? speed + track.relative : lead.speed) : lead.speed,
                  ttc: relative !== null && relative < -0.1 ? Math.max(0, x) / -relative : Infinity,
                  tracked: !!track, raw, flow: flow ? {relative: -flow.x * flow.rate} : null} : null,
    state: {enabled: c.enabled, engaged: c.engaged, active: c.active, alarm: c.alarm, steering: c.active && !c.paused,
            laneless: c.laneless, output: c.output},
    speedKph: c.speedKph,
    curvature: Number.isFinite(c.desired) && Number.isFinite(c.actual) ? {desired: c.desired, actual: c.actual} : null,
    blinkers: live ? {left: live.left, right: live.right} : null,
    brakeLights: c.braking,
    departure: {drive: c.drive, armed: c.armed, alert: c.alert, gas: shown ? shown.gas : [],
                plan10: shown ? shown.planEnd : NaN, history},
  };
}

// rows[i] 앞차의 모델 거리에 ±TREND.half s 동안 맞춘 직선: [기울기 m/s, rows[i] 시각의 level m]. 그 앞차가
// 창을 다 덮지 않으면 NaN (scene().leads).
function trendAt(rows, i) {
  const {t: at, segment} = rows[i];   // t는 번역 함수라 가리지 않는다
  let a = i, b = i;
  while (a > 0 && rows[a - 1].segment === segment && rows[a - 1].t >= at - TREND.half) a--;
  while (b + 1 < rows.length && rows[b + 1].segment === segment && rows[b + 1].t <= at + TREND.half) b++;
  const n = b - a + 1, reach = TREND.half - TREND.edge;
  if (n < TREND.points || rows[a].t > at - reach || rows[b].t < at + reach) return [NaN, NaN];
  let st = 0, sx = 0, stt = 0, stx = 0;
  for (let j = a; j <= b; j++) {
    const dt = rows[j].t - at, x = rows[j].x;
    st += dt; sx += x; stt += dt * dt; stx += dt * x;
  }
  const d = n * stt - st * st;
  if (!(d > 0)) return [NaN, NaN];
  const slope = (n * stx - st * sx) / d;
  return [slope, (sx - slope * st) / n];
}

function concat(a, b) {
  const joined = new Uint8Array(a.length + b.length);
  joined.set(a);
  joined.set(b, a.length);
  return joined;
}

// BEV 탭이 보이는 동안의 스트림: 프레임마다 onScene(scene()), 알릴 내용이 바뀌면 onStatus({state, text}). state는
// good(모델과 controlsd가 다 온다), warn(모델만), bad(끊김, 모델 없음·멈춤), wait(연결 중)이다. 스스로 다시
// 연결하며, 실패할 때마다 더 오래 기다린다. 연결할 때마다 필드 위치(/api/bev)부터 다시 받는다: 서버가 다시 뜨며
// 배치가 바뀌었으면 옛 위치로 읽지 않는다.
export class BevStream {
  constructor({onScene, onStatus, hz = 20}) {
    Object.assign(this, {onScene, onStatus, hz});
    this.layout = null;
    this.model = this.control = null;
    this.run = 0;
    this.abort = null;
    this.arrivals = [];                 // 최근 1초 동안 모델 프레임이 온 시각, ms
    this.history = [];                  // 최근 HISTORY_S 동안 모델 프레임의 [t s, gas 2 s, plan10]
    this.leads = [];                    // 앞차 카드의 행(scene().leads), 모델 프레임마다 하나
    this.segment = 0;                   // 행이 어느 앞차의 것인지(추세 맞춤은 한 앞차 안에서만)
    this.laneFilters = [new LaneLineFilter(), new LaneLineFilter()];   // 자차 차선선, 플래너의 것처럼
    this.said = "";
  }

  start() {
    if (this.abort) return;
    this.loop(++this.run);
  }

  stop() {
    this.run++;
    if (this.abort) this.abort.abort();
    this.abort = null;
  }

  status(state, text) {
    if (state + text === this.said) return;
    this.said = state + text;
    this.onStatus({state, text});
  }

  async loop(run) {
    let wait = RETRY_MS[0];
    while (run === this.run) {
      const abort = this.abort = new AbortController();
      try {
        this.status("wait", t("bev.connecting"));
        const layout = await api.get("/api/bev", {signal: abort.signal});
        const missing = Object.entries(NEEDS).flatMap(([part, keys]) => keys.filter((key) => !(key in (layout[part] || {}))));
        if (missing.length) throw new Error(t("bev.version_mismatch", {fields: missing.join(", ")}));
        this.layout = layout;
        const response = await fetch(`/api/bev/stream?hz=${this.hz}`, {cache: "no-store", signal: abort.signal});
        if (!response.ok || !response.body) throw new Error(`HTTP ${response.status}`);
        await this.read(response.body.getReader(), run);
        wait = RETRY_MS[0];
        if (run === this.run) this.status("wait", t("bev.reconnecting"));
      } catch (error) {
        if (run !== this.run) return;
        this.status("bad", t("bev.disconnected", {error: error.message || error}));
      } finally {
        // 읽다 실패해도(잘못된 프레임, 그리기 오류) 그 응답을 닫는다: 열린 채 두면 다시 연결할 때마다 연결이 하나씩
        // 쌓여 브라우저의 호스트당 연결(6개)을 다 쓰고 콘솔의 다른 요청까지 멈춘다
        abort.abort();
      }
      if (run !== this.run) return;
      await new Promise((resolve) => setTimeout(resolve, wait));
      wait = Math.min(2 * wait, RETRY_MS[1]);
    }
  }

  async read(reader, run) {
    let pending = new Uint8Array(0);
    for (;;) {
      const {value, done} = await silenceLimited(reader.read());
      if (done || run !== this.run) return;
      pending = pending.length ? concat(pending, value) : value;
      let at = 0;
      while (pending.length - at >= HEADER) {
        const head = new DataView(pending.buffer, pending.byteOffset + at, HEADER);
        if (head.getUint32(0, true) !== MAGIC) throw new Error(t("bev.bad_frame"));
        const model = head.getUint16(4, true), control = head.getUint16(6, true), end = at + HEADER + model + control;
        if (end > pending.length) break;
        this.frame(pending.subarray(at + HEADER, at + HEADER + model), pending.subarray(at + HEADER + model, end),
                   head.getBigUint64(8, true));
        at = end;
      }
      pending = pending.subarray(at);
    }
  }

  // 모델 프레임 하나의 앞차를 앞차 카드(scene().leads)에 넣는다. 행의 trend와 level은 그보다 TREND.half s
  // 뒤의 행들이 들어오면 채워진다. 아직 기다리는 행은 언제나 가장 새로운 것들이다.
  addLead(stamp, lead) {
    const rows = this.leads, last = rows[rows.length - 1];
    if (last && stamp < last.t) rows.length = 0;                 // 재부팅
    const x = lead ? lead.raw.x : NaN;
    if (!lead || !last || !Number.isFinite(last.x) || Math.abs(x - last.x) > TREND.jump) this.segment++;
    rows.push({t: stamp, x, v: lead && lead.raw.relative !== null ? lead.raw.relative : NaN, flow: lead && lead.flow ? lead.flow.relative : NaN,
               fx: lead && lead.tracked ? lead.x : NaN, fv: lead && lead.tracked ? lead.relative : NaN,
               trend: undefined, level: undefined, segment: lead ? this.segment : -1});
    let i = rows.length - 1;
    while (i > 0 && rows[i - 1].trend === undefined) i--;
    for (; i < rows.length && rows[i].t <= stamp - TREND.half; i++) {
      [rows[i].trend, rows[i].level] = rows[i].segment < 0 ? [NaN, NaN] : trendAt(rows, i);
    }
    while (rows.length && rows[0].t < stamp - LEAD_HISTORY_S) rows.shift();
  }

  // 자차 차선선을 controlsd의 속도와 측정 곡률(플래너가 받는 그대로, ControlState ego_speed_kph와
  // actual_curvature)로 플래너의 차선선 필터에 거른다. 플래너처럼, 유효하지 않은 모델 프레임은 필터를 건드리지
  // 않는다. controlsd가 없으면 모델의 차선선이 그대로 남고 필터는 처음부터 다시 시작한다.
  filterLanes(model, now) {
    if (!model.valid) return;
    const control = this.control && ageOf(now, this.control.stamp) <= CONTROL_STALE_S ? this.control : null;
    const speed = control ? control.speedKph / 3.6 : NaN, curvature = control ? control.actual : NaN;
    if (!Number.isFinite(speed) || !Number.isFinite(curvature)) {
      for (const filter of this.laneFilters) filter.reset();
      return;
    }
    const filtered = model.egoLines.map(([xs, ys], side) =>
      this.laneFilters[side].update(model.captured, xs, ys, speed, curvature));
    model.egoLines.forEach(([xs], side) => {
      const lane = model.lanes[1 + side];
      model.lanes[1 + side] = {...lane, raw: lane.points, points: linePoints(xs, filtered[side], model.reach)};
    });
    model.laneWidth = laneWidth(model.lanes, filtered[0], filtered[1]);
  }

  frame(model, control, now) {
    const layout = this.layout, fresh = model.length >= layout.model.size;
    let stamp = 0;  // 모델 프레임 시각(s). t는 번역 함수라 쓰지 않는다
    this.control = control.length >= layout.control.size ? readControl(layout.control, layout.hud_flags, control) : null;
    if (fresh) {
      this.model = readModel(layout.model, model);
      this.filterLanes(this.model, now);
      this.arrivals.push(performance.now());
      stamp = Number(this.model.stamp) / 1e9;
      const history = this.history;
      if (history.length && stamp < history[history.length - 1][0]) history.length = 0;  // 재부팅
      history.push([stamp, this.model.gas[GAS_PRESS_HORIZON], this.model.planEnd]);
      while (history.length && history[0][0] < stamp - HISTORY_S) history.shift();
    }
    const shown = scene(this.model, this.control, now, this.history);
    if (fresh) this.addLead(stamp, shown.lead);
    shown.leads = this.leads;
    this.onScene(shown);
    const since = performance.now() - 1000;
    while (this.arrivals.length && this.arrivals[0] < since) this.arrivals.shift();
    // 서버는 controlsd가 멈춰도 마지막 ControlState를 계속 보낸다: 오래된 것은 멈춤으로 알린다
    const note = !this.control ? ` · ${t("bev.no_controlsd")}`
      : shown.controlAge > CONTROL_STALE_S ? ` · ${t("bev.controlsd_stalled")}` : "";
    if (!this.model) this.status("bad", t("bev.no_model") + note);
    else if (shown.stale) this.status("bad", t("bev.model_stalled", {seconds: Math.round(shown.modelAge)}) + note);
    else this.status(note ? "warn" : "good", t("bev.model_rate", {hz: this.arrivals.length}) + note);
  }
}
