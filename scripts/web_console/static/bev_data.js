// What the board's model and controlsd publish, as the BEV (bev.js) draws it. The board only copies:
// /api/bev says where the fields the BEV reads sit in ModelState and ControlState
// (src/common/ipc_messages.h; check_web_console.py holds the two together), and /api/bev/stream sends both
// payloads as they were published, a frame per new model frame. A frame is a 16-byte head, magic
// "BEV1" (u32), ModelState and ControlState byte counts (u16 each) and the board's CLOCK_BOOTTIME ns
// (u64), then the payloads. A payload is 0 bytes when the board has none or, for the model, nothing
// new: with the model stopped, a frame comes every second all the same.
//
// scene() is what bev.js draws, in vehicle coordinates: metres, x forward from the front bumper, y
// to the left. The model's points are from the camera, y to the right; the camera sits
// RADAR_TO_CAMERA behind the bumper, as controlsd and the HUD take it. The model already sees from
// the car's centre line (modeld warps by camera_offset_m), so y needs no shift.

const MAGIC = 0x31564542;
const HEADER = 16;
const TRAJECTORY = 33;
// openpilot T_IDXS: when the car is at each plan point, s
const T_IDXS = Array.from({length: TRAJECTORY}, (_, i) => 10 * (i / (TRAJECTORY - 1)) ** 2);
const RADAR_TO_CAMERA = 1.52;           // model_output.h kRadarToCameraDistanceM (check_web_console.py)
const LEAD_PROBABILITY = 0.5;           // model_output.h kLeadProbabilityThreshold (check_web_console.py)
const LANE_SURE = 0.5;                  // both ego lines at least this sure for the lane width
const LANE_WIDTH_INDEX = 6;             // hud_scene.cc kLaneRulerIndex: 6.75 m from the camera
// hud_scene.cc draw_scene: lines and the path reach as far as the plan does, 10 to 100 m
const DRAW = {min: 10, max: 100};
const PLAN_NOW_INDEX = 2;               // the plan's speed now: at 0.04 s, past the first point's noise
const GAS_PRESS_HORIZON = 1;            // controls_tick.cc feeds the departure alert gas_press_probs[1], 2 s ahead
const DRIVE_GEAR = 5;                   // can_frame.h kGearDrive: the departure alert only watches in D
const HISTORY_S = 10;                   // how much of the departure inputs the card keeps
const MODEL_STALE_S = 0.5, CONTROL_STALE_S = 1.0;
const RETRY_MS = [500, 5000];
// the fields this page reads; a server that does not say where one is is older or newer than the page
const NEEDS = {
  model: ["size", "model_timestamp_ns", "valid", "plan", "lanes", "lane_probabilities", "road_edges", "road_edge_stds",
          "lead", "gas_press_probs"],
  control: ["size", "timestamp_ns", "enabled", "engaged", "active", "left_blinker", "right_blinker", "gear",
            "desired_curvature", "actual_curvature",
            "normalized_output", "departure_alert_type", "green_light_alert_armed", "hud_flags", "ego_speed_kph"],
  hud_flags: ["Laneless", "SteerPaused", "BrakeLights"],
};

const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));

// [[x, y(, speed)], ...] of the IpcPoint array at offset, as the HUD draws a ribbon of it: finite
// points, x growing, no farther than reach (model x).
function polyline(view, offset, reach, speeds = null) {
  const points = [];
  let last = -Infinity;
  for (let i = 0; i < TRAJECTORY; i++) {
    const x = view.getFloat32(offset + 12 * i, true), y = view.getFloat32(offset + 12 * i + 4, true);
    if (!Number.isFinite(x) || !Number.isFinite(y) || x > reach || x <= last) break;
    points.push(speeds ? [x - RADAR_TO_CAMERA, -y, speeds[i]] : [x - RADAR_TO_CAMERA, -y]);
    last = x;
  }
  return points;
}

// The plan's speed at each point, m/s, from its positions over T_IDXS. Against the model's own
// velocity output (not in ModelState) it is off by 0.04 m/s at 2 s and 0.07 m/s at 4 s (median,
// 2026-10-04 drives), and the board's U16 model keeps it (correlation 0.997 with fp32 to 4 s).
function planSpeeds(view, offset) {
  const at = (i, k) => view.getFloat32(offset + 12 * i + 4 * k, true);
  return T_IDXS.map((_, i) => {
    const a = Math.max(0, i - 1), b = Math.min(TRAJECTORY - 1, i + 1);
    return Math.hypot(at(b, 0) - at(a, 0), at(b, 1) - at(a, 1)) / (T_IDXS[b] - T_IDXS[a]);
  });
}

// One ModelState payload (bytes), read where layout says.
function readModel(at, bytes) {
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const f32 = (offset) => view.getFloat32(offset, true);
  const planEnd = f32(at.plan + 12 * (TRAJECTORY - 1));
  const reach = Number.isFinite(planEnd) ? clamp(planEnd, DRAW.min, DRAW.max) : DRAW.max;
  const speeds = planSpeeds(view, at.plan);
  const lanes = [0, 1, 2, 3].map((i) => ({probability: f32(at.lane_probabilities + 4 * i),
                                          points: polyline(view, at.lanes + 12 * TRAJECTORY * i, reach)}));
  const left = at.lanes + 12 * (TRAJECTORY + LANE_WIDTH_INDEX) + 4, right = left + 12 * TRAJECTORY;
  const lead = at.lead;
  return {
    stamp: view.getBigUint64(at.model_timestamp_ns, true),
    valid: view.getUint32(at.valid, true) !== 0,
    path: polyline(view, at.plan, reach, speeds),
    planNow: speeds[PLAN_NOW_INDEX],
    planEnd,
    gas: Array.from({length: 6}, (_, i) => f32(at.gas_press_probs + 4 * i)),
    lanes,
    edges: [0, 1].map((i) => ({std: f32(at.road_edge_stds + 4 * i), points: polyline(view, at.road_edges + 12 * TRAJECTORY * i, reach)})),
    laneWidth: lanes[1].probability >= LANE_SURE && lanes[2].probability >= LANE_SURE ? f32(right) - f32(left) : NaN,
    lead: view.getUint32(lead, true)
      ? {probability: f32(lead + 4), x: f32(lead + 8), y: f32(lead + 12), speed: f32(lead + 16), accel: f32(lead + 20)} : null,
  };
}

// One ControlState payload.
function readControl(at, flags, bytes) {
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const u32 = (offset) => view.getUint32(offset, true), f32 = (offset) => view.getFloat32(offset, true);
  const hud = u32(at.hud_flags);
  return {
    stamp: view.getBigUint64(at.timestamp_ns, true),
    enabled: u32(at.enabled) !== 0, engaged: u32(at.engaged) !== 0, active: u32(at.active) !== 0,
    laneless: (hud >>> flags.Laneless & 1) === 1, paused: (hud >>> flags.SteerPaused & 1) === 1,
    braking: (hud >>> flags.BrakeLights & 1) === 1,
    output: f32(at.normalized_output), speedKph: f32(at.ego_speed_kph),
    desired: f32(at.desired_curvature), actual: f32(at.actual_curvature),
    alert: u32(at.departure_alert_type), armed: u32(at.green_light_alert_armed) !== 0,
    drive: view.getInt32(at.gear, true) === DRIVE_GEAR,
    left: u32(at.left_blinker) !== 0, right: u32(at.right_blinker) !== 0,
  };
}

const ageOf = (now, stamp) => stamp ? Number(now - stamp) / 1e9 : Infinity;

// The scene of the latest model and control state (either may be null) at board time now:
//   path [[x, y, planned speed m/s]], planNow (m/s), lanes [{probability, points}] ×4 (far left,
//   left, right, far right), edges [{std, points}] ×2, laneWidth (m, NaN unless both ego lines are
//   sure)
//   lead {x: its rear, y, gap, speed, accel, relative (m/s, null without our speed), probability} or null
//   state {enabled, engaged, active, steering, laneless, output}, speedKph (NaN without controlsd)
//   curvature {desired, actual} (1/m, + to the right as in controlsd) or null without controlsd
//   blinkers {left, right} (both: hazard lights) or null without controlsd. controlsd holds each on
//   for 0.5 s past the lamp's last flash, so they stay on while it flashes
//   brakeLights: the ego car's brake lights (pedal pressed, or AUTO HOLD: vehicle_can brake_lights_on)
//   departure {drive (in D), armed, alert (0 none, 1 lead moved, 2 green light), gas [0, 2, … 10 s],
//   plan10 (m, model x at 10 s), history [[t s, gas 2 s, plan10]]}: what departure_alert.cc decides from
//   modelAge, controlAge (s, Infinity if none)
export function scene(model, control, now, history = []) {
  const modelAge = model ? ageOf(now, model.stamp) : Infinity;
  const controlAge = control ? ageOf(now, control.stamp) : Infinity;
  const live = control && controlAge <= CONTROL_STALE_S ? control : null;
  const speed = live ? live.speedKph / 3.6 : NaN;
  const shown = model && model.valid ? model : null;
  const lead = shown && shown.lead;
  const seen = lead && lead.probability >= LEAD_PROBABILITY && Number.isFinite(lead.x) && Number.isFinite(lead.speed);
  return {
    modelAge, controlAge, stale: modelAge > MODEL_STALE_S,
    path: shown ? shown.path : [], planNow: shown ? shown.planNow : NaN,
    lanes: shown ? shown.lanes : [], edges: shown ? shown.edges : [],
    laneWidth: shown ? shown.laneWidth : NaN,
    lead: seen ? {x: lead.x - RADAR_TO_CAMERA, y: -lead.y, gap: Math.max(0, lead.x - RADAR_TO_CAMERA), speed: lead.speed,
                  accel: lead.accel, relative: Number.isFinite(speed) ? lead.speed - speed : null,
                  probability: lead.probability} : null,
    state: {enabled: !!live && live.enabled, engaged: !!live && live.engaged, active: !!live && live.active,
            steering: !!live && live.active && !live.paused, laneless: !!live && live.laneless, output: live ? live.output : 0},
    speedKph: live ? live.speedKph : NaN,
    curvature: live && Number.isFinite(live.desired) && Number.isFinite(live.actual) ? {desired: live.desired, actual: live.actual} : null,
    blinkers: live ? {left: live.left, right: live.right} : null,
    brakeLights: !!live && live.braking,
    departure: {drive: !!live && live.drive, armed: !!live && live.armed, alert: live ? live.alert : 0, gas: shown ? shown.gas : [],
                plan10: shown ? shown.planEnd : NaN, history},
  };
}

function concat(a, b) {
  const joined = new Uint8Array(a.length + b.length);
  joined.set(a);
  joined.set(b, a.length);
  return joined;
}

// The stream while the BEV tab is shown: onScene(scene()) for every frame, onStatus({live, text})
// when what it has to say changes. It connects again by itself, waiting longer each time it fails.
export class BevStream {
  constructor({onScene, onStatus, hz = 20}) {
    Object.assign(this, {onScene, onStatus, hz});
    this.layout = null;
    this.model = this.control = null;
    this.run = 0;
    this.abort = null;
    this.arrivals = [];                 // when the model frames of the last second came, ms
    this.history = [];                  // [t s, gas 2 s, plan10] of the last HISTORY_S of model frames
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

  status(live, text) {
    if (text === this.said) return;
    this.said = text;
    this.onStatus({live, text});
  }

  async loop(run) {
    let wait = RETRY_MS[0];
    while (run === this.run) {
      const abort = this.abort = new AbortController();
      try {
        this.status(false, "연결 중");
        if (!this.layout) {
          const response = await fetch("/api/bev", {cache: "no-store", signal: abort.signal});
          if (!response.ok) throw new Error(`HTTP ${response.status}`);
          const layout = await response.json();
          const missing = Object.entries(NEEDS).flatMap(([part, keys]) => keys.filter((key) => !(key in (layout[part] || {}))));
          if (missing.length) throw new Error(`서버와 페이지 버전이 다릅니다(${missing.join(", ")}): web_console/을 통째로 다시 올리세요`);
          this.layout = layout;
        }
        const response = await fetch(`/api/bev/stream?hz=${this.hz}`, {cache: "no-store", signal: abort.signal});
        if (!response.ok || !response.body) throw new Error(`HTTP ${response.status}`);
        await this.read(response.body.getReader(), run);
        wait = RETRY_MS[0];
        if (run === this.run) this.status(false, "연결 끊김 · 다시 연결 중");
      } catch (error) {
        if (run !== this.run) return;
        this.status(false, `연결 끊김 · ${error.message || error}`);
      }
      if (run !== this.run) return;
      await new Promise((resolve) => setTimeout(resolve, wait));
      wait = Math.min(2 * wait, RETRY_MS[1]);
    }
  }

  async read(reader, run) {
    let pending = new Uint8Array(0);
    for (;;) {
      const {value, done} = await reader.read();
      if (done || run !== this.run) return;
      pending = pending.length ? concat(pending, value) : value;
      let at = 0;
      while (pending.length - at >= HEADER) {
        const head = new DataView(pending.buffer, pending.byteOffset + at, HEADER);
        if (head.getUint32(0, true) !== MAGIC) throw new Error("잘못된 프레임");
        const model = head.getUint16(4, true), control = head.getUint16(6, true), end = at + HEADER + model + control;
        if (end > pending.length) break;
        this.frame(pending.subarray(at + HEADER, at + HEADER + model), pending.subarray(at + HEADER + model, end),
                   head.getBigUint64(8, true));
        at = end;
      }
      pending = pending.subarray(at);
    }
  }

  frame(model, control, now) {
    const layout = this.layout;
    if (model.length >= layout.model.size) {
      this.model = readModel(layout.model, model);
      this.arrivals.push(performance.now());
      const t = Number(this.model.stamp) / 1e9, history = this.history;
      if (history.length && t < history[history.length - 1][0]) history.length = 0;     // another boot
      history.push([t, this.model.gas[GAS_PRESS_HORIZON], this.model.planEnd]);
      while (history.length && history[0][0] < t - HISTORY_S) history.shift();
    }
    this.control = control.length >= layout.control.size ? readControl(layout.control, layout.hud_flags, control) : null;
    const shown = scene(this.model, this.control, now, this.history);
    this.onScene(shown);
    const since = performance.now() - 1000;
    while (this.arrivals.length && this.arrivals[0] < since) this.arrivals.shift();
    const note = this.control ? "" : " · controlsd 없음";
    if (!this.model) this.status(false, `모델 상태 없음${note}`);
    else if (shown.stale) this.status(false, `모델 멈춤 · ${Math.round(shown.modelAge)}초 전${note}`);
    else this.status(true, `모델 ${this.arrivals.length} Hz${note}`);
  }
}
