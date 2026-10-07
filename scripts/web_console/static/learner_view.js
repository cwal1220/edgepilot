// 실시간 학습 탭: paramsd·torqued 학습값(스위치로 제어에 사용, 학습값을 수동값으로 저장), 학습 입력 출처, 카메라
// 보정(초기화), lagd 조향 지연, 최근 10분 추이. 1초마다 /api/learners를 읽는다.
import {clampToMeta, metaFor, params, saveParam} from "./param_state.js";
import {
  actionButton, api, clock, confirmAction, deg, el, infoTable, liveCard, ms, note, num, poller, section, sgn, svg, toast,
} from "./ui.js";

const GRAVITY = 9.81;
const NOTE = "paramsd·torqued는 항상 계산하고 기록합니다. 제어에는 스위치를 켠 쪽만 씁니다. 1초마다 갱신하고 추이는 최근 10분입니다.";

/* 학습값을 수동값(params/steering.json)으로 옮기는 버튼들. learned는 파라미터 단위의 학습값이고, 보여 주는
 * 자릿수(digits) 그대로 저장한다. */
const ADOPT = {
  vehicle: {
    switchKey: "use_live_vehicle_params",
    ready: s => s.flags.vehicle_valid,
    items: [
      {key: "steer_ratio", label: "조향비", learned: s => s.steer_ratio, digits: 2},
      {key: "angle_offset_deg", label: "영점 평균", learned: s => s.angle_offset_average_deg, digits: 2, signed: true,
       unit: "°", ignoredWhenOn: true},
    ],
  },
  torque: {
    switchKey: "use_live_torque_params",
    ready: s => s.flags.torque_valid,
    items: [
      {key: "torque_lat_accel_factor", label: "배율", learned: s => s.lat_accel_factor, digits: 2, resets: true},
      {key: "torque_friction", label: "마찰", learned: s => s.friction, digits: 3, resets: true},
      {key: "torque_lat_accel_offset", label: "절편", learned: s => s.lat_accel_offset, digits: 3, signed: true,
       ignoredWhenOn: true},
    ],
  },
};
const learnedText = (item, s) => `${(item.signed ? sgn : num)(item.learned(s), item.digits)}${item.unit || ""}`;

const TREND_CHARTS = [
  {title: "조향비", series: [[1, "main"]], ref: m => [m.steer_ratio], span: 0.2, digits: 2},
  {title: "조향각 영점 (°) · 평균 / 합계", series: [[2, "main"], [3, "thin"]], ref: m => [m.angle_offset_deg], span: 0.2, digits: 2},
  {title: "도로 롤 (°) · 학습 / 편경사 환산", series: [[4, "main"], [5, "dash"]], ref: () => [], span: 0.5, digits: 2},
  {title: "토크 배율 · 필터 / 원시", series: [[7, "main"], [6, "raw"]],
   ref: (m, s) => [s.prior_lat_accel_factor, s.prior_lat_accel_factor * 0.7, s.prior_lat_accel_factor * 1.3], span: 0.2, digits: 2},
];
const CALIBRATION_STATUS = {
  calibrated: ["보정 완료", "good"], uncalibrated: ["수렴 중", "warn"], invalid: ["범위 밖", "bad"],
  recalibrating: ["장착 변경 · 재보정 중", "warn"],
};
const LAG_STATUS = {0: ["추정 전", "warn"], 1: ["추정됨", "good"], 2: ["무효", "bad"]};

const steering = () => params().params.steering;
const restored = flag => (flag ? "이번 시동에 복원" : "새로 시작");

/* 카드 머리의 학습 스위치(steering.json의 bool 항목). 바꾸면 곧바로 저장한다. sync()는 스냅샷 값으로 되돌린다. */
function learnerSwitch(key) {
  const meta = metaFor("steering", key);
  const text = el("span");
  const button = el("button.switch-control.mini", {type: "button", role: "switch", title: meta.label, "aria-label": meta.label});
  const show = value => {
    button.setAttribute("aria-checked", String(value));
    text.textContent = value ? "켜짐" : "꺼짐";
  };
  button.append(text, el("span.switch"));
  button.addEventListener("click", async () => {
    button.disabled = true;
    try {
      const {applied} = await saveParam("steering", key, button.getAttribute("aria-checked") !== "true");
      show(applied);
      toast(`${meta.label} ${applied ? "켜짐" : "꺼짐"} · ${clock()}`);
    } catch (error) {
      toast(`${meta.label}: ${error.message}`, {failed: true});
    } finally {
      button.disabled = false;
    }
  });
  button.sync = () => show(steering()[key]);
  button.sync();
  return button;
}

// ------------------------------------------------------------ 학습값 카드와 수동값 저장 버튼
function adoptTarget(item, s) {
  const meta = metaFor("steering", item.key);
  return {meta, current: steering()[item.key], target: clampToMeta(item.learned(s), meta, item.digits)};
}

function adoptCard(title, spec, render) {
  let state = null;
  const buttons = new Map(spec.items.map(item => [item.key, actionButton({icon: "⤓", onclick: () => adopt(item)})]));
  const toggle = learnerSwitch(spec.switchKey);
  const card = liveCard(title, {control: toggle, actions: [...buttons.values()]});

  const adopt = async item => {
    if (!state) return;
    const {meta, current, target} = adoptTarget(item, state);
    const on = steering()[spec.switchKey] === true;
    if (!confirmAction([`${meta.label} (${item.key})`, `${current} → ${target}`, "",
                        !on ? "스위치가 꺼져 있어 지금 바로 제어에 반영됩니다."
                          : item.ignoredWhenOn ? "스위치가 켜져 있어 지금은 무시되고, 스위치를 끄면 이 값을 씁니다."
                          : "스위치가 켜져 있어 사전값으로만 쓰입니다(controlsd 다음 시작부터).",
                        item.resets ? "controlsd 다음 시작 때 torqued 학습이 이 값을 새 사전값으로 처음부터 다시 시작합니다." : null]))
      return;
    try {
      await saveParam("steering", item.key, target);
      toast(`${meta.label} ${current} → ${target} 저장됨`);
    } catch (error) {
      toast(`${meta.label}: ${error.message}`, {failed: true});
    }
    updateButtons();
  };

  const updateButtons = () => {
    const ready = spec.ready(state);
    const on = steering()[spec.switchKey] === true;
    for (const item of spec.items) {
      const button = buttons.get(item.key);
      const {current, target} = adoptTarget(item, state);
      const same = Number(current) === Number(target);
      const effect = !on ? "바로 제어에 반영" : item.ignoredWhenOn ? "스위치 켜짐: 끌 때 쓰임"
        : item.resets ? "스위치 켜짐: 사전값 · torqued 재학습" : "스위치 켜짐: 출발점·범위로만 쓰임";
      button.setContent(`수동값에 저장 · ${item.label} ${learnedText(item, state)}`,
        !ready ? "학습이 유효해지면 쓸 수 있습니다" : same ? `수동값과 같음 (${current})` : `${item.key} ${current} → ${target}`,
        ready && !same ? effect : null);
      button.disabled = !ready || same;
    }
  };

  // 학습 상태가 없을 때(controlsd가 없거나 아직 발행 전)
  const clear = () => {
    state = null;
    card.update([["상태 없음", "muted"]], []);
    for (const item of spec.items) {
      const button = buttons.get(item.key);
      button.setContent(`수동값에 저장 · ${item.label}`, "학습 상태가 없습니다");
      button.disabled = true;
    }
  };
  clear();

  return {
    element: card.element,
    toggle,
    clear,
    update(s, manual) {
      state = s;
      const [badgeList, children] = render(s, manual);
      card.update(badgeList, children);
      updateButtons();
    },
  };
}

function vehicleSummary(s, m) {
  const f = s.flags;
  const bankRoll = deg(-s.road_bank_lat_accel / GRAVITY);
  return [[
    f.vehicle_valid ? ["유효", "good"] : ["무효", "bad"],
    f.vehicle_inputs_ok ? null : ["입력 끊김", "warn"],
    f.sensor_valid ? null : ["센서 불일치", "warn"],
    f.use_vehicle ? ["제어에 사용 중", "accent"] : ["섀도", "muted"],
    f.localizer_inputs ? ["입력 locationd", "accent"] : ["입력 ESP12", "muted"],
  ], [
    infoTable(["항목", "학습값", "수동값", "±std · 범위"], [
      ["조향비", [num(s.steer_ratio, 2), f.steer_ratio_valid ? "" : "bad"], num(m.steer_ratio, 2),
       `±${num(s.steer_ratio_std, 2)} · ${num(s.prior_steer_ratio * 0.5, 1)}~${num(s.prior_steer_ratio * 2, 1)}`],
      ["타이어 강성", [num(s.stiffness_factor, 3), f.stiffness_valid ? "" : "bad"], `×${num(m.tire_stiffness_factor, 2)}`,
       `±${num(s.stiffness_factor_std, 3)} · 0.2~5`],
      ["영점 평균", [`${sgn(s.angle_offset_average_deg, 2)}°`, f.offset_average_valid ? "" : "bad"], `${sgn(m.angle_offset_deg, 2)}°`,
       `±${num(deg(s.angle_offset_average_std), 2)}° · ±10°`],
      ["영점 합계(사용)", [`${sgn(s.angle_offset_deg, 2)}°`, f.offset_valid ? "" : "bad"], "–",
       `빠른 성분 ±${num(deg(s.angle_offset_fast_std), 2)}°`],
      ["도로 롤", `${sgn(deg(s.roll_rad), 2)}°`, m.live_bank_compensation ? `편경사 ${sgn(bankRoll, 2)}°` : "보정 끔", "±10°"],
    ]),
    note(`자이로 바이어스 ${sgn(deg(s.yaw_bias_rad_s), 3)}°/s · 1분마다 저장 · ${restored(f.vehicle_restored)}`),
  ]];
}

function torqueSummary(s, m) {
  const f = s.flags;
  const calculable = s.bucket_points.every(n => n > 0) && s.lat_accel_factor_raw !== 0;
  const prior = s.prior_lat_accel_factor, pf = s.prior_friction;
  const factorOut = calculable && (s.lat_accel_factor_raw < prior * 0.7 || s.lat_accel_factor_raw > prior * 1.3);
  const frictionOut = calculable && (s.friction_raw < pf * 0.5 || s.friction_raw > pf * 1.5);
  return [[
    f.torque_valid ? ["유효", "good"] : [`학습 중 ${s.cal_perc}%`, "warn"],
    f.torque_inputs_ok ? null : ["입력 끊김", "warn"],
    f.use_torque ? ["제어에 사용 중", "accent"] : ["섀도", "muted"],
  ], [
    infoTable(["항목", "필터(사용값)", "원시", "사전값 · 허용"], [
      ["배율", num(s.lat_accel_factor, 2), [calculable ? num(s.lat_accel_factor_raw, 2) : "–", factorOut ? "warn" : ""],
       `${num(prior, 2)} · ${num(prior * 0.7, 2)}~${num(prior * 1.3, 2)}`],
      ["절편 (m/s²)", sgn(s.lat_accel_offset, 3), calculable ? sgn(s.lat_accel_offset_raw, 3) : "–",
       `수동 ${sgn(m.torque_lat_accel_offset, 3)}`],
      ["마찰", num(s.friction, 3), [calculable ? num(s.friction_raw, 3) : "–", frictionOut ? "warn" : ""],
       `${num(pf, 3)} · ${num(pf * 0.5, 3)}~${num(pf * 1.5, 3)}`],
    ]),
    note(`점 ${s.total_bucket_points.toLocaleString("ko-KR")} · decay ${num(s.decay, 1)} · 초기화 `
         + `${Math.max(0, Math.round(s.max_resets) - 1)}회 · 12초마다 저장 · ${restored(f.torque_restored)}`),
  ]];
}

// ------------------------------------------------------------ 학습 입력, 카메라 보정, 조향 지연
function inputCard() {
  const key = "use_locationd_learner_inputs";
  const toggle = learnerSwitch(key);
  const card = liveCard("학습 입력 · 요레이트·롤", {control: toggle});
  return {
    element: card.element,
    toggle,
    update(learner) {
      const using = learner ? learner.flags.localizer_inputs : null;
      card.update([using === null ? ["상태 없음", "muted"] : using ? ["locationd 사용 중", "accent"] : ["ESP12 사용 중", "muted"]],
                  [note(metaFor("steering", key).description)]);
    },
  };
}

function calibrationCard() {
  let latest = null;
  const button = actionButton({icon: "↺", danger: true, onclick: () => reset()});
  const card = liveCard("카메라 캘리브레이션", {actions: [button]});
  const reset = async () => {
    const c = latest;
    if (!confirmAction(["카메라 캘리브레이션을 초기화할까요?",
                        c && c.available ? `현재 pitch ${sgn(c.rpy_deg[1], 2)}° · yaw ${sgn(c.rpy_deg[2], 2)}° · 블록 ${c.valid_blocks}` : "현재 상태 없음",
                        "", "저장된 값(calibration.json)을 지우고 0°에서 다시 수렴합니다.",
                        "수렴할 때까지(시속 24 km 이상 직진 약 30초) 조향이 부정확할 수 있습니다. 학습값은 그대로 둡니다."])) return;
    button.disabled = true;
    try {
      update(await api.post("/api/calibration/reset"));
      toast("캘리브레이션 초기화를 요청했습니다");
    } catch (error) {
      toast(`초기화 실패: ${error.message}`, {failed: true});
      button.disabled = false;
    }
  };
  const update = c => {
    latest = c;
    if (!c || !c.available) {
      card.update([["상태 없음", "muted"]], [note("modeld가 보정 상태를 발행하지 않습니다.")]);
    } else {
      const [r, p, y] = c.rpy_deg, [sr, sp, sy] = c.spread_deg;
      card.update([CALIBRATION_STATUS[c.status] || [c.status, "muted"],
                   [`블록 ${c.valid_blocks}/50`, c.valid_blocks >= 5 ? "accent" : "muted"],
                   c.age_s > 2 ? [`${Math.round(c.age_s)}초 전`, "bad"] : null], [
        infoTable(["", "현재 (°)", "블록 편차 (°)", ""], [
          ["pitch", sgn(p, 2), num(sp, 2), "+ = 아래를 봄"],
          ["yaw", sgn(y, 2), num(sy, 2), "+ = 왼쪽을 봄"],
          ["roll", sgn(r, 2), num(sr, 2), ""],
        ]),
        note("시속 24 km 이상 직진 100프레임(약 5초)이 한 블록, 5블록이 모이면 보정 완료. 마운트가 yaw 2°·pitch 4°보다 "
             + "크게 바뀌면 스스로 다시 보정하고, 그보다 작게 옮겼으면 해제 상태에서 초기화하세요."),
      ]);
    }
    const blocked = Boolean(c && c.engaged);
    button.setContent(c && c.reset_pending ? "초기화 요청됨 · modeld 대기 중" : "캘리브레이션 초기화",
                      blocked ? "결합 중에는 초기화할 수 없습니다" : "저장된 보정을 지우고 처음부터 다시 수렴합니다");
    button.disabled = blocked;
  };
  return {element: card.element, update};
}

function lagCard() {
  const toggle = learnerSwitch("use_live_delay");
  const card = liveCard("lagd · 조향 지연", {control: toggle});
  return {
    element: card.element,
    toggle,
    update(localization, learner) {
      if (!localization || !localization.available) {
        card.update([["상태 없음", "muted"]], [note("locationd가 발행하지 않습니다(IMU·locationd 확인).")]);
        return;
      }
      const s = localization.state, f = s.flags, inputs = s.input_flags;
      const poseOk = f.filter_valid && f.inputs_ok && f.sensors_ok && f.posenet_ok;
      card.update([
        LAG_STATUS[s.lag_status] || [String(s.lag_status), "muted"],
        [`블록 ${s.lag_valid_blocks}/5`, s.lag_valid_blocks >= 5 ? "accent" : "muted"],
        poseOk ? ["자세 정상", "good"] : ["자세 무효", "bad"],
        learner && learner.flags.use_delay ? ["제어에 사용 중", "accent"] : ["섀도", "muted"],
        inputs.accel_invalid ? ["가속도 거부", "warn"] : null,
        inputs.gyro_invalid ? ["자이로 거부", "warn"] : null,
        inputs.camera_invalid ? ["카메라 거부", "warn"] : null,
        inputs.camera_guarded ? ["카메라≠차속", "muted"] : null,
        localization.age_s > 2 ? [`${Math.round(localization.age_s)}초 전`, "bad"] : null,
      ], [
        infoTable(["항목", "값", "비교", ""], [
          ["쓸 지연", ms(s.lateral_delay_s), `수동 ${ms(localization.manual_delay_s || 0)}`, "추정 전에는 수동값"],
          ["경로 지연(적용)", learner && learner.plan_delay_s > 0 ? ms(learner.plan_delay_s) : "–", "", "스위치를 켜고 확정되면 추정값"],
          ["진행 평균", `${ms(s.lag_estimate_s)} ±${ms(s.lag_estimate_std_s)}`, `창 안 점 ${s.lag_points}`, "블록 사이 0.1 s 넘으면 무효"],
          ["요레이트", `${sgn(deg(s.angular_velocity_calib[2]), 2)}°/s`, `±${num(deg(s.angular_velocity_calib_std[2]), 2)}`, "+ = 오른쪽"],
          ["도로 롤 · 피치", `${sgn(deg(s.orientation_calib[0]), 2)}° · ${sgn(deg(s.orientation_calib[1]), 2)}°`, "", "IMU 중력"],
        ]),
        note("목표 곡률 → 실제 요레이트 지연(상류 lagd). 시속 40 km 이상, 조향 중·핸들 비조작·비포화 구간만 쓰고 100점마다 "
             + `한 블록. 1분마다 저장 · ${restored(f.lag_restored)}`),
      ]);
    },
  };
}

// ------------------------------------------------------------ 추이
function trendChart(chart, rows, windowS, manual, state) {
  const end = rows.length ? rows[rows.length - 1][0] : 0;
  const shown = rows.filter(row => row[0] >= end - windowS);
  const valid = (row, index, style) => Number.isFinite(row[index]) && !(style === "raw" && row[index] === 0);
  const refs = chart.ref(manual, state).filter(Number.isFinite);
  const values = [...refs];
  for (const [index, style] of chart.series) for (const row of shown) if (valid(row, index, style)) values.push(row[index]);
  let lo = values.length ? Math.min(...values) : 0, hi = values.length ? Math.max(...values) : 1;
  if (hi - lo < chart.span) {
    const mid = (hi + lo) / 2;
    lo = mid - chart.span / 2;
    hi = mid + chart.span / 2;
  }
  const pad = (hi - lo) * 0.08;
  lo -= pad;
  hi += pad;
  const W = 300, H = 90;
  const x = t => ((t - (end - windowS)) / windowS) * W;
  const y = v => H - ((v - lo) / (hi - lo)) * H;
  const plot = svg("svg", {viewBox: `0 0 ${W} ${H}`, preserveAspectRatio: "none", role: "img", "aria-label": chart.title});
  refs.forEach((value, i) => plot.append(svg("line", {
    x1: 0, x2: W, y1: y(value), y2: y(value), stroke: i === 0 ? "#8a949b" : "#5b646b", "stroke-dasharray": i === 0 ? "5 4" : "2 4",
    "vector-effect": "non-scaling-stroke",
  })));
  const stroke = {main: "#58a6e7", thin: "#a5adb4", dash: "#a5adb4", raw: "#efb85b"};
  for (const [index, style] of chart.series) {
    const points = shown.filter(row => valid(row, index, style)).map(row => `${x(row[0]).toFixed(1)},${y(row[index]).toFixed(1)}`);
    if (points.length < 2) continue;
    plot.append(svg("polyline", {
      points: points.join(" "), fill: "none", stroke: stroke[style], "stroke-width": style === "main" ? 2 : 1.2,
      "vector-effect": "non-scaling-stroke", ...(style === "dash" ? {"stroke-dasharray": "4 3"} : {}),
    }));
  }
  const last = shown.length ? shown[shown.length - 1] : null;
  return el("article.card.trend",
    {},
    el("div.trend-head", {}, el("span", {text: chart.title}),
       el("span.trend-now", {text: last ? chart.series.map(([index, style]) => (valid(last, index, style) ? num(last[index], chart.digits) : "–")).join(" / ") : "–"})),
    plot,
    el("div.trend-range", {}, el("span", {text: `${num(lo, chart.digits)} ~ ${num(hi, chart.digits)}`}),
       el("span", {text: refs.length ? `점선 ${num(refs[0], chart.digits)}` : "최근 10분"})));
}

// ------------------------------------------------------------ 탭
export function createLearnerView() {
  const vehicle = adoptCard("paramsd · 차량 값", ADOPT.vehicle, vehicleSummary);
  const torque = adoptCard("torqued · 토크 값", ADOPT.torque, torqueSummary);
  const input = inputCard();
  const calibration = calibrationCard();
  const lag = lagCard();
  const notice = el("div");
  const trends = el("div.card-grid");
  let rows = [];
  let windowS = 600;
  let trendLoaded = false;

  const update = data => {
    const learner = data.available ? data.state : null;
    calibration.update(data.calibration);
    lag.update(data.localization, learner);
    input.update(learner);
    if (!learner) {
      notice.replaceChildren(el("p.group-note.warn", {text: "학습 상태가 없습니다. controlsd가 실행 중인지 확인하세요."}));
      vehicle.clear();
      torque.clear();
      return;
    }
    notice.replaceChildren(data.age_s > 2
      ? el("p.group-note.warn", {text: `학습 상태가 ${Math.round(data.age_s)}초째 갱신되지 않습니다. controlsd를 확인하세요.`}) : "");
    vehicle.update(learner, data.manual);
    torque.update(learner, data.manual);
    const row = data.trend_row;
    if (row && rows.length && row[0] < rows[rows.length - 1][0]) rows = [];
    if (row && (!rows.length || row[0] > rows[rows.length - 1][0])) {
      rows.push(row);
      while (rows.length && rows[0][0] < row[0] - windowS) rows.shift();
    }
    trends.replaceChildren(...TREND_CHARTS.map(chart => trendChart(chart, rows, windowS, data.manual, learner)));
  };

  const polling = poller(async isCurrent => {
    try {
      if (!trendLoaded) {
        const trend = await api.get("/api/learners/trend");
        rows = trend.rows;
        windowS = trend.window_s;
        trendLoaded = true;
      }
      const data = await api.get("/api/learners");
      if (isCurrent()) update(data);
    } catch (error) {
      if (isCurrent()) notice.replaceChildren(el("p.group-note.warn", {text: `학습 상태 읽기 실패: ${error.message}`}));
    }
  }, 1000);

  return {
    element: el("div.view",
      {},
      el("p.group-note", {text: NOTE}),
      notice,
      section("학습값 · 오른쪽 스위치로 제어에 사용", [vehicle.element, torque.element]),
      section("학습 입력", [input.element]),
      section("카메라 장착 · 조향 지연", [calibration.element, lag.element]),
      el("section.section", {}, el("h2.section-title", {text: "최근 10분 추이"}), trends)),
    start() {
      // 가려져 있던 동안의 추이는 서버에서 다시 받는다
      trendLoaded = false;
      for (const card of [vehicle, torque, input, lag]) card.toggle.sync();
      polling.start();
    },
    stop: polling.stop,
  };
}

