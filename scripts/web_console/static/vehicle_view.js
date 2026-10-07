// 차량 특성 탭: 차의 성질인 조향 값들(steering.json의 tab="vehicle" 항목). 학습기(paramsd·torqued·lagd)마다 한 묶음이고,
// 묶음 머리에서 제어에 학습값을 쓸지(자동 학습) 수동값을 쓸지(수동) 고른다. 값마다 카드 하나에 지금 제어에 쓰는 값과
// 그 출처, 학습값, 수동값(바로 고친다)과 자동 학습 중 그 쓰임을 함께 둔다. 그 아래 카메라 보정, 최근 10분 추이, 학습
// 입력, 고정 제원. 1초마다 /api/learners를 읽는다.
import {clampToMeta, defaultOf, metaFor, params, saveParam, valueOf} from "./param_state.js";
import {effectsDetails, formatValue, paramCard, paramEditor} from "./param_editor.js";
import {
  actionButton, api, badges, clock, confirmAction, deg, el, infoTable, liveCard, ms, note, num, poller, section, sgn, svg,
  toast,
} from "./ui.js";

const GRAVITY = 9.81;
const steering = () => params().params.steering;
const restored = flag => (flag ? "이번 시동에 복원" : "새로 시작");
const lagConfirmed = localization => Boolean(localization) && localization.state.lag_status === 1;
const torqueCalculable = s => s.bucket_points.every(n => n > 0) && s.lat_accel_factor_raw !== 0;

/* torqued 값의 원시값(필터 전)과 허용 폭. 원시값이 폭 밖이면 주의 톤. */
function torqueDetail(s, raw, prior, width, digits) {
  const calculable = torqueCalculable(s);
  const outside = calculable && (raw < prior * (1 - width) || raw > prior * (1 + width));
  return [`원시 ${calculable ? num(raw, digits) : "–"} · 허용 ${num(prior * (1 - width), digits)}~${num(prior * (1 + width), digits)}`,
          outside ? "warn" : ""];
}

/* 값마다 학습값을 읽는 법. live = {learner: LearnerState, localization: locationd 상태 또는 null}.
 *   learned(live)            학습값(파라미터 단위), 없으면 NaN. valid(live)는 그 값이 유효한가
 *   detail(live)             학습값 옆 작은 글씨(편차·범위), 또는 [글씨, 톤]
 *   digits, signed           학습값과 지금 쓰는 값의 자릿수·부호. 학습값을 수동값으로 저장할 때도 이 자릿수
 *   using(live, manual)      자동 학습 중 실제로 쓰는 값(없으면 학습값). both면 수동값도 함께 쓰인다
 *   learnedText, manualText  값 대신 보여 줄 글씨
 *   adopt                    수동값으로 저장할 값(없으면 학습값), false면 버튼 없음. adoptLabel은 그 값의 이름 */
const LIVE = {
  steer_ratio: {
    digits: 2,
    learned: ({learner: s}) => s.steer_ratio,
    valid: ({learner: s}) => s.flags.steer_ratio_valid,
    detail: ({learner: s}) => `±${num(s.steer_ratio_std, 2)} · 범위 ${num(s.prior_steer_ratio * 0.5, 1)}~${num(s.prior_steer_ratio * 2, 1)}`,
  },
  tire_stiffness_factor: {
    digits: 2,
    learned: ({learner: s}) => s.stiffness_factor,
    learnedText: value => `×${num(value, 3)}`,
    valid: ({learner: s}) => s.flags.stiffness_valid,
    detail: ({learner: s}) => `배율 ±${num(s.stiffness_factor_std, 3)} · 0.2~5`,
    using: ({learner: s}, manual) => manual * s.stiffness_factor,
    both: true,
    adopt: false,
  },
  angle_offset_deg: {
    digits: 2,
    signed: true,
    learned: ({learner: s}) => s.angle_offset_deg,
    valid: ({learner: s}) => s.flags.offset_valid,
    detail: ({learner: s}) => `평균 ${sgn(s.angle_offset_average_deg, 2)} ±${num(deg(s.angle_offset_average_std), 2)} · `
      + `빠른 성분 ±${num(deg(s.angle_offset_fast_std), 2)}`,
    adopt: ({learner: s}) => s.angle_offset_average_deg,
    adoptLabel: "평균",
  },
  live_bank_compensation: {
    title: "도로 기울기 보정",
    digits: 2,
    signed: true,
    learned: ({learner: s}) => deg(s.roll_rad),
    learnedText: value => `롤 ${sgn(value, 2)}°`,
    valid: ({learner: s}) => s.flags.vehicle_valid,
    detail: () => "학습한 도로 롤",
    manualText: ({learner: s}, on) => (on ? `편경사 ${sgn(deg(-s.road_bank_lat_accel / GRAVITY), 2)}°` : "보정 안 함"),
    adopt: false,
  },
  torque_lat_accel_factor: {
    digits: 2,
    learned: ({learner: s}) => s.lat_accel_factor,
    valid: ({learner: s}) => s.flags.torque_valid,
    detail: ({learner: s}) => torqueDetail(s, s.lat_accel_factor_raw, s.prior_lat_accel_factor, 0.3, 2),
  },
  torque_friction: {
    digits: 3,
    learned: ({learner: s}) => s.friction,
    valid: ({learner: s}) => s.flags.torque_valid,
    detail: ({learner: s}) => torqueDetail(s, s.friction_raw, s.prior_friction, 0.5, 3),
  },
  torque_lat_accel_offset: {
    digits: 3,
    signed: true,
    learned: ({learner: s}) => s.lat_accel_offset,
    valid: ({learner: s}) => s.flags.torque_valid,
    detail: ({learner: s}) => `원시 ${torqueCalculable(s) ? sgn(s.lat_accel_offset_raw, 3) : "–"}`,
  },
  steer_actuator_delay: {
    digits: 3,
    learned: ({localization}) => (lagConfirmed(localization) ? localization.state.lateral_delay_s : NaN),
    learnedText: (value, {localization}) => (Number.isFinite(value) ? num(value, 3) : localization ? "추정 전" : "–"),
    valid: ({localization}) => lagConfirmed(localization),
    detail: ({localization}) => (!localization ? "locationd 상태 없음"
      : `진행 평균 ${num(localization.state.lag_estimate_s, 3)} ±${ms(localization.state.lag_estimate_std_s)} · `
        + `블록 ${localization.state.lag_valid_blocks}/5`),
    using: ({learner: s}, manual) => (s.plan_delay_s > 0 ? s.plan_delay_s : manual),  // 실제로 쓴 경로 지연
  },
};

const LAG_STATUS = {0: ["추정 전", "warn"], 1: ["추정됨", "good"], 2: ["무효", "bad"]};
/* 학습기마다: 지금 제어에 학습값을 쓰고 있는가(controlsd가 LearnerState 플래그로 알린다), 쓰기 전의 상태 이름, 상태
 * 배지, 한 줄 요약. */
const LEARNERS = {
  paramsd: {
    inUse: ({learner: s}) => s.flags.use_vehicle,
    waiting: "학습 대기 · 수동값",
    status: ({learner: s}) => [
      s.flags.vehicle_valid ? ["유효", "good"] : ["무효", "bad"],
      s.flags.vehicle_inputs_ok ? null : ["입력 끊김", "warn"],
      s.flags.sensor_valid ? null : ["센서 불일치", "warn"],
      s.flags.localizer_inputs ? ["입력 locationd", "muted"] : ["입력 ESP12", "muted"],
    ],
    summary: ({learner: s}) => `자이로 바이어스 ${sgn(deg(s.yaw_bias_rad_s), 3)}°/s · 1분마다 저장 · ${restored(s.flags.vehicle_restored)}`,
  },
  torqued: {
    inUse: ({learner: s}) => s.flags.use_torque,
    waiting: "학습 대기 · 수동값",
    status: ({learner: s}) => [
      s.flags.torque_valid ? ["유효", "good"] : [`학습 중 ${s.cal_perc}%`, "warn"],
      s.flags.torque_inputs_ok ? null : ["입력 끊김", "warn"],
    ],
    summary: ({learner: s}) => `점 ${s.total_bucket_points.toLocaleString("ko-KR")} · decay ${num(s.decay, 1)} · `
      + `초기화 ${Math.max(0, Math.round(s.max_resets) - 1)}회 · 12초마다 저장 · ${restored(s.flags.torque_restored)}`,
  },
  lagd: {
    inUse: ({learner: s}) => s.flags.use_delay,
    waiting: "확정 전 · 수동값",
    status: ({localization}) => {
      if (!localization) return [["locationd 없음", "bad"]];
      const s = localization.state, f = s.flags, inputs = s.input_flags;
      return [
        LAG_STATUS[s.lag_status] || [String(s.lag_status), "muted"],
        [`블록 ${s.lag_valid_blocks}/5`, s.lag_valid_blocks >= 5 ? "accent" : "muted"],
        f.filter_valid && f.inputs_ok && f.sensors_ok && f.posenet_ok ? ["자세 정상", "good"] : ["자세 무효", "bad"],
        inputs.accel_invalid ? ["가속도 거부", "warn"] : null,
        inputs.gyro_invalid ? ["자이로 거부", "warn"] : null,
        inputs.camera_invalid ? ["카메라 거부", "warn"] : null,
        inputs.camera_guarded ? ["카메라≠차속", "muted"] : null,
        localization.age_s > 2 ? [`${Math.round(localization.age_s)}초 전`, "bad"] : null,
      ];
    },
    summary: ({localization}) => {
      if (!localization) return "locationd가 발행하지 않습니다(IMU·locationd 확인).";
      const s = localization.state;
      return `요레이트 ${sgn(deg(s.angular_velocity_calib[2]), 2)}°/s · 도로 롤 ${sgn(deg(s.orientation_calib[0]), 2)}° · `
        + `피치 ${sgn(deg(s.orientation_calib[1]), 2)}° · 창 안 점 ${s.lag_points} · 1분마다 저장 · ${restored(s.flags.lag_restored)}`;
    },
  },
};

// ------------------------------------------------------------ 학습기 묶음
/* 묶음 머리의 출처 선택: 자동 학습(학습 스위치 켬) | 수동(끔). 고르면 곧바로 저장하고 onChange를 부른다. */
function sourceControl(switchKey, label, onChange) {
  const auto = el("button", {type: "button", text: "자동 학습"});
  const manual = el("button", {type: "button", text: "수동"});
  const element = el("div.segmented", {role: "group", "aria-label": `${label} 값 출처`}, auto, manual);
  const sync = () => {
    const on = steering()[switchKey] === true;
    auto.setAttribute("aria-pressed", String(on));
    manual.setAttribute("aria-pressed", String(!on));
  };
  const choose = async on => {
    if ((steering()[switchKey] === true) === on) return;
    auto.disabled = manual.disabled = true;
    try {
      await saveParam("steering", switchKey, on);
      toast(`${label}: ${on ? "자동 학습" : "수동"} · ${clock()}`);
    } catch (error) {
      toast(`${label}: ${error.message}`, {failed: true});
    } finally {
      auto.disabled = manual.disabled = false;
      sync();
      onChange();
    }
  };
  auto.addEventListener("click", () => choose(true));
  manual.addEventListener("click", () => choose(false));
  sync();
  return {element, sync};
}

/* 값 카드: 지금 쓰는 값과 출처, 학습값, 수동값(편집기)과 자동 학습 중 그 쓰임, 학습값을 수동값으로 저장, 설명. */
function quantityCard(key, switchKey, learner) {
  const meta = metaFor("steering", key);
  const spec = LIVE[key];
  const card = el("article.card.quantity", {dataset: {key}});
  let live = null;
  const editor = paramEditor("steering", key, {card, onSaved: () => render()});
  const format = (value, digits = spec.digits) => (spec.signed ? sgn : num)(value, digits);
  const source = el("span.badge");
  const now = el("span.quantity-now");
  const learnedValue = el("span.quantity-value");
  const learnedDetail = el("span.quantity-detail");
  const learnedRow = el("div.quantity-row", {}, el("span.quantity-label", {text: "학습값"}), learnedValue, learnedDetail);
  const fallback = defaultOf("steering", key);
  const manualRow = el("div.quantity-row.manual",
    {},
    el("span.quantity-label", {text: "수동값"}),
    el("span.quantity-foot",
      {},
      fallback === undefined ? null : el("span", {text: `기본 ${formatValue(fallback, meta)}`}),
      editor.modifiedBadge, editor.resetButton, editor.status),
    editor.control.element);
  const role = el("p.card-note.prior");
  const adopt = spec.adopt === false ? null : el("button.adopt-button", {type: "button", onclick: () => adoptLearned()});
  const adoptValue = () => (typeof spec.adopt === "function" ? spec.adopt(live) : spec.learned(live));
  const showNow = (text, unit = "") => now.replaceChildren(text, unit ? el("small", {text: ` ${unit}`}) : "");
  // 수동값을 쓰는 중일 때의 큰 글씨: manualText가 있으면 그것, 켜고 끄는 값이면 켜짐·꺼짐, 숫자면 단위와 함께
  const showManual = manual => {
    if (spec.manualText && live) showNow(spec.manualText(live, manual));
    else if (typeof manual === "boolean") showNow(manual ? "켜짐" : "꺼짐");
    else showNow(String(manual), meta.unit);
  };

  const updateAdopt = valid => {
    if (!adopt) return;
    const current = valueOf("steering", key);
    const target = live ? clampToMeta(adoptValue(), meta, spec.digits) : NaN;
    const same = Number(current) === Number(target);
    adopt.replaceChildren(
      el("span", {text: `⤓ 학습값${spec.adoptLabel ? `(${spec.adoptLabel})` : ""}을 수동값으로`}),
      el("small", {text: !live || !valid || !Number.isFinite(target) ? "학습값이 유효해지면 쓸 수 있습니다"
        : same ? "수동값과 같습니다" : `${current} → ${target}`}));
    adopt.disabled = !live || !valid || same || !Number.isFinite(target);
  };

  const adoptLearned = async () => {
    if (!live) return;
    const current = valueOf("steering", key);
    const target = clampToMeta(adoptValue(), meta, spec.digits);
    const auto = steering()[switchKey] === true;
    if (!confirmAction([`${meta.label} (${key})`, `${current} → ${target}`, "",
                        auto ? `자동 학습 중에는 ${meta.learned_role}` : "수동이라 지금 바로 제어에 반영됩니다.",
                        meta.caution || null])) return;
    await editor.save(target);
  };

  function render() {
    const auto = steering()[switchKey] === true;
    const manual = valueOf("steering", key);
    role.textContent = auto && meta.learned_role ? `자동 학습 중에는 ${meta.learned_role}` : "";
    role.hidden = !role.textContent;
    if (!live) {
      source.className = "badge muted";
      source.textContent = auto ? "상태 없음" : "수동값";
      if (auto) showNow("–");
      else showManual(manual);
      learnedValue.textContent = "–";
      learnedValue.className = "quantity-value";
      learnedDetail.textContent = "controlsd 상태 없음";
      learnedRow.classList.remove("in-use");
      manualRow.classList.toggle("in-use", !auto);
      updateAdopt(false);
      return;
    }
    const inUse = auto && learner.inUse(live);
    const learned = spec.learned(live);
    const valid = spec.valid(live);
    learnedValue.textContent = spec.learnedText ? spec.learnedText(learned, live) : format(learned);
    learnedValue.className = `quantity-value${valid ? "" : " bad"}`;
    const detail = spec.detail(live);
    const [detailText, detailTone] = Array.isArray(detail) ? detail : [detail, ""];
    learnedDetail.textContent = detailText;
    learnedDetail.className = `quantity-detail${detailTone ? ` ${detailTone}` : ""}`;
    if (inUse) {
      const value = spec.using ? spec.using(live, manual) : learned;
      if (spec.learnedText && !spec.using) showNow(spec.learnedText(value, live));
      else showNow(format(value), meta.unit);
      source.className = "badge good";
      source.textContent = spec.both ? "수동값 × 학습 배율" : "학습값";
    } else {
      showManual(manual);
      source.className = `badge ${auto ? "warn" : "muted"}`;
      source.textContent = auto ? learner.waiting : "수동값";
    }
    learnedRow.classList.toggle("in-use", inUse);
    manualRow.classList.toggle("in-use", !inUse || Boolean(spec.both));
    updateAdopt(valid);
  }

  card.append(...[
    el("header.card-head", {}, el("h3.card-title", {text: spec.title || meta.label}), source),
    el("div.quantity-now-row", {}, now, el("span.quantity-caption", {text: "지금 쓰는 값"})),
    learnedRow,
    manualRow,
    role,
    meta.caution ? note(meta.caution, "warn") : null,
    adopt,
    effectsDetails(meta, typeof valueOf("steering", key) === "boolean",
                   {summary: "설명", lead: el("p.param-description", {text: meta.description})}),
  ].filter(Boolean));
  render();
  return {
    element: card,
    update(next) {
      live = next;
      render();
    },
    refresh() {
      editor.refresh();
      render();
    },
  };
}

/* 학습기 묶음 하나: 이름·학습기·설명, 출처 선택, 상태 배지, 값 카드들, 한 줄 요약. */
function learnerGroup(name, spec, keys) {
  const learner = LEARNERS[spec.learner];
  const cards = keys.map(key => quantityCard(key, spec.switch, learner));
  const statusRow = el("div.badge-row");
  const summary = el("p.group-summary");
  let live = null;
  const update = next => {
    live = next;
    const auto = steering()[spec.switch] === true;
    statusRow.replaceChildren(...badges([
      !live ? ["상태 없음", "muted"]
        : !auto ? ["제어에 수동값", "muted"]
        : learner.inUse(live) ? ["제어에 학습값", "accent"] : [learner.waiting, "warn"],
      ...(live ? learner.status(live) : []),
    ]));
    summary.textContent = live ? learner.summary(live) : "controlsd가 학습 상태를 내지 않습니다.";
    cards.forEach(card => card.update(live));
  };
  const source = sourceControl(spec.switch, name, () => update(live));
  return {
    element: el("section.section.learner-group",
      {},
      el("div.group-head",
        {},
        el("div.group-title",
          {},
          el("h2.section-title", {}, name, el("span", {text: spec.learner})),
          el("p.group-description", {text: metaFor("steering", spec.switch).description})),
        source.element),
      el("div.group-status", {}, statusRow),
      el("div.card-grid", {}, cards.map(card => card.element)),
      summary),
    update,
    refresh() {
      source.sync();
      cards.forEach(card => card.refresh());
      update(live);
    },
  };
}

// ------------------------------------------------------------ 카메라 보정
const CALIBRATION_STATUS = {
  calibrated: ["보정 완료", "good"], uncalibrated: ["수렴 중", "warn"], invalid: ["범위 밖", "bad"],
  recalibrating: ["장착 변경 · 재보정 중", "warn"],
};

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

// ------------------------------------------------------------ 추이
const TREND_CHARTS = [
  {title: "조향비", series: [[1, "main"]], ref: m => [m.steer_ratio], span: 0.2, digits: 2},
  {title: "조향각 영점 (°) · 평균 / 합계", series: [[2, "main"], [3, "thin"]], ref: m => [m.angle_offset_deg], span: 0.2, digits: 2},
  {title: "도로 롤 (°) · 학습 / 편경사 환산", series: [[4, "main"], [5, "dash"]], ref: () => [], span: 0.5, digits: 2},
  {title: "토크 배율 · 필터 / 원시", series: [[7, "main"], [6, "raw"]],
   ref: (m, s) => [s.prior_lat_accel_factor, s.prior_lat_accel_factor * 0.7, s.prior_lat_accel_factor * 1.3], span: 0.2, digits: 2},
];

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
export function createVehicleView() {
  const tab = params().vehicle_tab;
  const keysOf = name => Object.keys(steering()).filter(key => {
    const meta = metaFor("steering", key);
    return meta.tab === "vehicle" && meta.section === name;
  });
  const groups = tab.sections.filter(name => tab.learners[name]).map(name => {
    const spec = tab.learners[name];
    return learnerGroup(name, spec, keysOf(name).filter(key => key !== spec.switch));
  });
  const plain = tab.sections.filter(name => !tab.learners[name]).map(name => ({name, cards: keysOf(name).map(key => paramCard("steering", key))}));
  const calibration = calibrationCard();
  const notice = el("div");
  const trends = el("div.card-grid");
  let rows = [];
  let windowS = 600;
  let trendLoaded = false;

  const update = data => {
    calibration.update(data.calibration);
    if (!data.available) {
      notice.replaceChildren(el("p.group-note.warn", {text: "학습 상태가 없습니다. controlsd가 실행 중인지 확인하세요."}));
      groups.forEach(group => group.update(null));
      return;
    }
    notice.replaceChildren(data.age_s > 2
      ? el("p.group-note.warn", {text: `학습 상태가 ${Math.round(data.age_s)}초째 갱신되지 않습니다. controlsd를 확인하세요.`}) : "");
    const live = {learner: data.state, localization: data.localization && data.localization.available ? data.localization : null};
    groups.forEach(group => group.update(live));
    const row = data.trend_row;
    if (row && rows.length && row[0] < rows[rows.length - 1][0]) rows = [];
    if (row && (!rows.length || row[0] > rows[rows.length - 1][0])) {
      rows.push(row);
      while (rows.length && rows[0][0] < row[0] - windowS) rows.shift();
    }
    trends.replaceChildren(...TREND_CHARTS.map(chart => trendChart(chart, rows, windowS, steering(), data.state)));
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
      el("p.group-note", {text: tab.note}),
      notice,
      groups.map(group => group.element),
      section("카메라 보정", [calibration.element]),
      el("section.section", {}, el("h2.section-title", {text: "최근 10분 추이"}), trends),
      plain.map(({name, cards}) => section(name, cards, {count: cards.length})),
      el("p.file-path", {text: `파일: ${params().paths.steering}`})),
    start() {
      // 가려져 있던 동안의 추이는 서버에서 다시 받는다
      trendLoaded = false;
      groups.forEach(group => group.refresh());
      plain.forEach(({cards}) => cards.forEach(card => card.refresh()));
      polling.start();
    },
    stop: polling.stop,
  };
}
