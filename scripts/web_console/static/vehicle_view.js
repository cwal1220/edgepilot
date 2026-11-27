// 차량 특성 탭: 차의 성질인 조향 값들(steering.json의 tab="vehicle" 항목). 학습기(paramsd·torqued·lagd)마다 한 묶음이고,
// 묶음 머리에서 제어에 학습값을 쓸지(자동 학습) 수동값을 쓸지(수동) 고른다. 값마다 카드 하나에 지금 제어에 쓰는 값과
// 그 출처, 학습값, 수동값(바로 고친다)과 자동 학습 중 그 쓰임을 함께 둔다. 그 아래 학습 입력, 최근 10분 추이, 카메라
// 보정, 고정 제원(VEHICLE_TAB 섹션 순서에 추이와 보정을 끼운다). 1초마다 /api/learners를 읽는다.
import {codeText, locale, t} from "./i18n.js";
import {
  clampToMeta, defaultOf, metaFor, onParamsChanged, params, sectionName, updateRuntime, valueOf,
} from "./param_state.js";
import {conditionNoteList, effectsDetails, formatValue, paramEditor} from "./param_editor.js";
import {
  actionButton, api, badges, confirmAction, DASH, deg, el, fitRange, helpText, infoTable, liveCard, ms, note, num, poller,
  runAction, section, segmented, setBadge, sgn, sparkline, staleBadge,
} from "./ui.js";

const GRAVITY = 9.81;  // 편경사 롤 환산. learner_monitor.py와 같은 값과 식이다(check_web_console.py가 대조)
const steering = () => params().params.steering;
const restored = flag => (flag ? t("vehicle.restored") : t("vehicle.fresh"));
const lagConfirmed = localization => Boolean(localization) && localization.state.lag_status === 1;
// lagd 유효 블록 수: 5개부터 확정이고 50개까지 는다
const lagBlocks = blocks => t(blocks >= 5 ? "vehicle.blocks_confirmed" : "vehicle.blocks", {blocks});
const torqueCalculable = s => s.bucket_points.every(n => n > 0) && s.lat_accel_factor_raw !== 0;

// torqued 값의 원시값(필터 전)과 허용 폭. 원시값이 폭 밖이면 주의 톤.
function torqueDetail(s, raw, prior, width, digits) {
  const calculable = torqueCalculable(s);
  const outside = calculable && (raw < prior * (1 - width) || raw > prior * (1 + width));
  return [t("vehicle.torque_detail", {raw: calculable ? num(raw, digits) : DASH, low: num(prior * (1 - width), digits),
                                       high: num(prior * (1 + width), digits)}),
          outside ? "warn" : ""];
}

// 값마다 학습값을 읽는 법. live = {learner: LearnerState, localization: locationd 상태 또는 null}.
//   learned(live)            학습값(파라미터 단위), 없으면 NaN. valid(live)는 그 값이 유효한가
//   detail(live)             학습값 옆 작은 글씨(편차·범위), 또는 [글씨, 톤]
//   digits, signed           학습값과 지금 쓰는 값의 자릿수·부호. 학습값을 수동값으로 저장할 때도 이 자릿수
//   using(live, manual)      자동 학습 중 실제로 쓰는 값(없으면 학습값). both면 수동값도 함께 쓰인다
//   learnedText, manualText  값 대신 보여 줄 글씨
//   adopt                    수동값으로 저장할 값(없으면 학습값), false면 버튼 없음. adoptLabel()은 그 값의 이름
//   title()                  카드 제목(없으면 항목 이름)
const LIVE = {
  steer_ratio: {
    digits: 2,
    learned: ({learner: s}) => s.steer_ratio,
    valid: ({learner: s}) => s.flags.steer_ratio_valid,
    detail: ({learner: s}) => t("vehicle.ratio_detail", {std: num(s.steer_ratio_std, 2), low: num(s.prior_steer_ratio * 0.5, 1),
                                                         high: num(s.prior_steer_ratio * 2, 1)}),
  },
  tire_stiffness_factor: {
    digits: 2,
    learned: ({learner: s}) => s.stiffness_factor,
    learnedText: value => `×${num(value, 3)}`,
    valid: ({learner: s}) => s.flags.stiffness_valid,
    detail: ({learner: s}) => t("vehicle.stiffness_detail", {std: num(s.stiffness_factor_std, 3)}),
    using: ({learner: s}, manual) => manual * s.stiffness_factor,
    both: true,
    adopt: false,
  },
  angle_offset_deg: {
    digits: 2,
    signed: true,
    learned: ({learner: s}) => s.angle_offset_deg,
    valid: ({learner: s}) => s.flags.offset_valid,
    detail: ({learner: s}) => t("vehicle.offset_detail", {average: sgn(s.angle_offset_average_deg, 2),
                                                          std: num(deg(s.angle_offset_average_std), 2),
                                                          fast: num(deg(s.angle_offset_fast_std), 2)}),
    adopt: ({learner: s}) => s.angle_offset_average_deg,
    adoptLabel: () => t("vehicle.adopt_average"),
  },
  live_bank_compensation: {
    title: () => t("vehicle.bank_title"),
    digits: 2,
    signed: true,
    learned: ({learner: s}) => deg(s.roll_rad),
    learnedText: value => t("vehicle.roll_value", {roll: sgn(value, 2)}),
    valid: ({learner: s}) => s.flags.vehicle_valid,
    detail: () => t("vehicle.roll_detail"),
    manualText: ({learner: s}, on) => (on ? t("vehicle.bank_value", {bank: sgn(deg(-s.road_bank_lat_accel / GRAVITY), 2)})
      : t("vehicle.bank_off")),
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
    detail: ({learner: s}) => t("vehicle.raw_detail", {raw: torqueCalculable(s) ? sgn(s.lat_accel_offset_raw, 3) : DASH}),
  },
  steer_actuator_delay: {
    digits: 3,
    learned: ({localization}) => (lagConfirmed(localization) ? localization.state.lateral_delay_s : NaN),
    learnedText: (value, {localization}) => (Number.isFinite(value) ? num(value, 3) : localization ? t("vehicle.lag_pending") : DASH),
    valid: ({localization}) => lagConfirmed(localization),
    detail: ({localization}) => (!localization ? t("vehicle.no_locationd_state")
      : t("vehicle.lag_detail", {estimate: num(localization.state.lag_estimate_s, 3), std: ms(localization.state.lag_estimate_std_s),
                                 blocks: lagBlocks(localization.state.lag_valid_blocks)})),
    using: ({learner: s}, manual) => (s.plan_delay_s > 0 ? s.plan_delay_s : manual),  // 실제로 쓴 경로 지연
  },
};

const LAG_STATUS = {0: ["vehicle.lag_pending", "warn"], 1: ["vehicle.lag_estimated", "good"], 2: ["vehicle.invalid", "bad"]};
// 학습기마다: 지금 제어에 학습값을 쓰고 있는가(controlsd가 LearnerState 플래그로 알린다), 쓰기 전의 상태 이름, 상태
// 배지, 한 줄 요약.
const LEARNERS = {
  paramsd: {
    inUse: ({learner: s}) => s.flags.use_vehicle,
    waiting: () => t("vehicle.waiting"),
    status: ({learner: s}) => [
      s.flags.vehicle_valid ? [t("vehicle.valid"), "good"] : [t("vehicle.invalid"), "bad"],
      s.flags.vehicle_inputs_ok ? null : [t("vehicle.inputs_lost"), "warn"],
      s.flags.sensor_valid ? null : [t("vehicle.sensor_mismatch"), "warn"],
      [t("vehicle.input_source", {source: s.flags.localizer_inputs ? "locationd" : "ESP12"}), "muted"],
    ],
    summary: ({learner: s}) => t("vehicle.paramsd_summary", {bias: sgn(deg(s.yaw_bias_rad_s), 3),
                                                             restored: restored(s.flags.vehicle_restored)}),
  },
  torqued: {
    inUse: ({learner: s}) => s.flags.use_torque,
    waiting: () => t("vehicle.waiting"),
    status: ({learner: s}) => [
      s.flags.torque_valid ? [t("vehicle.valid"), "good"] : [t("vehicle.learning", {percent: s.cal_perc}), "warn"],
      s.flags.torque_inputs_ok ? null : [t("vehicle.inputs_lost"), "warn"],
    ],
    summary: ({learner: s}) => t("vehicle.torqued_summary", {
      points: s.total_bucket_points.toLocaleString(locale), decay: num(s.decay, 1),
      count: Math.max(0, Math.round(s.max_resets) - 1),
      restored: restored(s.flags.torque_restored),
    }),
  },
  lagd: {
    inUse: ({learner: s}) => s.flags.use_delay,
    waiting: () => t("vehicle.lag_waiting"),
    status: ({localization}) => {
      if (!localization) return [[t("vehicle.no_locationd"), "bad"]];
      const s = localization.state, f = s.flags, inputs = s.input_flags;
      const lag = LAG_STATUS[s.lag_status];
      return [
        lag ? [t(lag[0]), lag[1]] : [String(s.lag_status), "muted"],
        [lagBlocks(s.lag_valid_blocks), s.lag_valid_blocks >= 5 ? "accent" : "muted"],
        f.filter_valid && f.inputs_ok && f.sensors_ok && f.posenet_ok ? [t("vehicle.pose_ok"), "good"] : [t("vehicle.pose_invalid"), "bad"],
        inputs.accel_invalid ? [t("vehicle.accel_rejected"), "warn"] : null,
        inputs.gyro_invalid ? [t("vehicle.gyro_rejected"), "warn"] : null,
        inputs.camera_invalid ? [t("vehicle.camera_rejected"), "warn"] : null,
        inputs.camera_guarded ? [t("vehicle.camera_guarded"), "muted"] : null,
        staleBadge(localization),
      ];
    },
    summary: ({localization}) => {
      if (!localization) return t("vehicle.locationd_silent");
      const s = localization.state;
      return t("vehicle.lagd_summary", {
        yaw_rate: sgn(deg(s.angular_velocity_calib[2]), 2), roll: sgn(deg(s.orientation_calib[0]), 2),
        pitch: sgn(deg(s.orientation_calib[1]), 2), points: s.lag_points, restored: restored(s.flags.lag_restored),
      });
    },
  },
};

// ------------------------------------------------------------ 학습기 묶음
// 값 카드: 지금 쓰는 값과 출처, 설명, 학습값, 수동값(편집기)과 자동 학습 중 그 쓰임, 학습값을 수동값으로 저장, 바꾸면.
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
  const label = text => el("span.quantity-label", {text, dataset: {inUse: t("common.in_use")}});
  const learnedRow = el("div.quantity-row", {}, label(t("vehicle.learned")), learnedValue, learnedDetail);
  const fallback = defaultOf("steering", key);
  const manualRow = el("div.quantity-row.manual",
    {},
    label(t("vehicle.manual_value")),
    el("span.quantity-foot",
      {},
      el("span", {text: t("params.default", {value: formatValue(fallback, meta)})}),
      editor.modifiedBadge, editor.resetButton, editor.status),
    editor.control.element);
  const role = el("p.card-note.prior");
  const conditions = conditionNoteList("steering", key, card);
  const adopt = spec.adopt === false ? null : actionButton({icon: "↓", onclick: () => adoptLearned()});
  const adoptValue = () => (typeof spec.adopt === "function" ? spec.adopt(live) : spec.learned(live));
  const adoptTarget = () => (live ? clampToMeta(adoptValue(), meta, spec.digits) : NaN);
  const showNow = (text, unit = "") => now.replaceChildren(text, unit ? el("small", {text: ` ${unit}`}) : "");
  // 수동값을 쓰는 중일 때의 큰 글씨: manualText가 있으면 그것, 켜고 끄는 값이면 켜짐·꺼짐, 숫자면 단위와 함께
  const showManual = manual => {
    if (spec.manualText && live) showNow(spec.manualText(live, manual));
    else if (typeof manual === "boolean") showNow(manual ? t("common.on") : t("common.off"));
    else showNow(String(manual), meta.unit);
  };

  const updateAdopt = valid => {
    if (!adopt) return;
    const current = valueOf("steering", key);
    const target = adoptTarget();
    const same = Number(current) === Number(target);
    adopt.setContent(spec.adoptLabel ? t("vehicle.adopt_named", {name: spec.adoptLabel()}) : t("vehicle.adopt"),
                     !live || !valid || !Number.isFinite(target) ? t("vehicle.adopt_when_valid")
                       : same ? t("vehicle.adopt_same") : `${current} → ${target}`);
    adopt.disabled = !live || !valid || same || !Number.isFinite(target);
  };

  // 학습값을 수동값으로: 무엇이 어떻게 바뀌는지와 그 쓰임(학습을 버리는 항목이면 그 주의)을 묻고 저장한다
  const adoptLearned = () => {
    if (!live) return;
    const current = valueOf("steering", key), target = adoptTarget();
    const auto = steering()[switchKey] === true;
    if (!confirmAction([t("vehicle.adopt_confirm", {label: meta.label}),
                        `${formatValue(current, meta)} → ${formatValue(target, meta)}`, "",
                        auto ? t("vehicle.role", {role: meta.learned_role}) : t("vehicle.adopt_manual_now"),
                        meta.caution || null])) return;
    editor.save(target);
  };

  // 다른 항목 때문에 이 값을 쓰지 않으면 지금 쓰는 값이 없고, 학습값·수동값 어느 줄도 사용 중이 아니다
  const showUnused = () => {
    setBadge(source, [t("params.unused"), "muted"]);
    showNow(DASH);
    learnedRow.classList.remove("in-use");
    manualRow.classList.remove("in-use");
  };

  function render() {
    const auto = steering()[switchKey] === true;
    const manual = valueOf("steering", key);
    const unused = conditions.update();
    role.textContent = auto && meta.learned_role ? t("vehicle.role", {role: meta.learned_role}) : "";
    role.hidden = !role.textContent;
    if (!live) {
      setBadge(source, auto ? [t("vehicle.no_state"), "bad"] : [t("vehicle.manual_value"), "muted"]);
      if (auto) showNow(DASH);
      else showManual(manual);
      learnedValue.textContent = DASH;
      learnedValue.className = "quantity-value";
      learnedDetail.textContent = t("vehicle.no_controlsd_state");
      learnedRow.classList.remove("in-use");
      manualRow.classList.toggle("in-use", !auto);
      if (unused) showUnused();
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
      setBadge(source, [spec.both ? t("vehicle.manual_times_learned") : t("vehicle.learned"), "good"]);
    } else {
      showManual(manual);
      setBadge(source, auto ? [learner.waiting(), "warn"] : [t("vehicle.manual_value"), "muted"]);
    }
    learnedRow.classList.toggle("in-use", inUse);
    manualRow.classList.toggle("in-use", !inUse || Boolean(spec.both));
    if (unused) showUnused();
    updateAdopt(valid);
  }

  card.append(...[
    el("header.card-head", {}, el("h3.card-title", {text: spec.title ? spec.title() : meta.label}), source),
    helpText(meta.description),
    el("div.quantity-now-row", {}, now, el("span.quantity-caption", {text: t("vehicle.now")})),
    learnedRow,
    manualRow,
    role,
    conditions.element,
    meta.caution ? note(meta.caution, "warn") : null,
    adopt,
    el("footer.param-foot", {}, effectsDetails(meta, typeof valueOf("steering", key) === "boolean")),
  ].filter(Boolean));
  render();
  const refresh = () => {
    editor.refresh();
    render();
  };
  // 다른 항목(조향 탭의 스위치 등)이 저장되면 값과 안내를 다시 그린다
  onParamsChanged(refresh);
  return {
    element: card,
    update(next) {
      live = next;
      render();
    },
    refresh,
  };
}

// 학습기 묶음 하나: 이름·학습기·설명과 출처 선택(자동 학습·수동, 그 스위치의 편집기), 상태 배지, 값 카드들, 한 줄 요약.
function learnerGroup(id, spec, keys) {
  const learner = LEARNERS[spec.learner];
  const cards = keys.map(key => quantityCard(key, spec.switch, learner));
  const statusRow = el("div.badge-row.group-status");
  const summary = el("p.group-summary");
  let live = null;
  const update = next => {
    live = next;
    const auto = steering()[spec.switch] === true;
    statusRow.replaceChildren(...badges([
      !live ? [t("vehicle.no_state"), "bad"]
        : !auto ? [t("vehicle.using_manual"), "muted"]
        : learner.inUse(live) ? [t("vehicle.using_learned"), "good"] : [learner.waiting(), "warn"],
      ...(live ? learner.status(live) : []),
    ]));
    summary.textContent = live ? learner.summary(live) : t("vehicle.no_learner_state");
    cards.forEach(card => card.update(live));
  };
  const source = paramEditor("steering", spec.switch, {onSaved: () => update(live)});
  return {
    element: section([sectionName(id), el("span", {text: spec.learner})],
                     [statusRow, el("div.card-grid", {}, cards.map(card => card.element)), summary],
                     {grid: false, description: metaFor("steering", spec.switch).description,
                      aside: el("div.section-aside", {}, source.control.element, source.status)}),
    update,
    refresh() {
      source.refresh();
      cards.forEach(card => card.refresh());
      update(live);
    },
  };
}

// ------------------------------------------------------------ 카메라 보정
// 보정 상태의 톤. 문구는 calibration.status.<상태>(배지), 포즈 보정기의 상태는 표에 calibration.pose.<상태>.
const CALIBRATION_TONE = {calibrated: "good", uncalibrated: "warn", invalid: "bad", recalibrating: "warn"};
const statusBadge = status => [codeText("calibration.status", status), CALIBRATION_TONE[status] || "muted"];
const sourceText = source => (source === "flow" ? t("calibration.flow") : t("calibration.pose_source"));

// 워프 보정(modeld calibration)과 그 출처. 출처 버튼은 display.json의 calibration_use_flow를 바꾼다(결합 중에는 서버가
// 막고 modeld도 해제를 기다린다). 표는 워프에 쓴 값, 광류 추정, 모델 포즈 보정기 자신의 추정과 둘의 차이(모델 pose의
// 치우침)다. 초기화는 두 보정을 다 지운다.
function calibrationCard() {
  let latest = null;
  let saving = false;
  const button = actionButton({icon: "↺", danger: true, onclick: () => reset()});
  button.setContent(t("calibration.reset"));  // 첫 상태가 오기 전(막혀 있다)
  const source = segmented(t("calibration.source"), [{value: true, label: t("calibration.use_flow")},
                                                      {value: false, label: t("calibration.use_pose")}], useFlow => choose(useFlow));
  const card = liveCard(t("calibration.card"), {control: source.element, actions: [button]});
  const how = helpText(t("calibration.how"), "card-note");  // 한 번 만들어 펼친 상태가 1초마다 접히지 않게

  const choose = async useFlow => {
    if (!latest || !latest.available || saving) return;
    const result = await runAction({
      busy: on => {
        saving = on;
        if (latest) update(latest);
      },
      call: () => api.post("/api/calibration/source", {use_flow: useFlow}),
      done: () => t("calibration.source_changed", {source: useFlow ? t("calibration.use_flow") : t("calibration.use_pose")}),
      failed: error => t("calibration.source_failed", {error}),
    });
    if (result) update(result);
  };

  const reset = async () => {
    const c = latest;
    const result = await runAction({
      confirm: [t("calibration.confirm"),
                c && c.available
                  ? t("calibration.confirm_now", {pitch: sgn(c.rpy_deg[1], 2), yaw: sgn(c.rpy_deg[2], 2), source: sourceText(c.source)})
                  : t("calibration.confirm_no_state"),
                "", t("calibration.confirm_effect"), t("calibration.confirm_after")],
      busy: on => {
        button.disabled = on;
      },
      call: () => api.post("/api/calibration/reset"),
      done: () => t("calibration.reset_requested"),
      failed: error => t("calibration.reset_failed", {error}),
    });
    if (result) update(result);
  };

  const update = c => {
    latest = c;
    const blocked = Boolean(c && c.engaged);
    source.show(c && c.available ? Boolean(c.use_flow) : null);
    source.setDisabled(saving || blocked || !(c && c.available));
    source.element.title = blocked ? t("calibration.blocked_source") : "";
    if (!c || !c.available) {
      card.update([[t("vehicle.no_state"), "bad"]], [note(t("calibration.no_state"))]);
    } else {
      const [r, p, y] = c.rpy_deg;
      const flow = c.flow, pose = c.pose, w = flow.window;
      const hasFlow = flow.estimate_blocks > 0;
      const estimate = flow.status === "estimate";
      const dp = pose.rpy_deg[1] - flow.pitch_deg, dy = pose.rpy_deg[2] - flow.yaw_deg;
      card.update([statusBadge(c.status),
                   c.source === "flow" ? [t("calibration.flow_applied"), "accent"] : [t("calibration.pose_applied"), "muted"],
                   c.switch_pending ? [t("calibration.switch_pending"), "warn"] : null,
                   c.ramping ? [t("calibration.ramping"), "accent"] : null,
                   flow.status === "collecting" ? [t("calibration.flow_collecting", {blocks: flow.estimate_blocks}), "warn"]
                     : estimate ? [t("calibration.flow_blocks", {blocks: flow.estimate_blocks}), "muted"] : null,
                   staleBadge(c)], [
        infoTable(["", "pitch (°)", "yaw (°)", ""], [
          [t("calibration.row_warp"), sgn(p, 2), sgn(y, 2), t("calibration.row_warp_note", {roll: sgn(r, 2), source: sourceText(c.source)})],
          [t("calibration.row_flow"), hasFlow ? sgn(flow.pitch_deg, 2) : DASH, hasFlow ? sgn(flow.yaw_deg, 2) : DASH,
           hasFlow ? t("calibration.row_flow_note", {blocks: flow.estimate_blocks, pitch_sd: num(flow.sd_deg[0], 2), yaw_sd: num(flow.sd_deg[1], 2)})
                   : flow.status === "off" ? t("calibration.flow_off") : t("calibration.collecting")],
          [t("calibration.row_pose"), sgn(pose.rpy_deg[1], 2), sgn(pose.rpy_deg[2], 2),
           t("calibration.row_pose_note", {blocks: pose.valid_blocks,
                                           status: c.source === "flow" || c.ramping ? t("calibration.paused") : codeText("calibration.pose", pose.status)})],
          [t("calibration.row_bias"), estimate ? sgn(dp, 2) : DASH, estimate ? [sgn(dy, 2), Math.abs(dy) > 0.2 ? "warn" : ""] : DASH,
           t("calibration.row_bias_note")],
        ]),
        c.use_flow && !c.flow_usable ? note(t("calibration.flow_not_usable"), "warn") : null,
        flow.windows > 0
          ? note(t("calibration.windows", {windows: flow.windows, rejected: flow.rejected_windows, tracks: w.tracks, inliers: w.inliers,
                                           rotation: num(w.rotation_deg, 3), parallax: num(w.parallax_px, 1),
                                           changes: flow.mount_changes})) : null,
        how,
      ]);
    }
    button.setContent(c && c.reset_pending ? t("calibration.reset_pending") : t("calibration.reset"),
                      blocked ? t("calibration.blocked_reset") : t("calibration.reset_note"));
    button.disabled = blocked || !(c && c.available);
  };
  // 읽지 못하면 옛 상태를 버린다: 출처 버튼도 막고, 고르는 동안의 다시 그리기가 옛 상태로 덮지 않게
  const fail = error => {
    latest = null;
    source.setDisabled(true);
    card.fail(error);
  };
  return {element: card.element, update, fail};
}

// ------------------------------------------------------------ 추이
// 서버가 내는 추이 행의 열(learner_monitor.py LEARNER_TREND_COLUMNS와 같은 순서, check_web_console.py가 대조). 첫 열이
// 시각(s)이다.
const TREND_COLUMNS = ["time_s", "steer_ratio", "angle_offset_average_deg", "angle_offset_deg", "roll_deg",
                       "bank_roll_deg", "lat_accel_factor_raw", "lat_accel_factor"];
// title·dashed는 문구 키. series는 [추이 행의 열 이름, 선 이름(console.css .trend .<이름>)]이고, levels는 기준선: ref는
// 점선(dashed가 무엇인지 말한다: 수동값 또는 학습의 사전값), low·high는 그 허용 폭이다. span은 y 범위의 최소 폭.
const TREND_CHARTS = [
  {title: "vehicle.trend_ratio", series: [["steer_ratio", "main"]], levels: m => ({ref: m.steer_ratio}),
   dashed: "vehicle.trend_dashed_manual", span: 0.2, digits: 2},
  {title: "vehicle.trend_offset", series: [["angle_offset_average_deg", "main"], ["angle_offset_deg", "thin"]],
   levels: m => ({ref: m.angle_offset_deg}), dashed: "vehicle.trend_dashed_manual", span: 0.2, digits: 2},
  {title: "vehicle.trend_roll", series: [["roll_deg", "main"], ["bank_roll_deg", "dash"]], levels: () => ({}),
   span: 0.5, digits: 2},
  {title: "vehicle.trend_torque", series: [["lat_accel_factor", "main"], ["lat_accel_factor_raw", "raw"]],
   levels: (m, s) => ({ref: s.prior_lat_accel_factor, low: s.prior_lat_accel_factor * 0.7, high: s.prior_lat_accel_factor * 1.3}),
   dashed: "vehicle.trend_dashed_prior", span: 0.2, digits: 2},
];

// 추이 카드 하나: 제목과 지금 값, 차트(ui.js sparkline), 그린 범위와 점선의 뜻. 한 번 만들고 1초마다 update로 그린다.
function trendCard(chart) {
  const title = t(chart.title);
  const series = chart.series.map(([column, name]) => [TREND_COLUMNS.indexOf(column), name]);
  const now = el("span.trend-now", {text: DASH});
  const range = el("span"), dashed = el("span");
  const plot = sparkline({height: 30, series: series.map(([, name]) => name), levels: ["ref", "low", "high"], label: title});
  const element = el("article.card.trend", {}, el("div.trend-head", {}, el("span", {text: title}), now), plot.element,
                     el("div.trend-range", {}, range, dashed));
  // 원시 torqued 배율의 0은 아직 계산하지 못한 값이다
  const value = (row, index, name) => (Number.isFinite(row[index]) && !(name === "raw" && row[index] === 0) ? row[index] : NaN);
  return {
    element,
    update(rows, windowS, manual, state) {
      const end = rows.length ? rows[rows.length - 1][0] : 0;
      const shown = rows.filter(row => row[0] >= end - windowS);
      const at = chart.levels(manual, state);
      const data = Object.fromEntries(series.map(([index, name]) => [name, shown.map(row => [row[0], value(row, index, name)])]));
      const y = fitRange([...Object.values(at), ...Object.values(data).flat().map(([, v]) => v)], {minSpan: chart.span, pad: 0.08});
      plot.draw({x: [end - windowS, end], y, data, at});
      const last = shown[shown.length - 1];
      now.textContent = last ? series.map(([index, name]) => num(value(last, index, name), chart.digits)).join(" / ") : DASH;
      range.textContent = `${num(y[0], chart.digits)} ~ ${num(y[1], chart.digits)}`;
      dashed.textContent = Number.isFinite(at.ref) ? t(chart.dashed, {value: num(at.ref, chart.digits)}) : t("vehicle.trend_window");
    },
  };
}

// ------------------------------------------------------------ 탭
export function createVehicleView() {
  const tab = params().vehicle_tab;
  const keysOf = name => Object.keys(steering()).filter(key => {
    const meta = metaFor("steering", key);
    return meta.tab === "vehicle" && meta.section === name;
  });
  const groups = tab.sections.filter(id => tab.learners[id]).map(id => {
    const spec = tab.learners[id];
    return learnerGroup(id, spec, keysOf(id).filter(key => key !== spec.switch));
  });
  const calibration = calibrationCard();
  const notice = el("div");
  const trends = TREND_CHARTS.map(trendCard);
  let rows = [];
  let windowS = 600;
  let trendLoaded = false;

  const update = data => {
    updateRuntime(data.runtime);
    calibration.update(data.calibration);
    if (!data.available) {
      notice.replaceChildren(el("p.group-note.warn", {text: t("vehicle.no_learners")}));
      groups.forEach(group => group.update(null));
      return;
    }
    notice.replaceChildren(data.stale
      ? el("p.group-note.warn", {text: t("vehicle.learners_stale", {seconds: Math.round(data.age_s)})}) : "");
    const live = {learner: data.state, localization: data.localization.available ? data.localization : null};
    groups.forEach(group => group.update(live));
    const row = data.trend_row;
    if (row && rows.length && row[0] < rows[rows.length - 1][0]) rows = [];
    if (row && (!rows.length || row[0] > rows[rows.length - 1][0])) {
      rows.push(row);
      while (rows.length && rows[0][0] < row[0] - windowS) rows.shift();
    }
    trends.forEach(trend => trend.update(rows, windowS, steering(), data.state));
  };

  // 읽지 못하면 위의 알림과 함께 값과 보정 카드도 "상태 없음"으로 돌린다(옛 값이 살아 있는 것처럼 보이지 않게)
  const fail = error => {
    notice.replaceChildren(el("p.group-note.warn", {text: t("vehicle.learners_failed", {error: error.message})}));
    groups.forEach(group => group.update(null));
    calibration.fail(error);
  };
  // 처음(그리고 탭을 다시 열 때) 한 번은 서버가 모아 둔 10분 추이부터 받는다
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
      if (isCurrent()) fail(error);
    }
  }, 1000);

  return {
    element: el("div.view",
      {},
      el("p.group-note", {text: tab.note}),
      notice,
      groups.map(group => group.element),
      section(t("vehicle.trends"), trends.map(trend => trend.element)),
      section(t("calibration.section"), [calibration.element]),
      el("p.file-path", {text: t("params.file", {path: params().paths.steering})})),
    start() {
      // 가려져 있던 동안의 추이는 서버에서 다시 받는다
      trendLoaded = false;
      groups.forEach(group => group.refresh());
      polling.start();
    },
    stop: polling.stop,
  };
}
