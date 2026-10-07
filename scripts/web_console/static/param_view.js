// 파라미터 탭 하나(조향, 주행 제한, 비전 크루즈, 주행 기록, 기기 설정). 섹션별 카드, 검색, 바꾼 항목만 보기,
// 기본값 표시와 되돌리기. 값은 바꾸는 즉시 저장되고, 그 그룹을 읽는 프로세스가 다시 읽는다.
import {clampToMeta, defaultOf, isModified, metaFor, params, saveParam, valueOf} from "./param_state.js";
import {clock, el, note, section, toast} from "./ui.js";

const OTHER_SECTION = "기타";

/* 학습 스위치가 켜졌을 때 조향 수동값의 역할. 무시되는 값은 흐리게, 사전값으로만 쓰이는 값은 표시한다.
 * torqued 사전값 둘은 스위치와 무관하게 캐시 키라 바꾸면 학습이 처음부터 다시 시작된다. */
function learnerNotes(key, steering) {
  const vehicleOn = steering.use_live_vehicle_params === true;
  const torqueOn = steering.use_live_torque_params === true;
  const notes = [];
  if (vehicleOn && (key === "angle_offset_deg" || key === "live_bank_compensation"))
    notes.push(["ignored", "paramsd 학습값 사용 중 · 이 값은 무시됩니다. 실시간 학습 탭에서 끄면 다시 쓰입니다."]);
  if (vehicleOn && key === "steer_ratio")
    notes.push(["prior", "paramsd 학습값 사용 중 · 학습 조향비의 출발점과 유효 범위(0.5~2배)로만 쓰입니다. 저장된 학습값이 있으면 그게 우선입니다."]);
  if (vehicleOn && key === "tire_stiffness_factor")
    notes.push(["prior", "paramsd 학습값 사용 중 · 학습 강성 배율이 이 값에 곱해집니다."]);
  if (torqueOn && key === "torque_lat_accel_offset")
    notes.push(["ignored", "torqued 학습값 사용 중 · 이 값은 무시되고 학습 절편을 씁니다. 실시간 학습 탭에서 끄면 다시 쓰입니다."]);
  if (torqueOn && (key === "torque_lat_accel_factor" || key === "torque_friction"))
    notes.push(["prior", "torqued 학습값 사용 중 · 사전값과 허용 폭(배율 ±30%, 마찰 ±50%)으로만 쓰입니다."]);
  if (key === "torque_lat_accel_factor" || key === "torque_friction")
    notes.push(["reset", "바꾸면 controlsd 다음 시작 때 torqued 학습이 처음부터 다시 시작됩니다."]);
  if (steering.use_live_delay === true && key === "steer_actuator_delay")
    notes.push(["prior", "lagd 지연 사용 중 · 추정이 확정되면 경로 지연에는 추정값을 쓰고, 이 값은 확정 전 대체값과 토크 컨트롤러·torqued에 쓰입니다."]);
  return notes;
}

function formatValue(value, meta) {
  if (typeof value === "boolean") return value ? "켜짐" : "꺼짐";
  return `${value}${meta.unit ? ` ${meta.unit}` : ""}`;
}

/* 항목 카드 하나. 저장하는 동안 컨트롤을 막고, 저장 뒤 값·바뀜 표시·되돌리기 버튼을 그 자리에서 고친다.
 * refresh()는 스냅샷에서 다시 그린다(다른 탭에서 바뀐 값, 학습 스위치에 따른 안내). */
function paramCard(group, key) {
  const meta = metaFor(group, key);
  const value = valueOf(group, key);
  const notify = params().groups[group].notify;
  const status = el("span.param-status");
  const modifiedBadge = el("span.badge.warn", {text: "바뀜", title: "기본값과 다릅니다"});
  const resetButton = el("button.link-button", {type: "button", text: "↺ 기본값으로"});
  const notes = el("div.param-notes");
  const card = el("article.card.param", {dataset: {key}});
  let control = null;

  const refresh = () => {
    control.show(valueOf(group, key));
    const modified = isModified(group, key);
    modifiedBadge.hidden = !modified;
    resetButton.hidden = !modified;
    const list = group === "steering" ? learnerNotes(key, params().params.steering) : [];
    notes.replaceChildren(...list.map(([tone, text]) => note(text, tone)));
    card.classList.toggle("ignored", list.some(([tone]) => tone === "ignored"));
  };

  const save = async next => {
    card.classList.add("busy");
    control.setDisabled(true);
    status.className = "param-status";
    status.textContent = "적용 중";
    try {
      const {notified} = await saveParam(group, key, next);
      const waiting = notify && !notified.length;
      status.className = `param-status ${waiting ? "warn" : "ok"}`;
      status.textContent = waiting ? `저장됨 · ${notify} 미실행` : `적용됨 ${clock()}`;
      card.classList.add("saved");
      window.setTimeout(() => card.classList.remove("saved"), 900);
    } catch (error) {
      status.className = "param-status fail";
      status.textContent = "적용 실패";
      toast(`${meta.label}: ${error.message}`, {failed: true});
    } finally {
      card.classList.remove("busy");
      control.setDisabled(false);
      refresh();
    }
  };

  control = typeof value === "boolean" ? switchControl(meta, save)
    : meta.control === "slider" ? sliderControl(meta, save) : stepperControl(meta, save);
  resetButton.addEventListener("click", () => save(defaultOf(group, key)));

  const fallback = defaultOf(group, key);
  const description = el("p.param-description.clamped", {text: meta.description});
  description.addEventListener("click", () => description.classList.toggle("clamped"));
  card.append(
    el("header.param-head",
      {},
      el("div.param-name",
        {},
        el("h3.card-title", {text: meta.label}),
        el("div.param-key",
          {},
          el("code", {text: key}),
          fallback === undefined ? null : el("span", {text: ` · 기본 ${formatValue(fallback, meta)}`}))),
      modifiedBadge,
      meta.unit ? el("span.param-unit", {text: meta.unit}) : null),
    description,
    notes,
    control.element,
    el("footer.param-foot",
      {},
      el("details.param-effects",
        {},
        el("summary", {text: "바꾸면"}),
        el("div.effects",
          {},
          el("div.effect.down", {}, el("b", {text: typeof value === "boolean" ? "끄면" : "값을 줄이면"}), meta.decrease),
          el("div.effect.up", {}, el("b", {text: typeof value === "boolean" ? "켜면" : "값을 키우면"}), meta.increase))),
      resetButton,
      status));
  refresh();
  card.search = `${meta.label} ${key} ${meta.description}`.toLowerCase();
  card.isModified = () => isModified(group, key);
  card.refresh = refresh;
  return card;
}

// ------------------------------------------------------------ 컨트롤: {element, show(value), setDisabled(bool)}
function switchControl(meta, save) {
  const text = el("span");
  const button = el("button.switch-control",
    {type: "button", role: "switch", "aria-label": meta.label, onclick: () => save(button.getAttribute("aria-checked") !== "true")},
    text,
    el("span.switch"));
  return {
    element: button,
    show(value) {
      button.setAttribute("aria-checked", String(value));
      text.textContent = value ? "켜짐" : "꺼짐";
    },
    setDisabled(disabled) {
      button.disabled = disabled;
    },
  };
}

function stepperControl(meta, save) {
  const unit = meta.unit ? ` ${meta.unit}` : "";
  const input = el("input.stepper-value",
    {type: "number", inputMode: "decimal", step: String(meta.step), min: String(meta.min), max: String(meta.max),
     "aria-label": `${meta.label} 값`});
  let shown = null;
  const commit = next => {
    if (!Number.isFinite(next)) {
      toast("숫자를 입력하세요.", {failed: true});
      input.value = String(shown);
      return;
    }
    const value = clampToMeta(next, meta);
    input.value = String(value);
    if (value !== shown) save(value);
  };
  const stepButton = (direction, label) => el("button.stepper-button",
    {type: "button", text: label, title: `${meta.step}${unit} ${direction < 0 ? "감소" : "증가"}`,
     "aria-label": `${meta.step}${unit} ${direction < 0 ? "감소" : "증가"}`,
     onclick: () => commit(Number(input.value) + direction * meta.step)});
  const minus = stepButton(-1, "−");
  const plus = stepButton(1, "+");
  input.addEventListener("keydown", event => {
    if (event.key === "Enter") input.blur();
  });
  input.addEventListener("change", () => commit(Number(input.value)));
  return {
    element: el("div.stepper", {}, minus, el("label.stepper-field", {}, input, el("span.stepper-range", {text: `${meta.min} ~ ${meta.max}${unit}`})), plus),
    show(value) {
      shown = value;
      input.value = String(value);
    },
    setDisabled(disabled) {
      for (const control of [minus, input, plus]) control.disabled = disabled;
    },
  };
}

function sliderControl(meta, save) {
  const input = el("input.slider", {type: "range", min: String(meta.min), max: String(meta.max), step: String(meta.step),
                                   "aria-label": `${meta.label} 값`});
  const output = el("output.slider-value");
  let shown = null;
  input.addEventListener("input", () => {
    output.textContent = `${input.value}${meta.unit}`;
  });
  input.addEventListener("change", () => {
    const value = clampToMeta(Number(input.value), meta);
    if (value !== shown) save(value);
  });
  return {
    element: el("div.slider-control", {}, input, output),
    show(value) {
      shown = value;
      input.value = String(value);
      output.textContent = `${value}${meta.unit}`;
    },
    setDisabled(disabled) {
      input.disabled = disabled;
    },
  };
}

// ------------------------------------------------------------ 탭
/* extras: 파라미터 섹션 뒤에 붙일 것들({element, start, stop}, 예: 기기 설정의 Panda 펌웨어 카드). */
export function createParamView(group, {extras = []} = {}) {
  const spec = params().groups[group];
  const document_ = params().params[group];
  const bySection = new Map(spec.sections.map(name => [name, []]));
  for (const key of Object.keys(document_)) {
    const meta = metaFor(group, key);
    if (meta.hidden) continue;
    const name = bySection.has(meta.section) ? meta.section : OTHER_SECTION;
    if (!bySection.has(name)) bySection.set(name, []);
    bySection.get(name).push(paramCard(group, key));
  }
  const sections = [...bySection].filter(([, cards]) => cards.length).map(([name, cards]) => ({
    cards, element: section(name, cards, {count: cards.length}),
  }));
  const total = sections.reduce((sum, {cards}) => sum + cards.length, 0);

  const count = el("span.toolbar-count");
  const search = el("input.search", {type: "search", placeholder: "항목 검색", "aria-label": "항목 검색"});
  const modifiedOnly = el("button.chip", {type: "button", "aria-pressed": "false", text: "바꾼 항목만"});
  const empty = el("div.empty", {text: "맞는 항목이 없습니다.", hidden: true});
  const filter = () => {
    const words = search.value.trim().toLowerCase().split(/\s+/).filter(Boolean);
    const onlyModified = modifiedOnly.getAttribute("aria-pressed") === "true";
    let shown = 0;
    for (const {cards, element} of sections) {
      let visible = 0;
      for (const card of cards) {
        const match = words.every(word => card.search.includes(word)) && (!onlyModified || card.isModified());
        card.hidden = !match;
        visible += match ? 1 : 0;
      }
      element.hidden = visible === 0;
      shown += visible;
    }
    count.textContent = shown === total ? `${total}개 항목` : `${shown} / ${total}개 항목`;
    empty.hidden = shown > 0;
  };
  search.addEventListener("input", filter);
  modifiedOnly.addEventListener("click", () => {
    modifiedOnly.setAttribute("aria-pressed", String(modifiedOnly.getAttribute("aria-pressed") !== "true"));
    filter();
  });
  filter();

  const element = el("div.view",
    {},
    spec.note ? el("p.group-note", {text: spec.note}) : null,
    el("div.toolbar", {}, search, modifiedOnly, count),
    sections.map(({element: node}) => node),
    empty,
    extras.map(extra => extra.element),
    el("p.file-path", {text: `파일: ${params().paths[group]}`}));
  return {
    element,
    start() {
      for (const {cards} of sections) cards.forEach(card => card.refresh());
      filter();
      extras.forEach(extra => extra.start());
    },
    stop: () => extras.forEach(extra => extra.stop()),
  };
}
