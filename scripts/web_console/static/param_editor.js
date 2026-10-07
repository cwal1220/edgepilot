// 파라미터 항목 하나를 고치는 부품: 컨트롤(켜고 끄기, −/+, 슬라이더), 저장(그동안 막고 결과를 상태 줄에), 바뀜 표시와
// 기본값으로 되돌리기, 그리고 이것들로 짠 기본 항목 카드. 파라미터 탭과 차량 특성 탭이 같이 쓴다.
import {clampToMeta, defaultOf, isModified, metaFor, params, saveParam, valueOf} from "./param_state.js";
import {clock, el, toast} from "./ui.js";

export function formatValue(value, meta) {
  if (typeof value === "boolean") return value ? "켜짐" : "꺼짐";
  return `${value}${meta.unit ? ` ${meta.unit}` : ""}`;
}

/* 항목 하나의 편집기. card가 있으면 저장하는 동안 그 카드를 흐리게 하고 끝나면 잠깐 테를 칠한다. onSaved는 저장이
 * 끝날 때마다(실패해도) 불린다. refresh()는 스냅샷 값으로 다시 그린다. */
export function paramEditor(group, key, {card = null, onSaved = () => {}} = {}) {
  const meta = metaFor(group, key);
  const notify = params().groups[group].notify;
  const status = el("span.param-status");
  const modifiedBadge = el("span.badge.warn", {text: "바뀜", title: "기본값과 다릅니다"});
  const resetButton = el("button.link-button", {type: "button", text: "↺ 기본값으로"});
  let control = null;

  const refresh = () => {
    control.show(valueOf(group, key));
    const modified = isModified(group, key);
    modifiedBadge.hidden = !modified;
    resetButton.hidden = !modified;
  };

  const save = async next => {
    card?.classList.add("busy");
    control.setDisabled(true);
    status.className = "param-status";
    status.textContent = "적용 중";
    try {
      const {notified} = await saveParam(group, key, next);
      const waiting = notify && !notified.length;
      status.className = `param-status ${waiting ? "warn" : "ok"}`;
      status.textContent = waiting ? `저장됨 · ${notify} 미실행` : `적용됨 ${clock()}`;
      if (card) {
        card.classList.add("saved");
        window.setTimeout(() => card.classList.remove("saved"), 900);
      }
    } catch (error) {
      status.className = "param-status fail";
      status.textContent = "적용 실패";
      toast(`${meta.label}: ${error.message}`, {failed: true});
    } finally {
      card?.classList.remove("busy");
      control.setDisabled(false);
      refresh();
      onSaved();
    }
  };

  const value = valueOf(group, key);
  control = typeof value === "boolean" ? switchControl(meta, save)
    : meta.control === "slider" ? sliderControl(meta, save) : stepperControl(meta, save);
  resetButton.addEventListener("click", () => save(defaultOf(group, key)));
  refresh();
  return {meta, control, status, modifiedBadge, resetButton, refresh, save};
}

/* 접힌 "바꾸면": 값을 줄이거나 끄면, 키우거나 켜면 어떻게 되는지. lead가 있으면 그 앞에 둔다(설명 등). */
export function effectsDetails(meta, boolean, {summary = "바꾸면", lead = null} = {}) {
  return el("details.param-effects",
    {},
    el("summary", {text: summary}),
    lead,
    el("div.effects",
      {},
      el("div.effect.down", {}, el("b", {text: boolean ? "끄면" : "값을 줄이면"}), meta.decrease),
      el("div.effect.up", {}, el("b", {text: boolean ? "켜면" : "값을 키우면"}), meta.increase)));
}

/* 항목 카드: 이름·키·기본값, 설명(눌러 펼침), 컨트롤, 바꾸면, 되돌리기, 상태. 파라미터 탭의 검색이 쓰는 search와
 * isModified, 다시 그리는 refresh를 단다. */
export function paramCard(group, key) {
  const card = el("article.card.param", {dataset: {key}});
  const editor = paramEditor(group, key, {card});
  const {meta} = editor;
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
      editor.modifiedBadge,
      meta.unit ? el("span.param-unit", {text: meta.unit}) : null),
    description,
    editor.control.element,
    el("footer.param-foot",
      {},
      effectsDetails(meta, typeof valueOf(group, key) === "boolean"),
      editor.resetButton,
      editor.status));
  card.search = `${meta.label} ${key} ${meta.description}`.toLowerCase();
  card.isModified = () => isModified(group, key);
  card.refresh = editor.refresh;
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
