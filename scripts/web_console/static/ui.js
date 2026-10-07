// 웹 콘솔 공용 도구: DOM 만들기, 카드·배지·표, 토스트, 확인 창, API 호출, 주기 폴링, 숫자 표기.

/* el("div.card.live", {title: "..."}, child, "text", ...): 태그 이름 뒤 .클래스, 속성(on<event>는 리스너),
 * 자식(노드·문자열·배열, null·false는 건너뜀). */
export function el(spec, attributes = {}, ...children) {
  const [tag, ...classes] = spec.split(".");
  const node = document.createElement(tag || "div");
  if (classes.length) node.className = classes.join(" ");
  for (const [name, value] of Object.entries(attributes || {})) {
    if (value === undefined || value === null || value === false) continue;
    if (name.startsWith("on") && typeof value === "function") node.addEventListener(name.slice(2), value);
    else if (name === "dataset") Object.assign(node.dataset, value);
    else if (name === "text") node.textContent = value;
    else if (name in node && typeof node[name] !== "function" && name !== "list") node[name] = value;
    else node.setAttribute(name, value === true ? "" : String(value));
  }
  node.append(...children.flat(Infinity).filter(child => child !== null && child !== undefined && child !== false));
  return node;
}

export function svg(tag, attributes = {}) {
  const node = document.createElementNS("http://www.w3.org/2000/svg", tag);
  for (const [name, value] of Object.entries(attributes)) node.setAttribute(name, String(value));
  return node;
}

// ------------------------------------------------------------ 숫자 표기
export function num(value, digits) {
  return Number.isFinite(value) ? value.toFixed(digits) : "–";
}

export function sgn(value, digits) {
  if (!Number.isFinite(value)) return "–";
  return `${value < 0 ? "−" : "+"}${Math.abs(value).toFixed(digits)}`;
}

export function deg(rad) {
  return rad * 180 / Math.PI;
}

export function ms(seconds) {
  return `${Math.round(seconds * 1000)} ms`;
}

export function clock(date = new Date()) {
  return date.toLocaleTimeString("ko-KR", {hour12: false});
}

// ------------------------------------------------------------ 배지·표·카드
// [텍스트, 톤] 목록. 톤: good, warn, bad, accent, muted
function badges(list) {
  return list.filter(Boolean).map(([text, tone]) => el(`span.badge.${tone || "muted"}`, {text}));
}

/* 머리 줄과 행들. 셀은 문자열이나 [문자열, 톤]. 둘째 열은 값(굵게), 넷째 열은 설명(흐리게). */
export function infoTable(head, rows) {
  const cell = (tag, value, kind) => {
    const [text, tone] = Array.isArray(value) ? value : [value, ""];
    return el(`${tag}${kind ? `.${kind}` : ""}${tone ? `.${tone}` : ""}`, {text});
  };
  return el("table.info-table",
    {},
    el("thead", {}, el("tr", {}, head.map(text => el("th", {text})))),
    el("tbody", {}, rows.map(row => el("tr", {}, row.map((value, index) =>
      cell("td", value, index === 1 ? "value" : index === 3 ? "note" : ""))))));
}

/* 상태 카드: 제목·배지·(머리의 컨트롤)·본문·버튼 줄. 머리와 버튼은 한 번 만들고 본문과 배지만 바꾼다
 * (누르는 중인 스위치를 매초 갈아엎지 않게). */
export function liveCard(title, {control = null, actions = []} = {}) {
  const badgeRow = el("span.badge-row");
  const body = el("div.card-body");
  const element = el("article.card",
    {},
    el("header.card-head", {}, el("h3.card-title", {text: title}), badgeRow, control),
    body,
    actions.length ? el("div.action-row", {}, actions) : null);
  return {
    element,
    update(badgeList, children) {
      badgeRow.replaceChildren(...badges(badgeList));
      body.replaceChildren(...children.flat().filter(Boolean));
    },
  };
}

/* 카드 아래쪽의 큰 버튼: 아이콘, 제목, 작은 설명 줄들. danger면 붉은 테. */
export function actionButton({icon, danger = false, onclick}) {
  const button = el(`button.action${danger ? ".danger" : ""}`, {type: "button", onclick});
  button.setContent = (title, ...notes) => {
    button.replaceChildren(el("span.action-icon", {text: icon}),
      el("span.action-text", {}, title, notes.filter(Boolean).map(note => el("small", {text: note}))));
  };
  return button;
}

export function note(text, tone = "") {
  return el(`p.card-note${tone ? `.${tone}` : ""}`, {text});
}

export function section(title, children, {count = null, grid = true} = {}) {
  return el("section.section",
    {},
    el("h2.section-title", {}, title, count === null ? null : el("span", {text: `${count}개`})),
    el(grid ? "div.card-grid" : "div", {}, children));
}

// ------------------------------------------------------------ 토스트·확인
let toastRoot = null;

export function toast(text, {failed = false} = {}) {
  toastRoot = toastRoot || document.getElementById("toasts");
  const item = el(`div.toast${failed ? ".failed" : ""}`, {role: failed ? "alert" : "status", text});
  toastRoot.append(item);
  window.setTimeout(() => item.classList.add("leaving"), failed ? 5000 : 1800);
  window.setTimeout(() => item.remove(), failed ? 5400 : 2200);
}

// 줄 목록을 확인 창으로. 첫 줄이 질문이다.
export function confirmAction(lines) {
  return window.confirm(lines.filter(line => line !== null && line !== undefined).join("\n"));
}

// ------------------------------------------------------------ API
async function request(method, url, body) {
  const response = await fetch(url, {
    method,
    cache: "no-store",
    headers: body === undefined ? {} : {"Content-Type": "application/json"},
    body: body === undefined ? undefined : JSON.stringify(body),
  });
  const data = await response.json().catch(() => ({}));
  if (!response.ok) throw new Error(data.detail || response.statusText || `HTTP ${response.status}`);
  return data;
}

export const api = {
  get: url => request("GET", url),
  post: (url, body = {}) => request("POST", url, body),
  patch: (url, body) => request("PATCH", url, body),
};

/* fn을 interval_ms마다 부른다(겹치지 않게). start()는 곧바로 한 번 부르고, stop() 뒤에는 늦게 끝난 호출도
 * 버린다. */
export function poller(fn, intervalMs) {
  let timer = null;
  let generation = 0;
  const tick = async current => {
    try {
      await fn(() => current === generation);
    } finally {
      if (current === generation) timer = window.setTimeout(() => tick(current), intervalMs);
    }
  };
  return {
    start() {
      if (timer !== null) return;
      generation += 1;
      timer = 0;
      tick(generation);
    },
    stop() {
      generation += 1;
      if (timer) window.clearTimeout(timer);
      timer = null;
    },
  };
}
