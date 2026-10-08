// 웹 콘솔 페이지: 머리 줄(런타임 상태, 다시 불러오기)과 탭(파라미터 그룹들과 차량 특성, BEV). 주소의 #탭 이름이
// 지금 탭이라 새로고침해도 그 탭에 머문다. 탭 화면은 처음 열 때 만들고, 보이는 탭만(페이지가 가려지지 않은
// 동안만) 폴링·스트림을 돌린다.
import {createBevView} from "./bev_view.js";
import {createPandaCard} from "./panda_card.js";
import {loadParams, params} from "./param_state.js";
import {createParamView} from "./param_view.js";
import {api, el, poller, toast} from "./ui.js";
import {createVehicleView} from "./vehicle_view.js";

const STATUS_INTERVAL_MS = 2000;
// 파라미터 탭 아래에 붙는 카드
const PARAM_EXTRAS = {display: () => [createPandaCard()]};

const main = document.getElementById("main");
const nav = document.getElementById("tabs");
let tabs = [];
const views = new Map();
let current = null;

/* 탭 순서: 조향, 차량 특성(조향 값 중 차의 성질), 나머지 파라미터 그룹, BEV. keep: 다시 불러와도 새로 만들지
 * 않는다(파라미터를 읽지 않는 탭). */
function tabSpecs() {
  const groups = Object.entries(params().groups).map(([id, group]) => ({
    id, label: group.label, create: () => createParamView(id, {extras: (PARAM_EXTRAS[id] || (() => []))()}),
  }));
  const vehicle = {id: "vehicle", label: params().vehicle_tab.label, create: createVehicleView};
  return [groups[0], vehicle, ...groups.slice(1), {id: "bev", label: "BEV", create: createBevView, keep: true}];
}

function view(tab) {
  if (!views.has(tab.id)) views.set(tab.id, tab.create());
  return views.get(tab.id);
}

function show(id) {
  const tab = tabs.find(item => item.id === id) || tabs[0];
  if (current && current.id !== tab.id) view(current).stop();
  const changed = !current || current.id !== tab.id;
  current = tab;
  const next = view(tab);
  main.replaceChildren(next.element);
  if (!document.hidden) next.start();
  for (const link of nav.children) {
    if (link.dataset.tab === tab.id) link.setAttribute("aria-current", "page");
    else link.removeAttribute("aria-current");
  }
  nav.querySelector(`[data-tab="${tab.id}"]`)?.scrollIntoView({block: "nearest", inline: "nearest"});
  if (window.location.hash !== `#${tab.id}`) window.history.replaceState(null, "", `#${tab.id}`);
  document.title = `${tab.label} · edgepilot`;
  if (changed) window.scrollTo(0, 0);
}

function route() {
  const id = decodeURIComponent(window.location.hash.slice(1));
  show(id);
}

/* 파라미터를 (다시) 읽고 탭을 만든다. 다시 읽을 때는 파라미터를 쓰는 화면을 버리고 새로 만든다. */
async function load() {
  await loadParams();
  if (current) view(current).stop();
  for (const tab of tabs) if (!tab.keep) views.delete(tab.id);
  tabs = tabSpecs();
  nav.replaceChildren(...tabs.map(tab => el("a.tab", {href: `#${tab.id}`, dataset: {tab: tab.id}, text: tab.label})));
  current = null;
  route();
}

async function start() {
  try {
    await load();
  } catch (error) {
    main.replaceChildren(el("div.empty",
      {},
      el("p", {text: `파라미터를 읽지 못했습니다: ${error.message}`}),
      el("button.button", {type: "button", text: "다시 시도", onclick: start})));
  }
}

// ------------------------------------------------------------ 머리 줄: 런타임 상태
const runtime = document.getElementById("runtime");
const runtimeText = runtime.querySelector(".runtime-text");
const runtimePanel = document.getElementById("runtime-panel");

/* 머리 줄의 상태: 매니저가 내는 프로세스 실행 여부(누르면 목록)와 백라이트. failure는 웹 콘솔에 닿지 못했을 때. */
function showRuntime(status, failure = "") {
  if (failure) {
    runtime.dataset.state = "bad";
    runtimeText.textContent = "웹 콘솔 연결 끊김";
    runtimePanel.replaceChildren(el("p.runtime-note", {text: failure}));
    return;
  }
  const processes = status.processes;
  const running = processes.filter(process => process.running).length;
  runtime.dataset.state = !status.available ? "bad" : running === processes.length ? "good" : "warn";
  runtimeText.textContent = !status.available
    ? (processes.length ? `매니저 응답 없음 · ${Math.round(status.age_s)}초` : "매니저 없음")
    : `프로세스 ${running}/${processes.length}`;
  const backlight = status.backlight;
  runtimePanel.replaceChildren(
    processes.length
      ? el("ul.process-list", {}, processes.map(process => el(`li${process.running ? ".running" : ""}`,
        {}, el("span.dot"), el("code", {text: process.name}), el("span", {text: process.running ? "실행 중" : "멈춤"}))))
      : el("p.runtime-note", {text: "매니저가 프로세스 상태를 내지 않습니다."}),
    el("p.runtime-note", {
      text: backlight.error ? `백라이트 오류: ${backlight.error}`
        : !backlight.available ? "백라이트 제어 없음"
        : `백라이트 ${backlight.enabled ? `켜짐 · ${backlight.brightness_percent}%` : "꺼짐"}`,
    }));
}

const statusPolling = poller(async isCurrent => {
  try {
    const status = await api.get("/api/status");
    if (isCurrent()) showRuntime(status);
  } catch (error) {
    if (isCurrent()) showRuntime(null, error.message);
  }
}, STATUS_INTERVAL_MS);

document.addEventListener("click", event => {
  if (runtime.open && !runtime.contains(event.target)) runtime.open = false;
});
document.addEventListener("visibilitychange", () => {
  const shown = current ? view(current) : null;
  if (document.hidden) {
    statusPolling.stop();
    shown?.stop();
  } else {
    statusPolling.start();
    shown?.start();
  }
});
document.getElementById("reload").addEventListener("click", async () => {
  try {
    await load();
    toast("최신 값을 불러왔습니다");
  } catch (error) {
    toast(`읽기 실패: ${error.message}`, {failed: true});
  }
});
window.addEventListener("hashchange", () => {
  if (tabs.length) route();
});

statusPolling.start();
start();
