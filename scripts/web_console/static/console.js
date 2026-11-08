// 웹 콘솔 페이지: 머리 줄(런타임 상태, 언어, 다시 불러오기)과 탭(파라미터 그룹들과 차량 특성, BEV). 주소의 #탭
// 이름이 지금 탭이라 새로고침해도 그 탭에 머문다. 탭 화면은 처음 열 때 만들고, 보이는 탭만(페이지가 가려지지 않은
// 동안만) 폴링·스트림을 돌린다.
import {createBevView} from "./bev_view.js";
import {createDrivingModelCard} from "./driving_model_card.js";
import {LANGUAGES, lang, setLanguage, t} from "./i18n.js";
import {createLateralNnCard} from "./lateral_nn_card.js";
import {createPandaCard} from "./panda_card.js";
import {loadParams, params} from "./param_state.js";
import {createParamView} from "./param_view.js";
import {createRecordingsCard} from "./recordings_card.js";
import {el, pollJson, toast} from "./ui.js";
import {createVehicleView} from "./vehicle_view.js";

const STATUS_INTERVAL_MS = 2000;
// 파라미터 탭에 붙는 카드
const PARAM_EXTRAS = {steering: () => [createDrivingModelCard(), createLateralNnCard()],
                      recording: () => [createRecordingsCard()],
                      display: () => [createPandaCard()]};

const main = document.getElementById("main");
const nav = document.getElementById("tabs");

// index.html의 글자: data-i18n은 내용, data-i18n-title·data-i18n-aria-label은 그 속성의 문구 키
for (const node of document.querySelectorAll("[data-i18n]")) node.textContent = t(node.dataset.i18n);
for (const node of document.querySelectorAll("[data-i18n-title]")) node.title = t(node.dataset.i18nTitle);
for (const node of document.querySelectorAll("[data-i18n-aria-label]")) node.setAttribute("aria-label", t(node.dataset.i18nAriaLabel));

// 언어 버튼: 다른 언어의 코드를 보이고, 누르면 그 언어로 다시 연다
const other = Object.keys(LANGUAGES).find(code => code !== lang);
const languageButton = document.getElementById("language");
languageButton.textContent = other.toUpperCase();
languageButton.title = LANGUAGES[other];
languageButton.setAttribute("aria-label", LANGUAGES[other]);
languageButton.lang = other;
languageButton.addEventListener("click", () => setLanguage(other));
let tabs = [];
const views = new Map();
let current = null;

// 탭 순서: 조향, 차량 특성(조향 값 중 차의 성질), 나머지 파라미터 그룹, BEV. keep: 다시 불러와도 새로 만들지
// 않는다(파라미터를 읽지 않는 탭).
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

// 파라미터를 (다시) 읽고 탭을 만든다. 다시 읽을 때는 파라미터를 쓰는 화면을 버리고 새로 만든다.
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
      el("p", {text: t("app.load_failed", {error: error.message})}),
      el("button.button", {type: "button", text: t("app.retry"), onclick: start})));
  }
}

// ------------------------------------------------------------ 머리 줄: 런타임 상태
const runtime = document.getElementById("runtime");
const runtimeText = runtime.querySelector(".runtime-text");
const runtimePanel = document.getElementById("runtime-panel");

// 실행 파일에 새긴 빌드 버전(process_status.py). 대개 모두 같아서 가장 많은 하나를 목록 아래에 쓰고, 칸에는 다를
// 때만 쓴다: 다른 빌드가 돌고 있거나, 설치된 실행 파일이 바뀌었는데 아직 다시 뜨지 않았을 때.
function commonBuild(processes) {
  const counts = new Map();
  for (const process of processes) {
    if (process.running && process.version) counts.set(process.version, (counts.get(process.version) || 0) + 1);
  }
  const common = [...counts].sort((a, b) => b[1] - a[1])[0];
  return {common: common ? common[0] : null, mixed: counts.size > 1};
}

function versionNote(process, common) {
  if (process.running && process.installed_version && process.installed_version !== process.version)
    return t("app.installed_differs", {version: process.installed_version});
  return process.running && process.version && process.version !== common ? process.version : "";
}

// 머리 줄의 상태: 매니저가 내는 프로세스 실행 여부(누르면 목록)와 백라이트. failure는 웹 콘솔에 닿지 못했을 때.
// 빌드 버전은 목록을 펼쳤을 때만 받는다(서버가 /proc 전체를 훑는다).
function showRuntime(status, failure = "") {
  if (failure) {
    runtime.dataset.state = "bad";
    runtimeText.textContent = t("app.console_offline");
    runtimePanel.replaceChildren(el("p.runtime-note", {text: failure}));
    return;
  }
  const processes = status.processes;
  const running = processes.filter(process => process.running).length;
  runtime.dataset.state = status.stale ? "bad" : running === processes.length ? "good" : "warn";
  runtimeText.textContent = !status.available ? t("app.manager_missing")
    : status.stale ? t("app.manager_stale", {seconds: Math.round(status.age_s)})
    : t("app.processes", {running, total: processes.length});
  const backlight = status.backlight;
  const build = commonBuild(processes);
  // replaceChildren은 null도 글자로 넣으므로 빈 줄은 거른다
  runtimePanel.replaceChildren(...[
    processes.length
      ? el("ul.process-list", {}, processes.map(process => {
        const note = versionNote(process, build.common);
        return el(`li${process.running ? ".running" : ""}`, {}, el("span.dot"), el("code", {text: process.name}),
          el("span", {text: process.running ? t("app.running") : t("app.stopped")}),
          note && el("span.process-version", {text: note}));
      }))
      : el("p.runtime-note", {text: t("app.no_process_state")}),
    status.stale && processes.length ? el("p.runtime-note", {text: t("app.last_known")}) : null,
    build.common && el("p.runtime-note", {text: t(build.mixed ? "app.build_mixed" : "app.build", {version: build.common})}),
    el("p.runtime-note", {
      text: !backlight.available ? t("app.backlight_missing")
        : backlight.enabled ? t("app.backlight_on", {percent: backlight.brightness_percent}) : t("app.backlight_off"),
    }),
  ].filter(Boolean));
}

const statusPolling = pollJson(() => (runtime.open ? "/api/status?versions=1" : "/api/status"), STATUS_INTERVAL_MS,
                               status => showRuntime(status), error => showRuntime(null, error.message));

document.addEventListener("click", event => {
  if (runtime.open && !runtime.contains(event.target)) runtime.open = false;
});
// 목록을 펼치면 버전까지 곧바로 다시 읽는다
runtime.addEventListener("toggle", () => {
  if (runtime.open && !document.hidden) {
    statusPolling.stop();
    statusPolling.start();
  }
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
    toast(t("app.reloaded"));
  } catch (error) {
    toast(t("app.reload_failed", {error: error.message}), {failed: true});
  }
});
window.addEventListener("hashchange", () => {
  if (tabs.length) route();
});

statusPolling.start();
start();
