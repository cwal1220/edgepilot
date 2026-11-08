// 조향 탭의 주행 모델 카드(scripts/web_console/driving_models.py). models/의 axmodel 파일, 고른 것, 매니저가 modeld에
// 준 것을 2초마다 읽는다. 다른 파일을 누르면 고르기 파일을 바꾸고, 매니저가 1초 안에 modeld만 새 모델로 다시 띄운다(몇
// 초 동안 모델 출력이 없다). 결합 중이면 서버가 받지 않고 버튼도 막아 둔다(해제 상태면 주행 중에도 바꾼다).
import {codeText, t} from "./i18n.js";
import {api, badges, el, helpText, note, pollingCard, runAction, section, syncRows} from "./ui.js";

const fileName = name => (name ? `${name}.axmodel` : "–");
// 매니저가 modeld에 준 경로(models/<이름>.axmodel)의 이름, 다른 경로(시험용 EDGEPILOT_MODEL)면 경로 그대로
const nameOf = path => {
  const match = /^models\/(.+)\.axmodel$/.exec(path || "");
  return match ? match[1] : path || "";
};

export function createDrivingModelCard() {
  const list = el("div.nn-models");
  const about = helpText(t("driving_model.about"), "card-note");  // 한 번 만들어 펼친 상태가 폴링에 접히지 않게
  const rows = new Map();  // 이름 -> {element(버튼), tags, meta}(ui.js syncRows)
  let latest = null;
  let busy = false;

  const choose = async name => {
    if (busy || !latest || latest.blocker || name === latest.selected) return;
    const using = latest.manager ? nameOf(latest.manager.model) : "";
    const result = await runAction({
      confirm: [t("driving_model.confirm", {model: fileName(name)}),
                using ? t("driving_model.confirm_now", {model: fileName(using)}) : null, "",
                t("driving_model.confirm_restart")],
      busy: on => {
        busy = on;
        rows.forEach(row => {
          row.element.disabled = on;
        });
      },
      call: () => api.post("/api/driving_model", {model: name}),
      done: () => t("driving_model.selected", {model: fileName(name)}),
      failed: error => t("driving_model.select_failed", {error}),
    });
    if (result) card.show(result);
  };

  const makeRow = name => {
    const tags = el("span.badge-row");
    const meta = el("small.nn-model-meta");
    return {tags, meta, element: el("button.nn-model", {type: "button", onclick: () => choose(name)},
                                    el("code.nn-model-name", {text: fileName(name)}), meta, tags)};
  };

  const render = s => {
    latest = s;
    const manager = s.manager;  // 매니저가 돌지 않으면 null
    const using = manager && s.modeld_running ? nameOf(manager.model) : "";
    const overridden = manager && (manager.source === "env" || manager.source === "argument");
    syncRows(list, rows, s.models.map(model => model.name), makeRow);
    for (const model of s.models) {
      const row = rows.get(model.name);
      const selected = s.selected === model.name;
      const failed = manager?.error === "failed" && manager.selected === model.name;
      row.element.setAttribute("aria-pressed", String(selected));
      row.element.disabled = busy || Boolean(s.blocker);
      row.meta.textContent = t("driving_model.meta", {sha: model.sha256.slice(0, 8),
                                                       mb: (model.bytes / 1e6).toFixed(1)});
      row.tags.replaceChildren(...badges([
        using === model.name ? [t("common.in_use"), "good"] : null,
        failed ? [t("driving_model.failed_badge"), "bad"]
          : selected && using !== model.name && !overridden ? [t("driving_model.switching"), "accent"] : null,
        s.default === model.name ? [t("driving_model.default"), "muted"] : null,
      ]));
    }
    return {
      badges: [!manager ? [t("driving_model.no_manager"), "bad"]
        : !s.modeld_running ? [t("driving_model.modeld_stopped"), "warn"]
        : [t("driving_model.using", {model: fileName(nameOf(manager.model))}), "good"]],
      children: [
        s.models.length ? list : note(t("driving_model.no_models")),
        s.blocker ? note(codeText("driving_model.blocked", s.blocker), "warn") : null,
        overridden ? note(t("driving_model.overridden", {model: manager.model}), "warn") : null,
        manager?.error === "missing" ? note(t("driving_model.missing", {model: fileName(manager.selected)}), "warn")
          : null,
        manager?.error === "failed"
          ? note(t("driving_model.fell_back", {model: fileName(manager.selected), fallback: fileName(s.default)}), "bad")
          : null,
        s.selection_error && s.selection_error !== "none"
          ? note(codeText("driving_model.selection_error", s.selection_error), "warn") : null,
        about,
      ],
    };
  };
  const card = pollingCard(t("driving_model.card"), {url: "/api/driving_model", intervalMs: 2000, render});

  return {element: section(t("driving_model.section"), [card.element]), after: "",
          start: card.start, stop: card.stop};
}
