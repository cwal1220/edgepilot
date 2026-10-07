// 기기 설정 탭의 Panda 펌웨어 카드. pandad가 쓰는 상태를 1초마다 읽고, 주차 중일 때만 설치된 이미지로 플래싱을
// 요청한다(쓰는 것은 pandad이고, 그쪽도 같은 조건을 다시 본다).
import {actionButton, api, confirmAction, el, liveCard, note, poller, section, toast} from "./ui.js";

function describe(p) {
  const flash = p.flash || {};
  const flashing = flash.state === "flashing" || p.request_pending;
  const badgeList = [
    !p.available ? ["pandad 없음", "bad"] : p.mode === "app" ? ["연결됨", "good"]
      : p.mode === "bootstub" ? ["bootstub", "warn"] : ["판다 없음", "bad"],
    p.mode === "app" && p.image.valid ? (p.up_to_date ? ["최신", "accent"] : ["다른 버전", "warn"]) : null,
    flashing ? [`쓰는 중 ${flash.percent || 0}%`, "accent"]
      : flash.state === "done" ? ["플래싱 완료", "good"]
      : flash.state === "failed" ? ["마지막 시도 실패", "bad"] : null,
  ];
  const current = p.mode === "app" ? (p.firmware_version || "–") : p.mode === "bootstub" ? "bootstub (앱 없음)" : "–";
  const children = [
    el("dl.facts",
      {},
      el("dt", {text: "판다"}), el("dd", {}, el("code", {text: current}), p.hw_name ? el("small", {text: p.hw_name}) : null),
      el("dt", {text: "설치된 이미지"}),
      el("dd", {}, el("code", {text: p.image.present ? (p.image.version || "–") : "없음"}),
         p.image.present ? el("small", {text: `${p.image.size.toLocaleString("ko-KR")} B`}) : null)),
    p.image.present && !p.image.valid ? note(`이미지 문제: ${p.image.error}`, "warn") : null,
    flashing ? note(`${p.flash_step_text || "요청 전달 중"} · 약 10초 걸립니다.`, "prior")
      : flash.state === "failed" ? note(`마지막 시도: ${p.flash_error_text}${flash.detail ? ` · ${flash.detail}` : ""}`, "warn")
      : flash.state === "done" ? note(`마지막 플래싱: ${flash.version}`) : null,
    note("P단에 정지해 있고 조향이 꺼져 있을 때만 씁니다. 판다가 두 번 재부팅하는 약 10초 동안 조향 제어가 끊기고 "
         + "하네스가 순정 카메라 배선으로 돌아갑니다."),
  ];
  const reason = !p.available ? "pandad가 실행 중이 아닙니다"
    : flashing ? "쓰는 중입니다"
    : !p.image.valid ? (p.image.present ? "설치된 이미지를 쓸 수 없습니다" : "설치된 이미지가 없습니다")
    : p.mode === "none" ? "판다가 USB에 없습니다"
    : p.blocker ? p.blocker_text : "";
  const title = p.mode === "bootstub" ? "펌웨어 다시 쓰기" : p.up_to_date ? "같은 버전 다시 쓰기" : "펌웨어 플래싱";
  return {badgeList, children, reason, title};
}

export function createPandaCard() {
  let latest = null;
  const button = actionButton({icon: "⤓", danger: true, onclick: () => flash()});
  const card = liveCard("Panda 펌웨어", {actions: [button]});

  const show = p => {
    latest = p;
    const {badgeList, children, reason, title} = describe(p);
    card.update(badgeList, children);
    button.setContent(title, reason || `판다에 쓸 이미지: ${p.image.version}`);
    button.disabled = Boolean(reason);
  };

  const flash = async () => {
    const p = latest;
    if (!p || !p.image.valid) return;
    if (!confirmAction(["판다 펌웨어를 플래싱할까요?",
                        `지금: ${p.mode === "app" ? p.firmware_version : p.mode}`,
                        `쓸 이미지: ${p.image.version} (${p.image.size.toLocaleString("ko-KR")} B)`, "",
                        "판다가 두 번 재부팅하는 약 10초 동안 조향 제어가 끊기고 하네스가 순정 카메라 배선으로 돌아갑니다.",
                        "P단에 정지한 상태에서만 진행됩니다."])) return;
    button.disabled = true;
    try {
      show(await api.post("/api/panda/flash", {version: p.image.version}));
      toast("판다 펌웨어 플래싱을 요청했습니다");
    } catch (error) {
      toast(`플래싱 요청 실패: ${error.message}`, {failed: true});
      button.disabled = false;
    }
  };

  const polling = poller(async isCurrent => {
    try {
      const p = await api.get("/api/panda");
      if (isCurrent()) show(p);
    } catch (error) {
      if (isCurrent()) card.update([["읽기 실패", "bad"]], [note(error.message, "warn")]);
    }
  }, 1000);
  return {element: section("판다", [card.element]), start: polling.start, stop: polling.stop};
}
