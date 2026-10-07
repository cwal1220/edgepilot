// BEV 탭: 모델이 본 길과 앞차를 3D로(bev.js). 보드는 모델·제어 상태를 바이트 그대로 흘려보내기만 하고 해석과
// 그리기는 이 브라우저가 한다. three.js는 이 탭을 처음 열 때 불러오고, 패널과 WebGL은 한 번만 만들어 탭을
// 오가도 다시 쓴다. 스트림은 start()부터 stop()까지만 연다(탭이 보이고 페이지가 가려지지 않은 동안).
import {el} from "./ui.js";

const NOTE = "모델이 본 길과 앞차입니다. 경로가 주황·빨강인 곳은 모델이 속도를 줄이려는 곳, 흰 호는 목표 곡률, "
  + "하늘색 호는 실제 곡률입니다. 끌어서 돌리고 두 손가락·휠로 확대합니다.";

export function createBevView() {
  const statusText = el("span", {text: "불러오는 중"});
  const status = el("p.stream-status", {}, el("span.dot"), statusText);
  const body = el("div.bev-body");
  let bev = null;
  let loading = null;
  let shown = false;

  const showStatus = ({live, text}) => {
    status.classList.toggle("online", live);
    statusText.textContent = text;
  };
  const follow = () => {
    if (!bev) return;
    if (shown) bev.start();
    else bev.stop();
  };

  const load = async () => {
    try {
      loading = loading || import("./bev.js").then(module => module.createBev(showStatus));
      bev = await loading;
      body.replaceChildren(bev.element);
      follow();
    } catch (error) {
      // 브라우저는 실패한 모듈을 기억해 다시 import해도 다시 받지 않는다: 페이지를 새로 고쳐야 한다
      showStatus({live: false, text: "불러오기 실패"});
      body.replaceChildren(el("div.empty",
        {},
        el("p", {text: `BEV를 불러오지 못했습니다: ${error.message}`}),
        el("button.button", {type: "button", text: "페이지 새로 고침", onclick: () => window.location.reload()})));
    }
  };

  return {
    element: el("div.view", {}, el("p.group-note", {text: NOTE}), status, body),
    start() {
      shown = true;
      if (bev) follow();
      else load();
    },
    stop() {
      shown = false;
      follow();
    },
  };
}
