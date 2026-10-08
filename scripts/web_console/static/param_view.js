// 파라미터 탭 하나(조향, 비전 크루즈, 주행 기록, 기기 설정). 섹션별 항목 카드, 검색, 바꾼 항목만 보기.
// 값은 바꾸는 즉시 저장되고, 그 그룹을 읽는 프로세스가 다시 읽는다. 다른 탭에서 고치는 항목(tab, 예: 차량 특성
// 탭의 조향비)은 여기 나오지 않고, 검색에 걸리면 그 탭으로 가는 링크를 보여 준다.
import {metaFor, params} from "./param_state.js";
import {paramCard} from "./param_editor.js";
import {el, section} from "./ui.js";

const OTHER_SECTION = "기타";
const tabLabel = tab => (tab === "vehicle" ? params().vehicle_tab.label : tab);

/* extras: 파라미터 섹션 뒤에 붙일 것들({element, start, stop}, 예: 기기 설정의 Panda 펌웨어 카드). */
export function createParamView(group, {extras = []} = {}) {
  const spec = params().groups[group];
  const bySection = new Map(spec.sections.map(name => [name, []]));
  const elsewhere = [];
  for (const key of Object.keys(params().params[group])) {
    const meta = metaFor(group, key);
    if (meta.tab) {
      elsewhere.push({key, meta, search: `${meta.label} ${key} ${meta.description}`.toLowerCase()});
      continue;
    }
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
  const moved = el("p.group-note", {hidden: true});
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
    const found = words.length ? elsewhere.filter(item => words.every(word => item.search.includes(word))) : [];
    moved.replaceChildren(...found.flatMap(({meta}, index) => [
      index ? ", " : `다른 탭에 있는 항목: `,
      el("a", {href: `#${meta.tab}`, text: `${meta.label} (${tabLabel(meta.tab)})`}),
    ]));
    moved.hidden = !found.length;
    empty.hidden = shown > 0 || found.length > 0;
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
    moved,
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
