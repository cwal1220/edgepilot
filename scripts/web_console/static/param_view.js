// 파라미터 탭 하나(조향, 비전 크루즈, 주행 기록, 기기 설정). 섹션별 항목 카드, 검색, 바꾼 항목만 보기.
// 값은 바꾸는 즉시 저장되고, 그 그룹을 읽는 프로세스가 다시 읽는다. 다른 탭에서 고치는 항목(tab, 예: 차량 특성
// 탭의 조향비)은 여기 나오지 않고, 검색에 걸리면 그 탭으로 가는 링크를 보여 준다.
import {t} from "./i18n.js";
import {metaFor, params, sectionName} from "./param_state.js";
import {paramCard} from "./param_editor.js";
import {el, section} from "./ui.js";

const tabLabel = tab => (tab === "vehicle" ? params().vehicle_tab.label : tab);

// extras: 파라미터 섹션 뒤에 붙일 것들({element, start, stop, after}, 예: 기기 설정의 Panda 펌웨어 카드). after가
// 섹션 이름이면 그 섹션 바로 뒤에(예: 조향의 학습 토크 모델 카드는 토크 제어기 섹션 뒤), ""이면 섹션들 앞에(조향의 주행
// 모델 카드), 없으면 맨 뒤에.
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
    bySection.get(meta.section).push(paramCard(group, key));
  }
  const sections = [...bySection].filter(([, cards]) => cards.length).map(([name, cards]) => ({
    name, cards, element: section(sectionName(name), cards, {count: cards.length}),
  }));
  const placed = new Set(sections.map(({name}) => name));
  const extrasAfter = name => extras.filter(extra => extra.after === name).map(extra => extra.element);
  const total = sections.reduce((sum, {cards}) => sum + cards.length, 0);

  const count = el("span.toolbar-count");
  const search = el("input.search", {type: "search", placeholder: t("params.search"), "aria-label": t("params.search")});
  const modifiedOnly = el("button.chip", {type: "button", "aria-pressed": "false", text: t("params.changed_only")});
  const empty = el("div.empty", {text: t("params.no_match"), hidden: true});
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
      element.querySelector(".section-title span").textContent = visible === cards.length
        ? t("common.count", {count: cards.length}) : t("common.count_shown", {shown: visible, count: cards.length});
      shown += visible;
    }
    count.textContent = shown === total ? t("params.items", {count: total}) : t("params.items_shown", {shown, count: total});
    const found = words.length ? elsewhere.filter(item => words.every(word => item.search.includes(word))) : [];
    moved.replaceChildren(...found.flatMap(({meta}, index) => [
      index ? ", " : t("params.elsewhere"),
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
    extrasAfter(""),
    sections.map(({name, element: node}) => [node, ...extrasAfter(name)]),
    empty,
    extras.filter(extra => extra.after !== "" && !placed.has(extra.after)).map(extra => extra.element),
    el("p.file-path", {text: t("params.file", {path: params().paths[group]})}));
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
