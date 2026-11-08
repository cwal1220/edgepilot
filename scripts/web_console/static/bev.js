// 웹 콘솔 BEV 탭용, 모델이 보는 것의 3D 조감도: 자차 둘레의 차선선, 도로 경계, 계획 경로와 앞차를 브라우저에서
// three.js로 그린다. 보드는 발행된 상태를 페이지로 복사만 하므로(bev_data.js) 그리기에 드는 비용이 없다.
//
// sv_recorder_bev(static/bev.js)에서 옮겨 왔다: 렌더러, 바닥, 풋프린트, 경로 띠와 차(bev_car.js, 지금은 앞차용).
// 장면은 우리 것이고 차량 좌표계다: 미터 단위, x는 앞 범퍼에서 앞으로, y 왼쪽, z 위. 바닥, 선, 경로와 앞차는 모델이
// 본 노면 높이(scene.road, bev_road.js)를 따르고, 자차는 높이 0에 선다. 자차는 이 차 고유의 모델이다
// (bev_ego.js). HUD처럼 그린다(hud_scene.cc draw_scene): 같은 도달 거리, 차선 알파, 도로 경계 신뢰도, 경로 색, 앞차
// 임계값과 위험 색이고, laneless 모드에서도 선을 그린다.
// HUD에 없는 것: 모델이 감속을 계획한 곳에서 경로를 호박색~빨간색으로 물들이고 가장 느린 속도(또는 STOP)를
// 그 자리에 쓴다. controlsd가 요구하는 곡률과 실제로 얻는 곡률의 원호, 앞차 제동등, modeld의 앞차 트랙이 놓은
// 자리의 앞차(모델 거리와 광류를 융합, model/lead_tracker. 비전 크루즈가 이것을 따른다)와 막대로 그린 모델 자체
// 거리, 앞차 거리와 속도의 시간 변화 카드, 플래너의 차선 필터가 안정시킨 자차 차로 선(lane_filter.js)과 그 아래
// 가는 모델 자체 선, 그리고 정차 중에는 출발 알림의 판단 근거.
//
// 드래그로 궤도 회전, 오른쪽 드래그나 두 손가락으로 이동과 확대/축소. view("behind")와 view("top")이 카메라를
// 정한다. 장면 그래프는 한 번만 만든다: 프레임은 같은 버퍼와 인스턴스 메시(프레임이 더 필요하면 키운다)에 써 넣고,
// 프레임마다 만들거나 해제하는 것은 없다. 렌더링은 프레임이 오거나 카메라가 움직일 때만, 화면 프레임당 많아야
// 한 번 한다.

import * as THREE from "three";
import {OrbitControls} from "./three/OrbitControls.js";
import {CSS2DRenderer, CSS2DObject} from "./three/CSS2DRenderer.js";
import {carModel, fit, wheel} from "./bev_car.js";
import {BevStream, DEPARTURE_ALERT, GAS_PRESS_HORIZON, HISTORY_S, LANE_SURE, LEAD_HISTORY_S} from "./bev_data.js";
import {EGO_SIZE, makeEgo} from "./bev_ego.js";
import {arc, arrow, band, corridor, curb, egoLane, headingAt, normalAt} from "./bev_geometry.js";
import {BLACK, Batch, CORRIDOR, FOOTPRINT, GROUND, Instances, OVERLAY, SHARED, ink, modelMaterial, shader} from "./bev_gl.js";
import {clamp} from "./bev_math.js";
import {t} from "./i18n.js";
import {recall, remember} from "./storage.js";
import {DASH, el, fitRange, sparkline} from "./ui.js";

// 바닥: 5 m와 20 m 격자선, 앞으로 GRID.ahead, 양옆으로 GRID.side.
const GRID = {ahead: 160, behind: 8, side: 25, major: 20};
// 바닥 띠: 범퍼에서 bend m까지 step m마다 노면 높이를 따라 앞뒤로만 휜다(옆으로는 평평, side m씩). 그 뒤로는 behind
// m까지 높이 0, 그 앞으로는 ahead m까지 노면의 마지막 높이로 평평하게 이어진다.
const GROUND_STRIP = {behind: -800, bend: 200, ahead: 880, side: 800, step: 1};
const EGO = [EGO_SIZE.length, EGO_SIZE.width, EGO_SIZE.height];
const LEAD = [4.5, 1.8, 1.5];            // 모델은 앞차 크기를 주지 않는다: 흔한 승용차
const LANE_WIDTH = 0.15;                 // 차선선은 도색 폭만큼의 띠로
const RAW_LANE = {width: 0.05, alpha: [0.15, 0.35]};  // 거른 선 아래의 모델 자체 자차 차로 선
const CURB = {width: 0.2, height: 0.12};
const PATH_WIDTH = 1.8;                  // hud_scene.cc kPathHalfWidth의 양쪽 합
// hud_scene.cc: 이 확률보다 낮은 선과 이 신뢰도보다 낮은 경계는 그리지 않는다
const LANE_MIN_PROBABILITY = 0.05, EDGE_MIN_CONFIDENCE = 0.05;
// lead_risk(): 이 간격이나 이 접근 속도에서 최대
const RISK = {distance: 40, closing: 10};
// 앞차의 상대 속도를 그 1 s만큼 길이의 화살표로, 0.3 m/s부터
const ARROW = {seconds: 1.0, slowest: 0.3};
// 계획 감속, (지금 속도 - 그 지점 속도) / 지금 속도(적어도 floor m/s): 경로 띠는 tint부터 물들고, 가장 느린
// 지점은 label 이상이면서 labelKph 이상 느리면 라벨을 단다. stop m/s 아래면 계획이 거기서 멈춘 것이다.
const SLOW = {floor: 3, tint: 0.1, label: 0.15, labelKph: 5, stop: 0.5};
// 앞차 제동등: 가속도가 on m/s² 이하면 켜고, off보다 높아지면 다시 끈다(모델의 앞차 가속도는 조심스럽다:
// 2026-10-04 앞차 프레임의 2%에서 -1 아래, -2 아래는 한 번도 없다)
const BRAKE = {on: -1.0, off: -0.6};
// 자차 방향지시등은 이 차의 실제 등처럼 깜빡인다(CGW1, 2026-10-04 주행: 0.70 s마다)
const BLINK = {period: 0.7, duty: 0.5};
// 곡률 원호: 차가 2.5 s 동안 가는 거리만큼, 3 m/s부터
const ARC = {seconds: 2.5, min: 10, max: 70, width: 0.16, tick: 0.7, speed: 3};
// departure_alert.cc: 0.1 m/s 이하면 정차로 보고 0.5를 넘어야 움직인 것으로 본다. 2 s 가속 페달 확률은
// 0.3에서 알림을 내고, 10 s 시점의 계획은 ±5 m 안에서 대기(arm)하고 10 m를 넘으면 알림을 낸다
const DEPART = {stopped: 0.1, moving: 0.5, gas: 0.3, open: 10, arm: 5, planMax: 20};
// 앞차 카드: 최근 LEAD_HISTORY_S 동안의 앞차 거리와 상대 속도(bev_data.js가 보관한다). 속도 차트는 적어도
// ±minScale km/h, 거리 차트는 적어도 minSpan m 높이다. 충돌까지 시간은 ttc s 아래에서 보이고, warn 아래는
// 호박색, alarm 아래는 빨간색이다. 비전 크루즈는 앞차 트랙을 쓰고(controls_tick observe_cruise_lead) 출발
// 알림은 모델의 앞차를 그대로 쓴다.
const LEAD_CARD = {minScale: 10, minSpan: 10, ttc: 10, warn: 6, alarm: 3, inset: 2 / 60};  // inset: 차트 위아래 여백
// 차트별 선: 선 이름(CSS 클래스) -> 앞차 카드 행 필드
const LEAD_CHARTS = {distance: {model: "x", trend: "level", fused: "fx"}, speed: {model: "v", flow: "flow", trend: "trend", fused: "fv"}};
// 이 브라우저에만 기억하는 것: 앞차 카드의 접힘, 숨긴 레이어
const LEAD_FOLD_KEY = "edgepilot.bev.leadCardFolded";
const HIDDEN_LAYERS_KEY = "edgepilot.bev.hiddenLayers";
const MAX_PIXEL_RATIO = 1.5;
const THEME = {
  background: "#0a0d12", ground: "#0e1217", road: "#131820", gridLine: "#c8d2e0", axis: "#3f74e0",
  halo: "#8a949b", model: "#bfc5cd", lane: "#ffffff", egoLane: "#ffffff",
  modelTint: 0.3,                 // 앞차 도색이 위험 색을 얼마나 띠는지
};
// hud_draw.h 색, BEV가 HUD처럼 읽히도록
const HUD = {white: "#ffffff", green: "#30d158", blue: "#409cff", gray: "#969ca5", amber: "#ffb020", red: "#ff453a",
             cyan: "#5ac8fa"};

// 도구 막대의 레이어 버튼(이름은 bev.layer.<이름> 문구). 버튼마다 같은 이름의 그룹을 보이거나 숨기고, labels만 두
// 그룹(앞차와 경로의 라벨)이다.
const LAYERS = ["path", "lanes", "rawLanes", "edges", "lead", "curvature", "labels"];
const LAYER_GROUPS = {labels: ["lead_labels", "path_labels"]};
const VIEWS = ["behind", "top"];   // 시점 버튼, bev.view.<이름> 문구

// 이 탭의 스타일은 console.css의 BEV 부분이다. HUD 색은 createBev가 CSS 변수(--hud-<이름>, --bev-bg)로 넘긴다.

class BevView {
  // container: 3D 패널, side: 그 옆(또는 아래)의 카드용 열, 카드가 화면을 가리지 않도록
  constructor(container, side) {
    this.container = container;
    this.renderer = new THREE.WebGLRenderer({antialias: true});
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, MAX_PIXEL_RATIO));
    SHARED.uPixel.value = this.renderer.getPixelRatio();
    // 패널 위에 겹쳐 놓아, 캔버스 크기가 패널 크기에 되먹임되지 않게 한다
    Object.assign(this.renderer.domElement.style, {position: "absolute", top: "0", left: "0"});
    container.appendChild(this.renderer.domElement);
    this.labels = new CSS2DRenderer();
    this.labels.domElement.className = "bev-labels";
    container.appendChild(this.labels.domElement);
    this.hud = container.appendChild(el("div.bev-hud"));
    this.chips = {};
    for (const name of ["mode", "speed", "plan", "lane", "lat", "lead", "model"]) this.chips[name] = this.hud.appendChild(el("span"));
    // 화면 위의 도구 막대, 옆 열의 카드
    this.tools(container.appendChild(el("div.bev-tools")));
    this.sideEmpty = side.appendChild(el("div.bev-side-empty", {text: t("bev.side_empty", {seconds: LEAD_HISTORY_S})}));
    this.leadCard(side);
    this.departureCard(side);

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(THEME.background);
    SHARED.uBackground.value.set(THEME.background);
    this.scene.fog = new THREE.Fog(THEME.background, 70, 200);
    this.scene.add(new THREE.HemisphereLight("#e4ebf5", "#1c2026", 1.7));
    const key = new THREE.DirectionalLight("#ffffff", 1.9);
    key.position.set(-25, 18, 40);
    const rim = new THREE.DirectionalLight("#a8c8ff", 0.7);
    rim.position.set(40, -30, 15);
    this.scene.add(key, rim);
    this.camera = new THREE.PerspectiveCamera(50, 1, 0.5, 1500);
    this.camera.up.set(0, 0, 1);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.maxPolarAngle = Math.PI * 0.47;           // 도로 아래로는 가지 않는다
    this.controls.addEventListener("change", () => this.requestRender());

    this.layers = {};
    for (const name of ["path", "lanes", "edges", "lead", "curvature"]) this.scene.add(this.layers[name] = new THREE.Group());
    this.layers.lead.add(this.layers.lead_labels = new THREE.Group());
    this.layers.path.add(this.layers.path_labels = new THREE.Group());
    this.layers.lanes.add(this.layers.rawLanes = new THREE.Group());
    for (const name of this.hidden) this.showLayer(name, false);   // 도구 막대가 읽어 둔 숨긴 레이어
    this.tags = [];
    this.makeParts();
    this.scene.add(this.staticParts());

    new ResizeObserver(() => this.resize()).observe(container);
    this.view("behind");
  }

  // 왼쪽에 시점 버튼, 오른쪽에 레이어 버튼.
  tools(bar) {
    const views = bar.appendChild(el("div")), layers = bar.appendChild(el("div"));
    for (const name of VIEWS) {
      const button = views.appendChild(el("button", {text: t(`bev.view.${name}`)}));
      button.type = "button";
      button.addEventListener("click", () => this.view(name));
    }
    // 숨긴 레이어는 이 브라우저에 기억한다. 바꾸면 마지막 장면을 다시 그린다: 그리기가 레이어를 보고 고르는 것이
    // 있다(앞차 카드).
    const hidden = this.hidden = hiddenLayers();
    for (const name of LAYERS) {
      const button = layers.appendChild(el("button", {type: "button", text: t(`bev.layer.${name}`),
                                                      "aria-pressed": String(!hidden.has(name))}));
      button.addEventListener("click", () => {
        const on = button.getAttribute("aria-pressed") !== "true";
        button.setAttribute("aria-pressed", String(on));
        this.showLayer(name, on);
        if (on) hidden.delete(name);
        else hidden.add(name);
        remember(HIDDEN_LAYERS_KEY, JSON.stringify([...hidden]));
        this.redraw();
      });
    }
  }

  // -- 프레임마다 써 넣는 메시 ----------------------------------------------------------------------
  makeParts() {
    const plane = new THREE.PlaneGeometry(1, 1), print = {iSize: 3, iStyle: 4};
    const put = (layer, part) => { this.layers[layer].add(part.object || part.mesh); return part; };
    this.lanes = put("lanes", new Batch("color", 4, OVERLAY, 2));
    this.rawLanes = put("rawLanes", new Batch("color", 4, OVERLAY, 1));
    this.curbs = put("edges", new Batch("color", 4, new THREE.MeshStandardMaterial({
      vertexColors: true, flatShading: true, roughness: 0.8, side: THREE.DoubleSide, transparent: true, depthWrite: false}), 1));
    this.corridorMaterial = shader(...CORRIDOR, {uColor: {value: new THREE.Color()}, uSlow: {value: ink(HUD.amber)},
                                                 uStop: {value: ink(HUD.red)}, uLength: {value: 1}, uWidth: {value: PATH_WIDTH},
                                                 uStrength: {value: 1}, uTint: {value: SLOW.tint}});
    this.path = put("path", new Batch("corridor", 3, this.corridorMaterial, 3));
    this.speedTag = this.tag("path_labels", "speed");
    this.speedTag.center.set(0, 0);             // 점의 오른쪽 아래에 매단다: 앞차 라벨을 피한다
    this.arcs = put("curvature", new Batch("color", 4, OVERLAY, 6));

    this.modelMaterial = modelMaterial();
    this.lampMaterial = new THREE.MeshBasicMaterial({vertexColors: true, toneMapped: false});
    this.wheelMaterial = new THREE.MeshStandardMaterial({vertexColors: true, flatShading: true, roughness: 0.7});
    const model = carModel();
    this.lead = {
      body: put("lead", new Instances(model.body, this.modelMaterial)),
      lamps: put("lead", new Instances(model.lamps, this.lampMaterial, {colors: false})),
      brake: put("lead", new Instances(model.brake, this.lampMaterial, {colors: false})),
      wheels: put("lead", new Instances(wheel(), this.wheelMaterial, {colors: false})),
      prints: put("lead", new Instances(plane, shader(...FOOTPRINT), {order: 4, extra: print})),
      lines: put("lead", new Batch("color", 4, OVERLAY, 5)),
      tag: this.tag("lead_labels", "", true),
      gap: this.tag("lead_labels", "gap"),
      ttc: this.tag("lead_labels", "ttc"),
    };
    this.lead.ttc.center.set(1, 0.5);            // 차 옆, 왼쪽에
  }

  // group 안의 라벨, place()할 때까지 숨긴다: 점과 종류가 있는 알약 모양(dotted) 또는 일반 텍스트.
  tag(group, variant = "", dotted = false) {
    const element = el(`div.bev-tag${variant ? `.${variant}` : ""}`);
    if (dotted) element.append(el("i"), el("em"));
    element.appendChild(el("span"));
    const label = new CSS2DObject(element);
    label.center.set(0.5, dotted ? 1 : 0.5);
    label.visible = false;
    this.layers[group].add(label);
    this.tags.push(label);
    return label;
  }

  place(label, x, y, z, text, kind = "", color = "") {
    const element = label.element;
    if (label.text !== text) element.lastChild.textContent = label.text = text;
    if (label.kind !== kind && element.children.length > 1) element.children[1].textContent = label.kind = kind;
    if (label.color !== color) {
      label.color = color;
      if (element.children.length > 1) element.firstChild.style.background = color;
      else element.style.borderColor = color;
      if (element.classList.contains("speed")) element.style.color = color;
    }
    label.position.set(x, y, z);
    label.visible = true;
  }

  // -- 바뀌지 않는 부분: 바닥, 거리, 자차 --------------------------------------------------------------
  staticParts() {
    const group = new THREE.Group();
    this.ground = groundStrip(shader(...GROUND, {
      uGround: {value: new THREE.Color(THEME.ground)}, uRoad: {value: new THREE.Color(THEME.road)},
      uLine: {value: new THREE.Color(THEME.gridLine)}, uAxis: {value: new THREE.Color(THEME.axis)},
      uArea: {value: new THREE.Vector4(GRID.behind, GRID.ahead, GRID.side, 4)}}, {transparent: false, depthWrite: true}));
    this.ground.mesh.position.z = -0.01;
    group.add(this.ground.mesh);
    this.distances = [];
    for (let x = GRID.major; x <= GRID.ahead; x += GRID.major) {
      const label = new CSS2DObject(el("div.bev-dist", {text: x + " m"}));
      label.position.set(x, -GRID.side - 1.5, 0);
      group.add(label);
      this.distances.push(label);
    }
    // 자차, 앞 범퍼가 x = 0. 방향지시등과 제동등, 그리고 그 도로 위 빛은 이 차의
    // 실제 등을 따른다(drawEgo())
    const [l, w] = EGO, x = -l / 2;
    this.ego = makeEgo(this.renderer);
    this.ego.position.x = x;
    this.egoGlow = new Instances(new THREE.PlaneGeometry(1, 1), shader(...FOOTPRINT), {order: 4, extra: {iSize: 3, iStyle: 4}});
    const shadow = new Instances(new THREE.PlaneGeometry(1, 1), shader(...FOOTPRINT), {order: 4, extra: {iSize: 3, iStyle: 4}});
    shadow.begin(1);
    shadow.add(x, 0, 0.006, 0, l + 1.2, w + 1.2, 1, BLACK, [l * 0.92, w * 0.9, 0.6], [0.5, 0, 0, 0.5]);
    shadow.end();
    // 후광은 HUD의 상태 색을 띤다(drawEgo())
    this.halo = new Instances(new THREE.PlaneGeometry(1, 1), shader(...FOOTPRINT), {order: 4, extra: {iSize: 3, iStyle: 4}});
    group.add(this.ego, shadow.object, this.halo.object, this.egoGlow.object);
    this.drawEgo(null);
    return group;
  }

  // -- 카메라 -------------------------------------------------------------------------------------------
  view(name) {
    if (name === "top") {
      this.controls.target.set(45, 0, 0);
      this.camera.position.set(44.99, 0, 120);     // 살짝 뒤에서, 앞쪽이 화면 위로 오도록
    } else {
      this.controls.target.set(22, 0, 0);
      this.camera.position.set(-19, 0, 13.5);
    }
    this.controls.update();
    this.requestRender();
  }

  resize() {
    const width = this.container.clientWidth, height = this.container.clientHeight;
    if (!width || !height) return;
    this.renderer.setSize(width, height);
    this.labels.setSize(width, height);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.requestRender();
  }

  // 다음 화면 프레임에 그릴 장면. 그 사이에 오는 프레임은 건너뛴다.
  update(scene) {
    this.next = scene;
    this.requestRender();
  }

  showLayer(name, on) {
    for (const group of LAYER_GROUPS[name] || [name]) this.layers[group].visible = on;
  }

  // 마지막 장면을 다시 그린다(레이어를 바꿨을 때, 스트림이 끊겨 새 프레임이 오지 않아도)
  redraw() {
    if (this.last) this.update(this.last);
    else this.requestRender();
  }

  requestRender() {
    if (this.pending) return;
    this.pending = true;
    requestAnimationFrame(() => {
      this.pending = false;
      if (this.next) {
        this.draw(this.next);
        this.next = null;
      }
      // 안개는 카메라가 보는 곳이 얼마나 멀든 그 너머에서 시작한다
      const distance = this.camera.position.distanceTo(this.controls.target);
      this.scene.fog.near = distance + 40;
      this.scene.fog.far = distance + 160;
      SHARED.uFog.value.set(this.scene.fog.near, this.scene.fog.far);
      this.fadeLabels(distance);
      this.labels.domElement.classList.toggle("far", distance > 90);     // 멀리서는 종류를 뺀다
      this.renderer.render(this.scene, this.camera);
      this.labels.render(this.scene, this.camera);
    });
  }

  // 카메라가 보는 곳보다 한참 먼 라벨은 흐리게 해 가까운 라벨이 읽히게 한다.
  fadeLabels(distance) {
    for (const label of [...this.tags, ...this.distances]) {
      const beyond = this.camera.position.distanceTo(label.position) - distance;
      const opacity = (beyond < 40 ? 1 : Math.max(0.25, 1 - (beyond - 40) / 60)).toFixed(2);
      if (label.opacity !== opacity) label.element.style.opacity = label.opacity = opacity;
    }
  }

  // -- 한 프레임 ------------------------------------------------------------------------------------
  draw(scene) {
    this.last = scene;
    this.plan = planSummary(scene.path, scene.planNow);
    this.container.classList.toggle("stale", scene.stale);
    this.drawGround(scene.road);
    this.drawLanes(scene.lanes, scene.road);
    this.drawEdges(scene.edges, scene.road);
    this.drawPath(scene);
    this.drawCurvature(scene);
    this.drawLead(scene);
    this.drawEgo(scene);
    this.drawHud(scene);
    this.drawDeparture(scene);
    this.drawLeadCard(scene);
    this.sideEmpty.hidden = !(this.card.hidden && this.leadPanel.hidden);
  }

  // 바닥 띠를 노면 높이로 휘고, 거리 라벨을 그 위에 놓는다.
  drawGround(road) {
    const {mesh, xs} = this.ground, position = mesh.geometry.attributes.position, array = position.array;
    xs.forEach((x, i) => { array[6 * i + 2] = array[6 * i + 5] = road.at(x); });
    position.needsUpdate = true;
    for (const label of this.distances) label.position.z = road.at(label.position.x);
  }

  // 차선선은 띠로, HUD가 그리는 것과 같은 확신도로 그린다. 두 선이 모두 확실한 동안 자차 차로를 그 사이에
  // 옅게 채운다. HUD와 달리 laneless 모드에서도(경계와 함께) 그린다: 모델은 여전히 선을 보고, 차선 버튼으로
  // 숨긴다. 자차 차로 선은 플래너의 것으로 그 차선 필터를 거친 선이고, 모델 자체 선은 그 아래 가늘고
  // 회색으로 그린다(원시 차선 버튼으로 숨긴다).
  drawLanes(lanes, road) {
    const b = this.lanes, raw = this.rawLanes, color = ink(THEME.lane), gray = ink(HUD.gray);
    b.begin(road);
    raw.begin(road);
    if (lanes.length === 4 && lanes[1].probability >= LANE_SURE && lanes[2].probability >= LANE_SURE) egoLane(b, lanes[1].points, lanes[2].points, ink(THEME.egoLane));
    for (const lane of lanes) {
      if (lane.probability < LANE_MIN_PROBABILITY) continue;
      band(b, lane.points, LANE_WIDTH, 0.03, color, 0.25 + 0.65 * lane.probability);
      if (lane.raw) band(raw, lane.raw, RAW_LANE.width, 0.025, gray, RAW_LANE.alpha[0] + RAW_LANE.alpha[1] * lane.probability);
    }
    raw.end();
    b.end();
  }

  // 도로 경계는 낮은 빨간 연석으로, 모델의 확신도(1 - std)만큼 그린다.
  drawEdges(edges, road) {
    const b = this.curbs, color = ink(HUD.red);
    b.begin(road);
    for (const edge of edges) {
      const confidence = clamp(1 - edge.std, 0, 1);
      if (confidence >= EDGE_MIN_CONFIDENCE) curb(b, edge.points, CURB.width, CURB.height, color, 0.7 * confidence);
    }
    b.end();
  }

  // 계획을 HUD 경로 색의 띠로, 모델이 감속을 계획한 곳은 물들여 그린다.
  drawPath(scene) {
    const path = scene.path;
    this.path.begin(scene.road);
    this.speedTag.visible = false;
    if (path.length > 1) {
      const [hex, strength] = pathStyle(scene.state), now = Number.isFinite(scene.planNow) ? scene.planNow : 0;
      const slow = path.map(([, , speed]) => clamp((now - speed) / Math.max(now, SLOW.floor), 0, 1));
      const uniforms = this.corridorMaterial.uniforms;
      uniforms.uColor.value.copy(ink(hex));
      uniforms.uStrength.value = strength;
      uniforms.uLength.value = Math.max(corridor(this.path, path, PATH_WIDTH, 0.025, slow), 1);
      // 계획의 가장 느린 지점(또는 STOP, 또는 가속해 도달하는 속도)을 경로 옆에 쓴다. 단 앞차가 보이면 쓰지
      // 않는다: 원근 때문에 어디에 있든 앞차 라벨 위에 겹치고, 칩이 같은 내용을 보여 준다
      const plan = this.plan;
      if (plan && !(scene.lead && this.layers.lead.visible)) {
        const [nx, ny] = normalAt(path, plan.i), beside = PATH_WIDTH / 2 + 0.4, x = path[plan.i][0] - nx * beside;
        this.place(this.speedTag, x, path[plan.i][1] - ny * beside, scene.road.at(x), plan.text, "", plan.color);
      }
    }
    this.path.end();
  }

  // controlsd가 요구하는 곡률(흰색)과 측정한 곡률(청록색)의 원호를 범퍼에서 차가 ARC.seconds 동안 가는
  // 거리만큼 그리고, 각 끝에 가로 눈금을 둔다: 두 눈금이 떨어진 거리가 곧 각 곡률을 따라갔을 때 차가 놓일
  // 자리의 차이다. 곡률은 오른쪽이 +이고, BEV의 y는 왼쪽이다.
  drawCurvature(scene) {
    const b = this.arcs, curvature = scene.curvature, speed = scene.speedKph / 3.6;
    b.begin(scene.road);
    if (curvature && speed >= ARC.speed) {
      const length = clamp(speed * ARC.seconds, ARC.min, ARC.max);
      for (const [k, color, alpha, z] of [[curvature.desired, HUD.white, scene.state.active ? 0.85 : 0.45, 0.06],
                                          [curvature.actual, HUD.cyan, 0.9, 0.07]]) {
        const points = arc(-k, length), [x, y] = points[points.length - 1], [nx, ny] = normalAt(points, points.length - 1);
        band(b, points, ARC.width, z, ink(color), alpha);
        band(b, [[x + nx * ARC.tick, y + ny * ARC.tick], [x - nx * ARC.tick, y - ny * ARC.tick]], ARC.width * 1.6, z, ink(color), alpha);
      }
    }
    b.end();
  }

  // 앞차를 경로 방향으로 돌린 차로, HUD 위험 색 테두리를 두르고, 우리 범퍼에서 앞차 뒤까지의 간격과 상대
  // 속도 화살표를 붙여 그린다: 앞차 트랙이 있으면 트랙이 놓은 자리에 그리고 모델 자체 거리에는 차로를
  // 가로지르는 회색 막대를 두며, 없으면 모델의 앞차를 그린다. 옆에는 다가가는 동안 충돌까지 시간을 쓴다.
  drawLead(scene) {
    const lead = scene.lead, parts = this.lead, model = carModel();
    // 제동등은 히스테리시스를 두어 임계값 부근에서 깜빡이지 않게 한다
    const braking = this.braking = !!lead && Number.isFinite(lead.accel) && lead.accel <= (this.braking ? BRAKE.off : BRAKE.on);
    for (const part of [parts.body, parts.lamps]) part.begin(lead ? 1 : 0);
    parts.brake.begin(braking ? 1 : 0);
    parts.wheels.begin(lead ? model.wheels.length : 0);
    parts.prints.begin(lead ? (braking ? 3 : 2) : 0);
    parts.lines.begin(scene.road);
    parts.tag.visible = parts.gap.visible = parts.ttc.visible = false;
    if (lead) {
      const road = scene.road, [l, w, h] = LEAD, scale = fit(LEAD), yaw = headingAt(scene.path, lead.x);
      const c = Math.cos(yaw), s = Math.sin(yaw), x = lead.x + c * l / 2, y = lead.y + s * l / 2;
      // 앞차는 제 가운데의 노면에 서서 그 기울기만큼 앞을 든다(풋프린트도 같이)
      const z = road.at(x), pitch = Math.atan(road.slope(x)), cp = Math.cos(pitch), sp = Math.sin(pitch);
      const hex = riskColor(lead), color = ink(hex);
      parts.body.addPitched(x, y, z, yaw, pitch, ...scale, ink(THEME.model).clone().lerp(color, THEME.modelTint));
      parts.lamps.addPitched(x, y, z, yaw, pitch, ...scale, null);
      if (braking) {                            // 그리고 차 뒤 도로 위의 그 빛
        const bx = lead.x - c * 0.9;
        parts.brake.addPitched(x, y, z, yaw, pitch, ...scale, null);
        parts.prints.addPitched(bx, lead.y - s * 0.9, road.at(bx) + 0.012, yaw, pitch, 3.4, w + 2.4, 1, ink(HUD.red),
                                [0.4, w * 0.8, 1.1], [0, 0, 0, 0.55]);
      }
      for (const [wx, wy, wz, r, width] of model.wheels) {
        // 차 좌표(앞 px, 왼쪽 py, 위 pz)를 든 차의 축으로(Instances.addPitched)
        const px = wx * scale[0], py = wy * scale[1], pz = wz * scale[2], ahead = px * cp - pz * sp;
        parts.wheels.addPitched(x + c * ahead - s * py, y + s * ahead + c * py, z + px * sp + pz * cp, yaw, pitch,
                                r * scale[2], width, r * scale[2]);
      }
      parts.prints.addPitched(x, y, z + 0.006, yaw, pitch, l + 1.2, w + 1.2, 1, BLACK, [l * 0.92, w * 0.9, 0.6],
                              [0.5, 0, 0, 0.5]);
      parts.prints.addPitched(x, y, z + 0.016, yaw, pitch, l + 1.6, w + 1.6, 1, color, [l + 0.3, w + 0.3, 0.5],
                              [0.12, 0.85, 0, 0.25]);
      if (lead.x > 1) {                         // 양 끝에 눈금이 있는 치수선
        band(parts.lines, [[0.3, 0], [lead.x - 0.3, lead.y]], 0.05, 0.02, color, 0.6);
        band(parts.lines, [[0.3, -0.45], [0.3, 0.45]], 0.06, 0.02, color, 0.8);
        band(parts.lines, [[lead.x - 0.3, lead.y - 0.45], [lead.x - 0.3, lead.y + 0.45]], 0.06, 0.02, color, 0.8);
        this.place(parts.gap, lead.x / 2, lead.y / 2, road.at(lead.x / 2) + 0.05, `${lead.gap.toFixed(1)} m`, "", hex);
      }
      if (lead.relative !== null && Math.abs(lead.relative) >= ARROW.slowest) {
        arrow(parts.lines, x, y, x + c * lead.relative * ARROW.seconds, y + s * lead.relative * ARROW.seconds, color);
      }
      if (lead.tracked && Math.abs(lead.raw.x - lead.x) > 0.05) {     // 모델 자체 거리
        const ahead = lead.raw.x - lead.x, rx = lead.x + c * ahead, ry = lead.y + s * ahead, half = w / 2 + 0.2;
        band(parts.lines, [[rx + s * half, ry - c * half], [rx - s * half, ry + c * half]], 0.14, 0.035, ink(HUD.gray), 0.85);
      }
      if (lead.ttc < LEAD_CARD.ttc) {
        this.place(parts.ttc, x - s * (w / 2 + 1.2), y + c * (w / 2 + 1.2), z + 0.3, ttcText(lead.ttc), "", TTC_COLOR[ttcTone(lead.ttc)]);
      }
      this.place(parts.tag, x, y, z + h + 0.3, `${Math.round(lead.speed * 3.6)} km/h`, braking ? "BRAKE" : "LEAD", braking ? HUD.red : hex);
    }
    for (const part of [parts.body, parts.lamps, parts.brake, parts.wheels, parts.prints]) part.end();
    parts.lines.end();
  }

  // 자차 후광은 HUD 상태 색: 조향 중에는 초록이나 파랑이고 출력이 한계에 가까워지면 호박색으로 바뀌며(HUD는
  // 경로가 그렇게 한다. 여기서는 경로의 호박색이 감속을 뜻한다), 결합 중에는 회색. 그다음 등화:
  // 방향지시등과 비상등, 제동등.
  drawEgo(scene) {
    const [l, w] = EGO, state = scene && scene.state, hex = state && stateColor(state);
    const strain = state && state.steering ? clamp((Math.abs(state.output) - 0.7) / 0.3, 0, 1) : 0;
    this.halo.begin(1);
    this.halo.add(-l / 2, 0, 0.008, 0, l + 2.4, w + 2.4, 1, ink(hex || THEME.halo).clone().lerp(ink(HUD.amber), strain),
                  [l + 0.4, w + 0.4, 1.2], hex ? [0.06, 0.7, 0, 0.3] : [0.04, 0.5, 0, 0.18]);
    this.halo.end();
    // 방향지시등과 비상등(양쪽)은 BLINK.period마다 앞 절반 동안 켠다. 제동등(페달을 밟았거나 AUTO HOLD)
    const blinkers = scene && scene.blinkers, lit = (performance.now() / 1000) % BLINK.period < BLINK.period * BLINK.duty;
    const sides = [!!(blinkers && blinkers.left && lit), !!(blinkers && blinkers.right && lit)];
    const braking = !!scene && !!scene.brakeLights;
    this.ego.setLamps({left: sides[0], right: sides[1], brake: braking});
    // 그리고 도로 위의 그 빛: 깜빡이는 쪽 모서리에, 제동 중이면 차 뒤에
    this.egoGlow.begin(2 * sides.filter(Boolean).length + (braking ? 1 : 0));
    sides.forEach((on, i) => {
      if (!on) return;
      for (const corner of [-0.25, -l + 0.25]) {
        this.egoGlow.add(corner, (i ? -1 : 1) * (w / 2 + 0.1), 0.014, 0, 2.2, 2.2, 1, ink(HUD.amber), [0.3, 0.3, 0.8], [0, 0, 0, 0.75]);
      }
    });
    if (braking) this.egoGlow.add(-l - 0.6, 0, 0.012, 0, 3.4, w + 2.4, 1, ink(HUD.red), [0.4, w * 0.8, 1.1], [0, 0, 0, 0.55]);
    this.egoGlow.end();
  }

  // 아래쪽 칩: 모드, 속도, 차로 폭, 간격, 모델 프레임이 얼마나 오래됐는지.
  drawHud(scene) {
    const lead = scene.lead;
    // 앞차 트랙의 상대 속도는 항상, 모델 자체 상대 속도는 다가가는 동안만
    const relative = lead && lead.relative !== null && (lead.tracked || lead.relative * 3.6 <= -3) ? ` ${kph(lead.relative)}` : "";
    this.chip("mode", `<i style="background:${stateColor(scene.state) || "#566170"}"></i>${modeText(scene.state)}`);
    this.chip("speed", Number.isFinite(scene.speedKph) ? `${Math.round(scene.speedKph)} km/h` : "");
    const plan = this.plan;
    this.chip("plan", plan ? `<b>PLAN</b><em style="color:${plan.color}">${plan.text}</em> · ${Math.max(0, scene.path[plan.i][0]).toFixed(0)} m` : "");
    this.chip("lane", Number.isFinite(scene.laneWidth) ? `<b>LANE W</b>${scene.laneWidth.toFixed(2)} m` : "");
    const curvature = scene.curvature, speed = scene.speedKph / 3.6;
    this.chip("lat", curvature && speed >= ARC.speed
      ? `<b>LAT</b><i style="background:${HUD.white}"></i>${Math.abs(curvature.desired * speed * speed).toFixed(2)}`
        + `<i style="background:${HUD.cyan}"></i>${Math.abs(curvature.actual * speed * speed).toFixed(2)} m/s²` : "");
    this.chip("lead", lead ? `<b>LEAD</b>${lead.gap.toFixed(1)} m${relative}` : "");
    this.chip("model", scene.stale ? `<b>MODEL</b>${Number.isFinite(scene.modelAge) ? scene.modelAge.toFixed(1) + " s" : t("bev.none")}` : "", true);
  }

  // -- 정차: 출발 알림의 판단 근거 --------------------------------------------------------------------
  // D에서 정차해 있는 동안과 알림이 떠 있는 동안 보이는 카드: 2 s 가속 페달 확률과 알림을 내는 0.3, 최근
  // HISTORY_S 동안의 두 입력, 그리고 10 s 시점 계획의 x와 녹색 신호 알림을 대기시키는 구간(1.5 s
  // 동안 ±5 m), 알림을 내는 10 m.
  departureCard(parent) {
    const card = this.card = parent.appendChild(el("div.bev-card"));
    card.hidden = true;
    const head = card.appendChild(el("div.bev-card-head"));
    head.appendChild(el("b", {text: "DEPART"}));
    this.badge = head.appendChild(el("span.bev-badge"));
    const row = (title, mark, zone = 0) => {
      const line = card.appendChild(el("div.bev-row")), meter = el("div.bev-meter");
      if (zone) meter.appendChild(el("i.zone")).style.width = `${zone}%`;
      const fill = meter.appendChild(el("i.fill"));
      meter.appendChild(el("i.mark")).style.left = `${mark}%`;
      const value = el("em");
      line.append(el("span", {text: title}), meter, value);
      return {fill, value};
    };
    this.gasRow = row("GAS 2s", DEPART.gas * 100);
    // 두 입력을 제 끝값(확률 1, DEPART.planMax m)에 대한 비율로 한 차트에, 알림을 내는 가속 페달 확률을 기준선으로
    this.departureChart = sparkline({className: "bev-spark", height: 30, series: ["plan", "gas"], levels: ["limit"]});
    card.appendChild(this.departureChart.element);
    this.planRow = row("PLAN 10s", DEPART.open / DEPART.planMax * 100, DEPART.arm / DEPART.planMax * 100);
    const foot = card.appendChild(el("div.bev-card-foot"));
    for (const [title, color] of [["gas 2s", HUD.green], ["plan 10s", "var(--muted)"], [`${HISTORY_S} s`, null]]) {
      const item = foot.appendChild(el("span", {text: title}));
      if (color) item.insertBefore(el("i"), item.firstChild).style.background = color;
    }
  }

  drawDeparture(scene) {
    const card = this.card, departure = scene.departure, speed = scene.speedKph / 3.6;
    const standing = departure.drive && Number.isFinite(speed) && speed <= (card.hidden ? DEPART.stopped : DEPART.moving);
    card.hidden = !(standing || departure.alert !== DEPARTURE_ALERT.none);
    if (card.hidden) return;
    const [text, tone] = departure.alert === DEPARTURE_ALERT.greenLight ? ["GREEN", "go"]
      : departure.alert === DEPARTURE_ALERT.leadDeparted ? ["LEAD GO", "go"]
      : departure.armed ? ["ARMED", "armed"] : ["WAIT", ""];
    if (this.badge.textContent !== text) {
      this.badge.textContent = text;
      this.badge.className = `bev-badge ${tone}`.trim();
    }
    const gas = departure.gas.length > GAS_PRESS_HORIZON ? departure.gas[GAS_PRESS_HORIZON] : NaN, plan = departure.plan10;
    meter(this.gasRow, gas, 1, gas >= DEPART.gas, Number.isFinite(gas) ? gas.toFixed(2) : DASH);
    meter(this.planRow, plan, DEPART.planMax, plan > DEPART.open, Number.isFinite(plan) ? `${plan.toFixed(1)} m` : DASH);
    // 모델이 값을 못 낸 프레임(NaN)에서는 선을 끊는다(history 행: [시각 s, 2 s 가속 페달 확률, 10 s 계획 거리 m])
    const history = departure.history, end = history.length ? history[history.length - 1][0] : 0;
    this.departureChart.draw({x: [end - HISTORY_S, end], y: [0, 1], at: {limit: DEPART.gas}, data: {
      gas: history.map(([time, gas]) => [time, gas]),
      plan: history.map(([time, , plan10]) => [time, plan10 / DEPART.planMax]),
    }});
  }

  // -- 앞차가 있을 때: 거리와 상대 속도의 시간 변화 --------------------------------------------------------
  // 최근 LEAD_HISTORY_S 안에 앞차가 보였던 동안(그리고 앞차 레이어가 켜져 있을 때) 보이는 카드: 앞차 트랙(초록:
  // 모델 거리와 광류를 융합), 모델 자체 거리와 상대 속도(회색), 광류 상대 속도(청록)와 모델 거리의 추세(호박색
  // 점선: ±2 s에 걸친 직선이라 2 s 전에서 끝난다)를 값과 시간 변화로 보인다. 배지는 충돌까지 시간이다.
  leadCard(parent) {
    const card = this.leadPanel = parent.appendChild(el("div.bev-card"));
    card.hidden = true;
    const head = card.appendChild(el("div.bev-card-head"));
    // 머리를 누르면 차트를 접는다. 선택은 이 브라우저에 기억한다
    const fold = head.appendChild(el("button", {text: "LEAD"}));
    fold.type = "button";
    const setFolded = (folded) => {
      card.classList.toggle("collapsed", folded);
      fold.setAttribute("aria-expanded", String(!folded));
    };
    setFolded(recall(LEAD_FOLD_KEY) === "1");
    fold.addEventListener("click", () => {
      const folded = !card.classList.contains("collapsed");
      setFolded(folded);
      remember(LEAD_FOLD_KEY, folded ? "1" : "0");
    });
    this.ttcBadge = head.appendChild(el("span.bev-badge"));
    const value = (title, color) => {
      const line = card.appendChild(el("div.bev-value"));
      line.appendChild(el("i")).style.background = color;
      line.appendChild(el("span", {text: title}));
      return line.appendChild(el("em"));
    };
    this.leadValues = {fused: value(t("bev.lead_track"), HUD.green), model: value(t("bev.lead_model"), HUD.gray),
                       flow: value(t("bev.lead_flow"), HUD.cyan), trend: value(t("bev.lead_trend"), HUD.amber)};
    // 위의 값들이 색의 이름을 알려 주므로 차트에는 범례가 필요 없다
    const chart = (title, lines, levels) => {
      const box = card.appendChild(el("div.bev-chart"));
      box.appendChild(el("div.bev-chart-title", {text: title}));
      const plot = sparkline({className: "bev-spark tall", height: 60, series: Object.keys(lines), levels, label: title});
      box.appendChild(plot.element);
      return plot;
    };
    this.distanceChart = chart(t("bev.distance"), LEAD_CHARTS.distance, []);
    this.speedChart = chart(t("bev.relative_speed"), LEAD_CHARTS.speed, ["zero"]);
    this.leadScale = card.appendChild(el("div.bev-card-foot")).appendChild(el("span"));
  }

  drawLeadCard(scene) {
    const card = this.leadPanel, rows = scene.leads, lead = scene.lead;   // rows: 최근 LEAD_HISTORY_S
    card.hidden = !(this.layers.lead.visible && rows.some((row) => row.segment >= 0));
    if (card.hidden) return;
    // 가장 최근에 맞춘 추세(그 행에 앞차가 없으면 NaN)
    let fitted = rows.length - 1;
    while (fitted >= 0 && rows[fitted].trend === undefined) fitted--;
    const show = (em, v) => { const text = Number.isFinite(v) ? kph(v) : DASH; if (em.textContent !== text) em.textContent = text; };
    show(this.leadValues.fused, lead && lead.tracked ? lead.relative : NaN);
    show(this.leadValues.model, lead ? lead.raw.relative : NaN);
    show(this.leadValues.flow, lead && lead.flow ? lead.flow.relative : NaN);
    show(this.leadValues.trend, fitted >= 0 ? rows[fitted].trend : NaN);
    const ttc = lead ? lead.ttc : Infinity, tone = ttcTone(ttc);
    const text = ttc < LEAD_CARD.ttc ? ttcText(ttc) : `TTC ${DASH}`;
    if (this.ttcBadge.textContent !== text) this.ttcBadge.textContent = text;
    if (this.ttcBadge.className !== `bev-badge ${tone}`.trim()) this.ttcBadge.className = `bev-badge ${tone}`.trim();
    // 접었으면 차트와 눈금 줄이 숨어 있으니 그리지 않는다(펼치면 다음 프레임에 그린다)
    if (card.classList.contains("collapsed")) return;
    // 차트마다 선 이름 -> 행 필드의 [[시각, 값 × unit]]. 값이 없는 행(NaN)에서 선이 끊긴다.
    const end = rows[rows.length - 1].t, x = [end - LEAD_HISTORY_S, end];
    const columns = (fields, unit = 1) => Object.fromEntries(Object.entries(fields).map(([name, key]) =>
      [name, rows.map((row) => [row.t, row[key] * unit])]));
    const values = (data) => Object.values(data).flat().map(([, v]) => v);
    // 거리: 보이는 값의 최저에서 최고까지, 적어도 LEAD_CARD.minSpan m(위아래로 5%씩 더)
    const distance = columns(LEAD_CHARTS.distance);
    const near = fitRange(values(distance), {minSpan: LEAD_CARD.minSpan, pad: 0.05});
    this.distanceChart.draw({x, y: near, data: distance, inset: LEAD_CARD.inset});
    // 상대 속도(km/h): 0이 가운데, 적어도 ±LEAD_CARD.minScale, 10 단위로
    const speed = columns(LEAD_CHARTS.speed, 3.6);
    const scale = fitRange(values(speed), {symmetric: true, minSpan: 2 * LEAD_CARD.minScale, step: 10});
    this.speedChart.draw({x, y: scale, data: speed, at: {zero: 0}, inset: LEAD_CARD.inset});
    const note = t("bev.lead_scale", {low: near[0].toFixed(0), high: near[1].toFixed(0), scale: scale[1],
                                      seconds: LEAD_HISTORY_S});
    if (this.leadScale.textContent !== note) this.leadScale.textContent = note;
  }

  // 화면 아래의 작은 칸 하나. HUD처럼 영어 약어(PLAN, LANE W, LAT, LEAD, MODEL)를 쓴다(LCD HUD와 같은 말).
  chip(name, html, warn = false) {
    const span = this.chips[name];
    if (span.shown === html) return;
    span.shown = html;
    span.innerHTML = html;
    span.classList.toggle("warn", warn);
  }
}


// GROUND_STRIP의 바닥 띠: {mesh, xs}. 앞 거리 xs마다 왼쪽(y = side)과 오른쪽 정점 한 쌍이고, drawGround()가 그 높이를
// 정한다. 격자 셰이더(GROUND)는 월드 x, y로 선을 그어 격자가 띠를 따라 휜다.
function groundStrip(material) {
  const {behind, bend, ahead, side, step} = GROUND_STRIP, xs = [behind];
  for (let x = 0; x <= bend; x += step) xs.push(x);
  xs.push(ahead);
  const positions = new Float32Array(6 * xs.length), index = [];
  xs.forEach((x, i) => {
    positions.set([x, side, 0, x, -side, 0], 6 * i);
    if (i) index.push(2 * i - 2, 2 * i - 1, 2 * i, 2 * i - 1, 2 * i + 1, 2 * i);
  });
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3).setUsage(THREE.DynamicDrawUsage));
  geometry.setIndex(index);
  const mesh = new THREE.Mesh(geometry, material);
  mesh.frustumCulled = false;
  return {mesh, xs};
}

// 이 브라우저에 기억한 숨긴 레이어(LAYERS의 이름만, 저장값이 깨졌으면 없다)
function hiddenLayers() {
  try {
    const names = JSON.parse(recall(HIDDEN_LAYERS_KEY) || "[]");
    return new Set(Array.isArray(names) ? names.filter((name) => LAYERS.includes(name)) : []);
  } catch {
    return new Set();
  }
}

// 상대 속도(m/s)를 LEAD 칩과 카드가 보여 주는 대로 부호 있는 km/h로.
function kph(v) {
  const k = Math.round(v * 3.6);
  return `${k > 0 ? "+" : k < 0 ? "−" : ""}${Math.abs(k)} km/h`;
}

// 충돌까지 시간의 단계(배지의 클래스): LEAD_CARD.alarm s 아래는 "alarm", LEAD_CARD.warn s 아래는 "warn", 그리고
// 그 라벨의 색.
function ttcTone(ttc) {
  return ttc < LEAD_CARD.alarm ? "alarm" : ttc < LEAD_CARD.warn ? "warn" : "";
}
const TTC_COLOR = {alarm: HUD.red, warn: HUD.amber, "": HUD.gray};
const ttcText = (ttc) => `TTC ${ttc.toFixed(1)} s`;

// -- HUD의 규칙 -------------------------------------------------------------------------------------
// 앞쪽에서 계획이 가장 많이 감속하는 곳(멈추면 STOP), 또는 대신 가속하면 끝에서 도달하는 속도:
// {i(경로 인덱스), text, color}. 어느 쪽도 SLOW.labelKph만큼이 안 되면 null.
function planSummary(path, now) {
  if (path.length < 2 || !Number.isFinite(now)) return null;
  let k = -1;
  for (let i = 0; i < path.length; i++) if (path[i][0] >= 0 && (k < 0 || path[i][2] < path[k][2])) k = i;
  if (k < 0) return null;
  const drop = now - path[k][2], end = path.length - 1;
  if (drop * 3.6 >= SLOW.labelKph && drop / Math.max(now, SLOW.floor) >= SLOW.label) {
    const stop = path[k][2] < SLOW.stop;
    return {i: k, text: stop ? "STOP" : `↓ ${Math.round(path[k][2] * 3.6)} km/h`, color: stop ? HUD.red : HUD.amber};
  }
  if ((path[end][2] - now) * 3.6 >= SLOW.labelKph) return {i: end, text: `↑ ${Math.round(path[end][2] * 3.6)} km/h`, color: HUD.green};
  return null;
}

// hud_draw.h state_color(): 조향 상태의 색, 결합 중이 아니면 null. 조향 결함·해제 예고는 빨강이다. HUD의 Panda 결함
// 빨강과 서비스 대기 호박색은 BEV가 PandaState·ManagerState를 읽지 않아 뺀다.
function stateColor(state) {
  if (state.alarm) return HUD.red;
  if (state.steering) return state.laneless ? HUD.blue : HUD.green;
  return state.engaged ? HUD.gray : null;
}

function modeText(state) {
  if (state.active) return state.laneless ? "LANELESS" : "LANE";
  return state.engaged || state.enabled ? "READY" : "OFF";
}

// path_color(): [색, 세기]: 결합 중이 아니면 옅은 흰색, 조향하지 않으면 회색, 조향 중이면 초록이나 파랑.
// (출력 한계 부근의 호박색은 여기서는 자차 후광에 있다, drawEgo().)
function pathStyle(state) {
  if (!state.engaged) return [HUD.white, 0.6];
  if (!state.steering) return [HUD.gray, 0.85];
  return [state.laneless ? HUD.blue : HUD.green, 1];
}

// lead_risk(): 가깝거나 빠르게 다가갈수록 1에 가깝다. 흰색, 호박색, 그다음 빨간색. (HUD는 SCC 레이더의
// 간격을 먼저 쓰지만 이 차의 레이더에는 앞차가 잡히지 않으므로 여기서는 뺐다.)
function riskColor(lead) {
  const relative = lead.relative ?? 0;
  const risk = clamp(clamp(1 - lead.gap / RISK.distance, 0, 1) + clamp(-relative / RISK.closing, 0, 1), 0, 1);
  if (risk > 0.75) return HUD.red;
  return "#" + ink(HUD.white).clone().lerp(ink(HUD.amber), risk / 0.75).getHexString();
}

// 출발 카드의 미터 행: full 대비 value, 알림을 낼 때 초록으로 채운다.
function meter(row, value, full, go, text) {
  row.fill.style.width = `${(Number.isFinite(value) ? clamp(value / full, 0, 1) : 0) * 100}%`;
  row.fill.classList.toggle("go", go);
  if (row.value.textContent !== text) row.value.textContent = text;
}

// BEV 탭: 뷰와 그 스트림을 한 번 만든다. start()와 stop()은 탭과 페이지가 보이는지를 따른다.
// onStatus({state, text})가 스트림 상태를 알린다(BevStream.status).
export function createBev(onStatus) {
  const element = el("div.bev-layout");
  element.style.setProperty("--bev-bg", THEME.background);
  for (const [name, color] of Object.entries(HUD)) element.style.setProperty(`--hud-${name}`, color);
  const view = new BevView(element.appendChild(el("div.bev-panel")), element.appendChild(el("div.bev-side")));
  // 끊기거나 모델이 멈추면 장면·칸·카드를 흐리게 한다: 새 프레임이 오지 않아 그리기는 마지막 모습에 머문다
  const stream = new BevStream({
    onScene: (scene) => view.update(scene),
    onStatus: (status) => {
      element.classList.toggle("offline", status.state === "bad" || status.state === "wait");
      onStatus(status);
    },
  });
  return {element, start: () => stream.start(), stop: () => stream.stop()};
}
