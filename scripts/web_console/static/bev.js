// The bird's eye view of what the model sees, in 3D, for the web console's BEV tab: lane lines, road
// edges, the planned path and the lead round the ego car, drawn with three.js in the browser. The
// board only copies the published state to the page (bev_data.js), so the drawing costs it nothing.
//
// Ported from sv_recorder_bev (static/bev.js): its renderer, ground, footprints, corridor and car
// (bev_car.js, now the lead's); the scene is ours, in vehicle coordinates: metres, x forward from the
// front bumper, y left, z up. The ego car is the car's own model (bev_ego.js). It draws as the HUD
// does (hud_scene.cc draw_scene): the same reach, lane alpha, road edge confidence, path colours, lead
// threshold and risk colour; lines also in laneless mode.
// Beyond the HUD: the path is tinted amber to red where the model plans to slow down, with the
// slowest speed (or STOP) where it is; arcs of the curvature controlsd asks for and the one it gets;
// the lead's brake lights; and, standing, what the departure alert decides from.
//
// Drag to orbit, right-drag or two fingers to pan and zoom; view("behind") and view("top") set the
// camera. The scene graph is made once: a frame writes itself into the same buffers and instanced
// meshes (grown when a frame needs more), and nothing is made or freed per frame. Rendering happens
// only when a frame arrives or the camera moves, at most once per display frame.

import * as THREE from "three";
import {OrbitControls} from "./three/OrbitControls.js";
import {CSS2DRenderer, CSS2DObject} from "./three/CSS2DRenderer.js";
import {carModel, fit, wheel} from "./bev_car.js";
import {BevStream} from "./bev_data.js";
import {EGO_SIZE, makeEgo} from "./bev_ego.js";

// The ground: 5 m and 20 m grid lines, GRID.ahead in front and GRID.side to each side.
const GRID = {ahead: 160, behind: 8, side: 25, major: 20};
const EGO = [EGO_SIZE.length, EGO_SIZE.width, EGO_SIZE.height];
const LEAD = [4.5, 1.8, 1.5];            // the model gives the lead no size: a typical car
const LANE_WIDTH = 0.15;                 // a lane line as a band as wide as its paint
const CURB = {width: 0.2, height: 0.12};
const PATH_WIDTH = 1.8;                  // hud_scene.cc kPathHalfWidth, both sides
// hud_scene.cc: lines below this probability and edges below this confidence are not drawn
const LANE_MIN_PROBABILITY = 0.05, EDGE_MIN_CONFIDENCE = 0.05;
// lead_risk(): full at this gap or this closing speed
const RISK = {distance: 40, closing: 10};
// the lead's speed relative to ours as an arrow as long as 1 s of it, from 0.3 m/s
const ARROW = {seconds: 1.0, slowest: 0.3};
// The planned slowdown, (speed now - speed there) / speed now (at least floor m/s): the corridor
// tints from tint, and the slowest point is labelled from label and labelKph slower; below stop m/s
// the plan has stopped there.
const SLOW = {floor: 3, tint: 0.1, label: 0.15, labelKph: 5, stop: 0.5};
// the lead's brake lights: on at accel m/s² or below, off again above off (the model's lead accel
// is cautious: below -1 in 2% of the 2026-10-04 lead frames, never below -2)
const BRAKE = {on: -1.0, off: -0.6};
// the ego car's turn signals flash as the car's own lamps do (CGW1, 2026-10-04 drives: every 0.70 s)
const BLINK = {period: 0.7, duty: 0.5};
// curvature arcs: as far as the car goes in 2.5 s, from 3 m/s
const ARC = {seconds: 2.5, min: 10, max: 70, width: 0.16, tick: 0.7, speed: 3};
// departure_alert.cc: standing at 0.1 m/s or less until moving above 0.5, the 2 s gas press
// probability that fires at 0.3, the plan at 10 s that arms within ±5 m and fires above 10 m
const DEPART = {stopped: 0.1, moving: 0.5, gas: 0.3, open: 10, arm: 5, planMax: 20, history: 10};
const MAX_PIXEL_RATIO = 1.5;
const THEME = {
  background: "#0a0d12", ground: "#0e1217", road: "#131820", gridLine: "#c8d2e0", axis: "#3f74e0",
  halo: "#8a949b", model: "#bfc5cd", lane: "#ffffff", egoLane: "#ffffff",
  modelTint: 0.3,                 // how much of the lead's risk colour its paint takes
};
// hud_draw.h colours, so the BEV reads like the HUD
const HUD = {white: "#ffffff", green: "#30d158", blue: "#409cff", gray: "#969ca5", amber: "#ffb020", red: "#ff453a",
             cyan: "#5ac8fa"};

// The toolbar's layer buttons, and the groups each one shows or hides.
const LAYERS = {path: "경로", lanes: "차선", edges: "도로 경계", lead: "앞차", curvature: "곡률", labels: "라벨"};
const LAYER_GROUPS = {path: ["path"], lanes: ["lanes"], edges: ["edges"], lead: ["lead"], curvature: ["curvature"],
                      labels: ["lead_labels", "path_labels"]};
const VIEWS = {behind: "뒤에서", top: "위에서"};

// -- colours ---------------------------------------------------------------------------------------
const BLACK = new THREE.Color(0, 0, 0);
const inks = new Map();
const ink = (hex) => inks.get(hex) || inks.set(hex, new THREE.Color(hex)).get(hex);
const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const smoothstep = (a, b, x) => { const t = clamp((x - a) / (b - a), 0, 1); return t * t * (3 - 2 * t); };
const el = (tag, className = "", text) => {
  const node = document.createElement(tag);
  if (className) node.className = className;
  if (text !== undefined) node.textContent = text;
  return node;
};

// -- geometry rewritten every frame ---------------------------------------------------------------------
// Triangles in buffers kept from frame to frame: position and one more attribute (a colour with
// alpha, uv, ...) of size values. A frame begin()s, writes, and end()s, which sends the new part.
class Batch {
  constructor(name, size, material, order) {
    this.name = name;
    this.size = size;
    this.geometry = new THREE.BufferGeometry();
    this.mesh = new THREE.Mesh(this.geometry, material);
    Object.assign(this.mesh, {frustumCulled: false, renderOrder: order, matrixAutoUpdate: false});
    this.positions = new Float32Array(3 * 512);
    this.values = new Float32Array(size * 512);
    this.indices = new Uint32Array(1024);
    this.v = this.n = 0;
    this.grown = true;
  }

  begin() { this.v = this.n = 0; }

  // Room for vertices and indices more; the first new vertex's index.
  reserve(vertices, indices) {
    const grow = (array, need) => {
      const bigger = new array.constructor(Math.max(2 * array.length, need));
      bigger.set(array);
      this.grown = true;
      return bigger;
    };
    if (3 * (this.v + vertices) > this.positions.length) {
      this.positions = grow(this.positions, 3 * (this.v + vertices));
      this.values = grow(this.values, this.size * (this.v + vertices));
    }
    if (this.n + indices > this.indices.length) this.indices = grow(this.indices, this.n + indices);
    return this.v;
  }

  put(x, y, z, a, b, c, d) {
    const v = this.v++, o = this.size * v, values = this.values;
    this.positions[3 * v] = x; this.positions[3 * v + 1] = y; this.positions[3 * v + 2] = z;
    values[o] = a;
    if (this.size > 1) values[o + 1] = b;
    if (this.size > 2) values[o + 2] = c;
    if (this.size > 3) values[o + 3] = d;
  }

  tri(a, b, c) { this.indices[this.n++] = a; this.indices[this.n++] = b; this.indices[this.n++] = c; }

  end() {
    const geometry = this.geometry;
    if (this.grown) {                       // new buffers: made again on the GPU at the next render
      geometry.dispose();
      geometry.setAttribute("position", new THREE.BufferAttribute(this.positions, 3).setUsage(THREE.DynamicDrawUsage));
      geometry.setAttribute(this.name, new THREE.BufferAttribute(this.values, this.size).setUsage(THREE.DynamicDrawUsage));
      geometry.setIndex(new THREE.BufferAttribute(this.indices, 1).setUsage(THREE.DynamicDrawUsage));
      this.grown = false;
    } else if (this.n) {                    // the same buffers: only the part this frame wrote
      for (const [attribute, count] of [[geometry.attributes.position, 3 * this.v], [geometry.attributes[this.name], this.size * this.v],
                                        [geometry.index, this.n]]) {
        attribute.clearUpdateRanges();
        attribute.addUpdateRange(0, count);
        attribute.needsUpdate = true;
      }
    }
    geometry.setDrawRange(0, this.n);
    this.mesh.visible = this.n > 0;
  }
}

// Instances of one geometry, as many as a frame has, in one draw: each placed at x, y, z, turned
// by yaw and stretched by sx, sy, sz, with a colour (colors) and attributes of their own (extra,
// name: size). The mesh is made again, bigger, when a frame has more than it holds.
class Instances {
  constructor(geometry, material, {order = 0, colors = true, extra = {}} = {}) {
    this.geometry = Object.keys(extra).length ? geometry.clone() : geometry;
    Object.assign(this, {material, order, colors, extra});
    this.object = new THREE.Group();
    this.capacity = 0;
    this.count = 0;
    this.allocate(4);
  }

  allocate(capacity) {
    if (this.mesh) {
      this.object.remove(this.mesh);
      this.mesh.dispose();
      if (this.attributes.length) this.geometry.dispose();
    }
    this.mesh = new THREE.InstancedMesh(this.geometry, this.material, capacity);
    this.mesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
    if (this.colors) {
      this.mesh.instanceColor = new THREE.InstancedBufferAttribute(new Float32Array(3 * capacity), 3).setUsage(THREE.DynamicDrawUsage);
    }
    this.attributes = Object.entries(this.extra).map(([name, size]) => {
      const attribute = new THREE.InstancedBufferAttribute(new Float32Array(size * capacity), size).setUsage(THREE.DynamicDrawUsage);
      this.geometry.setAttribute(name, attribute);
      return attribute;
    });
    Object.assign(this.mesh, {frustumCulled: false, renderOrder: this.order, count: 0, matrixAutoUpdate: false});
    this.object.add(this.mesh);
    this.capacity = capacity;
  }

  begin(count) {
    if (count > this.capacity) this.allocate(Math.max(count, 2 * this.capacity));
    this.count = 0;
  }

  add(x, y, z, yaw, sx, sy, sz, color, ...extra) {
    const i = this.count++, m = this.mesh.instanceMatrix.array, o = 16 * i, c = Math.cos(yaw), s = Math.sin(yaw);
    m[o] = c * sx; m[o + 1] = s * sx; m[o + 2] = 0; m[o + 3] = 0;
    m[o + 4] = -s * sy; m[o + 5] = c * sy; m[o + 6] = 0; m[o + 7] = 0;
    m[o + 8] = 0; m[o + 9] = 0; m[o + 10] = sz; m[o + 11] = 0;
    m[o + 12] = x; m[o + 13] = y; m[o + 14] = z; m[o + 15] = 1;
    if (this.colors) {
      const a = this.mesh.instanceColor.array;
      a[3 * i] = color.r; a[3 * i + 1] = color.g; a[3 * i + 2] = color.b;
    }
    this.attributes.forEach((attribute, k) => attribute.array.set(extra[k], attribute.itemSize * i));
  }

  end() {
    this.mesh.count = this.count;
    this.mesh.visible = this.count > 0;
    if (!this.count) return;
    for (const attribute of [this.mesh.instanceMatrix, this.mesh.instanceColor, ...this.attributes]) {
      if (!attribute) continue;
      attribute.clearUpdateRanges();
      attribute.addUpdateRange(0, attribute.itemSize * this.count);
      attribute.needsUpdate = true;
    }
  }
}

// -- materials --------------------------------------------------------------------------------------
// Shared uniforms, set once per render: fog (near, far, as view depth) and the pixel ratio.
const SHARED = {uFog: {value: new THREE.Vector2(70, 200)}, uPixel: {value: 1}, uBackground: {value: new THREE.Color(THEME.background)}};
const FOG_FADE = "float fogFade(float depth) { return 1.0 - smoothstep(uFog.x, uFog.y, depth); }";

function shader(vertex, fragment, uniforms = {}, options = {}) {
  return new THREE.ShaderMaterial({
    uniforms: {...SHARED, ...uniforms}, transparent: true, depthWrite: false, ...options,
    vertexShader: vertex,
    fragmentShader: `uniform vec2 uFog; uniform float uPixel; uniform vec3 uBackground;\n${FOG_FADE}\n${fragment}`,
  });
}

// Lit models, flat-shaded: their own colours, the instance colour on what is paint.
function modelMaterial() {
  const material = new THREE.MeshStandardMaterial({vertexColors: true, flatShading: true, roughness: 0.55, metalness: 0.05});
  material.onBeforeCompile = (program) => {
    program.vertexShader = program.vertexShader
      .replace("#include <common>", "#include <common>\nattribute float paint;")
      .replace("#include <color_vertex>", `#include <color_vertex>
        #ifdef USE_INSTANCING_COLOR
          vColor.xyz = color.xyz * mix(vec3(1.0), instanceColor.xyz, paint);
        #endif`);
  };
  material.customProgramCacheKey = () => "bev-model";
  return material;
}

// Unlit, vertex colours with alpha: lane lines, the ego lane, the gap and arrow.
const overlayMaterial = () => new THREE.MeshBasicMaterial({vertexColors: true, transparent: true, depthWrite: false,
                                                           side: THREE.DoubleSide, toneMapped: false});

// A footprint on the road, in metres round a length x width rectangle with rounded corners:
// iStyle is its fill, outline, corner brackets only (1) and glow, iSize its length, width and
// the margin the glow fades out in. With a black colour it is a soft contact shadow.
const FOOTPRINT = [`
  attribute vec3 iSize; attribute vec4 iStyle;
  varying vec2 vQ; varying vec3 vSize; varying vec4 vStyle; varying vec3 vColor; varying float vDepth;
  void main() {
    vQ = position.xy * (iSize.xy + 2.0 * iSize.z); vSize = iSize; vStyle = iStyle; vColor = instanceColor;
    vec4 view = modelViewMatrix * instanceMatrix * vec4(position, 1.0);
    vDepth = -view.z;
    gl_Position = projectionMatrix * view;
  }`, `
  varying vec2 vQ; varying vec3 vSize; varying vec4 vStyle; varying vec3 vColor; varying float vDepth;
  void main() {
    vec2 hs = vSize.xy * 0.5;
    float r = min(0.35, min(hs.x, hs.y) * 0.45);
    vec2 k = abs(vQ) - hs + r;
    float d = length(max(k, 0.0)) + min(max(k.x, k.y), 0.0) - r;               // metres outside the outline
    float aa = fwidth(d);
    float fill = vStyle.x * (1.0 - smoothstep(-aa, aa, d));
    float width = max(0.045, 1.1 * uPixel * aa);
    float line = vStyle.y * (1.0 - smoothstep(width - aa, width + aa, abs(d)));
    if (vStyle.z > 0.5) {
      vec2 corner = abs(vQ) - (hs - min(0.8, min(hs.x, hs.y) * 0.7));
      line *= step(0.0, min(corner.x, corner.y));
    }
    float glow = vStyle.w * exp(-max(d, 0.0) / max(vSize.z * 0.4, 0.05)) * (1.0 - step(vSize.z, d));
    float alpha = 1.0 - (1.0 - fill) * (1.0 - line) * (1.0 - glow);
    alpha *= fogFade(vDepth);
    if (alpha < 0.004) discard;
    gl_FragColor = vec4(mix(vColor, vec3(1.0), line * 0.15), alpha);
    #include <colorspace_fragment>
  }`];

// The planned path as a corridor as wide as the HUD's ribbon: bright edges in the HUD's path colour,
// chevrons every 4 m, fading out ahead; uStrength dims it as the HUD does when not steering. Inside,
// it turns amber, then red, where the model plans to slow down. corridor is across (0..1), along
// (metres) and the planned slowdown there (0..1, SLOW).
const CORRIDOR = [`
  attribute vec3 corridor;
  varying vec3 vC; varying float vDepth;
  void main() {
    vC = corridor;
    vec4 view = modelViewMatrix * vec4(position, 1.0);
    vDepth = -view.z;
    gl_Position = projectionMatrix * view;
  }`, `
  uniform vec3 uColor; uniform vec3 uSlow; uniform vec3 uStop; uniform float uLength; uniform float uWidth;
  uniform float uStrength; uniform float uTint;
  varying vec3 vC; varying float vDepth;
  void main() {
    float across = abs(vC.x - 0.5) * 2.0, aa = fwidth(across);
    float edge = smoothstep(1.0 - 0.1 - aa, 1.0 - 0.1 + aa, across);
    float v = (vC.y - across * uWidth * 0.4) / 4.0, av = fwidth(v);
    float stripe = min(fract(v), 1.0 - fract(v));
    float chevron = 1.0 - smoothstep(0.035 - av, 0.035 + av, stripe);
    float slow = smoothstep(uTint, uTint + 0.25, vC.z);
    // fading out ahead, but not where it slows down: that is mostly at its end, by a stop
    float fade = max(1.0 - smoothstep(0.6, 1.0, vC.y / uLength), 0.8 * slow) * smoothstep(0.0, 1.5, vC.y);
    vec3 inner = mix(uColor, mix(uSlow, uStop, smoothstep(0.45, 0.9, vC.z)), slow);
    float alpha = (0.1 + 0.2 * slow + 0.7 * edge + (0.22 + 0.3 * slow) * chevron * (1.0 - edge)) * fade * fogFade(vDepth) * uStrength;
    if (alpha < 0.004) discard;
    gl_FragColor = vec4(mix(inner, mix(uColor, vec3(1.0), 0.15), edge), alpha);
    #include <colorspace_fragment>
  }`];

// The ground: the BEV's range a shade lighter, 5 m and 20 m grid lines that fade before they crowd
// into moire, the ego axes, all fading into the background far away.
const GROUND = [`
  varying vec3 vWorld; varying float vDepth;
  void main() {
    vec4 world = modelMatrix * vec4(position, 1.0);
    vWorld = world.xyz;
    vec4 view = viewMatrix * world;
    vDepth = -view.z;
    gl_Position = projectionMatrix * view;
  }`, `
  uniform vec3 uGround; uniform vec3 uRoad; uniform vec3 uLine; uniform vec3 uAxis; uniform vec4 uArea;
  varying vec3 vWorld; varying float vDepth;
  float grid(vec2 p, float cell, float width) {
    vec2 g = p / cell, w = fwidth(g);
    vec2 a = abs(fract(g - 0.5) - 0.5) / max(w, vec2(1e-5));
    return (1.0 - min(min(a.x, a.y) / width, 1.0)) * (1.0 - smoothstep(0.12, 0.35, max(w.x, w.y)));
  }
  void main() {
    vec2 p = vWorld.xy;
    float soft = uArea.w;
    float inside = (1.0 - smoothstep(0.0, soft, p.x - uArea.y)) * (1.0 - smoothstep(0.0, soft, -uArea.x - p.x))
                 * (1.0 - smoothstep(0.0, soft, abs(p.y) - uArea.z));
    vec3 color = mix(uGround, uRoad, inside);
    float lines = max(grid(p, 5.0, 1.0) * 0.035, grid(p, 20.0, 1.2) * 0.08) * (0.25 + 0.75 * inside);
    lines *= 1.0 - smoothstep(uFog.x - 30.0, uFog.x + 30.0, vDepth);
    color = mix(color, uLine, lines);
    vec2 w = fwidth(p);
    float axis = max(1.0 - min(abs(p.y) / w.y / 1.2, 1.0), 1.0 - min(abs(p.x) / w.x / 1.2, 1.0));
    color = mix(color, uAxis, axis * 0.45 * inside);
    gl_FragColor = vec4(mix(color, uBackground, 1.0 - fogFade(vDepth)), 1.0);
    #include <colorspace_fragment>
  }`];

const STYLE_SHEET = `
.bev-panel { position: relative; height: calc(100dvh - 230px); min-height: 340px; overflow: hidden;
  border: 1px solid var(--line, #353b40); border-radius: 7px; background: ${THEME.background}; touch-action: none; }
.bev-panel.stale canvas { filter: grayscale(1) brightness(0.6); }
.bev-labels { position: absolute; top: 0; left: 0; pointer-events: none; }
.bev-labels .bev-tag { display: flex; align-items: center; gap: 4px; padding: 2px 6px 2px 5px; border-radius: 9px;
  font: 600 10px/13px ui-monospace, Consolas, monospace; color: #dde3ea; white-space: nowrap;
  background: rgba(10, 13, 18, 0.78); border: 1px solid rgba(255, 255, 255, 0.07); }
.bev-labels .bev-tag i { width: 6px; height: 6px; border-radius: 50%; flex: none; }
.bev-labels .bev-tag em { font-style: normal; color: #8f9aa8; }
.bev-labels.far .bev-tag em { display: none; }
.bev-labels.far .bev-tag { padding: 1px 5px 1px 4px; font-size: 9px; line-height: 11px; }
.bev-labels .bev-tag.gap { color: #fff; font-weight: 700; border-radius: 4px; }
.bev-labels .bev-tag.speed { font-weight: 700; border-radius: 4px; }
.bev-labels .bev-dist { font: 500 10px ui-monospace, Consolas, monospace; color: #566170; }
.bev-hud { position: absolute; left: 8px; right: 8px; bottom: 8px; display: flex; flex-wrap: wrap; gap: 6px; pointer-events: none; }
.bev-hud span { display: flex; align-items: center; gap: 6px; font: 600 11px/1 ui-monospace, Consolas, monospace; color: #c9d1db;
  padding: 6px 8px; border-radius: 5px; background: rgba(10, 13, 18, 0.8); border: 1px solid rgba(255, 255, 255, 0.07); }
.bev-hud span b { color: #6c7684; font-weight: 600; }
.bev-hud span em { font-style: normal; }
.bev-hud span i { width: 8px; height: 8px; border-radius: 50%; }
.bev-hud span.warn { color: ${HUD.amber}; }
.bev-hud span:empty { display: none; }
.bev-top { position: absolute; top: 8px; left: 8px; right: 8px; display: flex; flex-direction: column; align-items: flex-end;
  gap: 6px; pointer-events: none; }
.bev-tools { align-self: stretch; display: flex; flex-wrap: wrap; justify-content: space-between; gap: 6px; }
.bev-tools div { display: flex; flex-wrap: wrap; gap: 6px; pointer-events: auto; }
.bev-tools button { height: 32px; padding: 0 11px; border: 1px solid rgba(255, 255, 255, 0.14); border-radius: 6px;
  background: rgba(10, 13, 18, 0.82); color: #dde3ea; font: 650 12px system-ui, sans-serif; cursor: pointer; touch-action: manipulation; }
.bev-tools button:hover { background: rgba(38, 43, 47, 0.92); }
.bev-tools button[aria-pressed="false"] { color: #6c7684; border-style: dashed; }
.bev-card { width: 214px; padding: 8px 10px 9px; border-radius: 7px;
  background: rgba(10, 13, 18, 0.86); border: 1px solid rgba(255, 255, 255, 0.09);
  font: 600 11px/1.2 ui-monospace, Consolas, monospace; color: #c9d1db; }
.bev-card[hidden] { display: none; }
.bev-card-head { display: flex; align-items: center; justify-content: space-between; margin-bottom: 7px; }
.bev-card-head b { color: #8f9aa8; font-weight: 650; letter-spacing: 0.04em; }
.bev-badge { padding: 2px 7px; border-radius: 9px; font-weight: 700; color: #0a0d12; background: #566170; }
.bev-badge.armed { background: ${HUD.amber}; }
.bev-badge.go { background: ${HUD.green}; }
.bev-row { display: grid; grid-template-columns: 62px 1fr 44px; align-items: center; gap: 7px; margin-top: 6px; }
.bev-row span { color: #8f9aa8; }
.bev-row em { font-style: normal; text-align: right; color: #fff; }
.bev-meter { position: relative; height: 7px; border-radius: 4px; background: rgba(255, 255, 255, 0.1); overflow: hidden; }
.bev-meter i { position: absolute; top: 0; bottom: 0; left: 0; }
.bev-meter .zone { background: rgba(255, 176, 32, 0.22); }
.bev-meter .fill { border-radius: 4px; background: #c9d1db; }
.bev-meter .fill.go { background: ${HUD.green}; }
.bev-meter .mark { width: 2px; margin-left: -1px; background: #fff; }
.bev-spark { display: block; width: 100%; height: 34px; margin-top: 7px; border-radius: 4px; background: rgba(255, 255, 255, 0.04); }
.bev-spark .limit { stroke: rgba(255, 255, 255, 0.45); stroke-dasharray: 3 3; }
.bev-spark .gas { stroke: ${HUD.green}; fill: none; stroke-width: 1.6; }
.bev-spark .plan { stroke: #8f9aa8; fill: none; stroke-width: 1; stroke-dasharray: 2 2; }
.bev-spark line, .bev-spark polyline { vector-effect: non-scaling-stroke; }
.bev-card-foot { margin-top: 5px; display: flex; gap: 10px; color: #6c7684; font-size: 10px; }
.bev-card-foot i { display: inline-block; width: 10px; height: 2px; margin-right: 4px; vertical-align: middle; }
@media (max-width: 460px) {
  .bev-panel { height: calc(100dvh - 250px); }
  .bev-tools { gap: 4px; }
  .bev-tools div { gap: 4px; }
  .bev-tools button { height: 28px; padding: 0 6px; font-size: 11px; }
  .bev-hud span { padding: 5px 6px; font-size: 10px; }
  .bev-top { gap: 4px; }
  .bev-card { width: 184px; padding: 7px 8px 8px; }
  .bev-row { grid-template-columns: 56px 1fr 38px; gap: 5px; }
}`;

export class BevView {
  constructor(container) {
    this.container = container;
    if (!document.getElementById("bev-style")) {
      document.head.appendChild(Object.assign(document.createElement("style"), {id: "bev-style", textContent: STYLE_SHEET}));
    }
    this.renderer = new THREE.WebGLRenderer({antialias: true});
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, MAX_PIXEL_RATIO));
    SHARED.uPixel.value = this.renderer.getPixelRatio();
    // laid over the panel, so its size never feeds back into the panel's own
    Object.assign(this.renderer.domElement.style, {position: "absolute", top: "0", left: "0"});
    container.appendChild(this.renderer.domElement);
    this.labels = new CSS2DRenderer();
    this.labels.domElement.className = "bev-labels";
    container.appendChild(this.labels.domElement);
    this.hud = container.appendChild(el("div", "bev-hud"));
    this.chips = {};
    for (const name of ["mode", "speed", "plan", "lane", "lat", "lead", "model"]) this.chips[name] = this.hud.appendChild(el("span"));
    // the toolbar, and the departure card under it however many rows the toolbar wraps to
    const top = container.appendChild(el("div", "bev-top"));
    this.tools(top.appendChild(el("div", "bev-tools")));
    this.departureCard(top);

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(THEME.background);
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
    this.controls.maxPolarAngle = Math.PI * 0.47;           // not under the road
    this.controls.addEventListener("change", () => this.requestRender());

    this.layers = {};
    for (const name of ["path", "lanes", "edges", "lead", "curvature"]) this.scene.add(this.layers[name] = new THREE.Group());
    this.layers.lead.add(this.layers.lead_labels = new THREE.Group());
    this.layers.path.add(this.layers.path_labels = new THREE.Group());
    this.tags = [];
    this.makeParts();
    this.scene.add(this.staticParts());

    new ResizeObserver(() => this.resize()).observe(container);
    this.view("behind");
  }

  // View buttons on the left, layer buttons on the right.
  tools(bar) {
    const views = bar.appendChild(el("div")), layers = bar.appendChild(el("div"));
    for (const [name, title] of Object.entries(VIEWS)) {
      const button = views.appendChild(el("button", "", title));
      button.type = "button";
      button.addEventListener("click", () => this.view(name));
    }
    for (const [name, title] of Object.entries(LAYERS)) {
      const button = layers.appendChild(el("button", "", title));
      button.type = "button";
      button.setAttribute("aria-pressed", "true");
      button.addEventListener("click", () => {
        const on = button.getAttribute("aria-pressed") !== "true";
        button.setAttribute("aria-pressed", String(on));
        for (const group of LAYER_GROUPS[name]) this.layers[group].visible = on;
        this.requestRender();
      });
    }
  }

  // -- the meshes every frame writes into -----------------------------------------------------------
  makeParts() {
    const plane = new THREE.PlaneGeometry(1, 1), print = {iSize: 3, iStyle: 4};
    const put = (layer, part) => { this.layers[layer].add(part.object || part.mesh); return part; };
    this.lanes = put("lanes", new Batch("color", 4, overlayMaterial(), 2));
    this.curbs = put("edges", new Batch("color", 4, new THREE.MeshStandardMaterial({
      vertexColors: true, flatShading: true, roughness: 0.8, side: THREE.DoubleSide, transparent: true, depthWrite: false}), 1));
    this.corridorMaterial = shader(...CORRIDOR, {uColor: {value: new THREE.Color()}, uSlow: {value: ink(HUD.amber)},
                                                 uStop: {value: ink(HUD.red)}, uLength: {value: 1}, uWidth: {value: PATH_WIDTH},
                                                 uStrength: {value: 1}, uTint: {value: SLOW.tint}});
    this.path = put("path", new Batch("corridor", 3, this.corridorMaterial, 3));
    this.speedTag = this.tag("path_labels", "speed");
    this.speedTag.center.set(0, 0);             // hung below and right of its point: clear of the lead's label
    this.arcs = put("curvature", new Batch("color", 4, overlayMaterial(), 6));

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
      lines: put("lead", new Batch("color", 4, overlayMaterial(), 5)),
      tag: this.tag("lead_labels", "", true),
      gap: this.tag("lead_labels", "gap"),
    };
  }

  // A label in group, hidden until place()d: a pill with a dot and a kind (dotted), or plain text.
  tag(group, variant = "", dotted = false) {
    const element = el("div", `bev-tag ${variant}`.trim());
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

  // -- the parts that do not change: the ground, distances, ego car ------------------------------------
  staticParts() {
    const group = new THREE.Group();
    const ground = new THREE.Mesh(new THREE.PlaneGeometry(1600, 1600), shader(...GROUND, {
      uGround: {value: new THREE.Color(THEME.ground)}, uRoad: {value: new THREE.Color(THEME.road)},
      uLine: {value: new THREE.Color(THEME.gridLine)}, uAxis: {value: new THREE.Color(THEME.axis)},
      uArea: {value: new THREE.Vector4(GRID.behind, GRID.ahead, GRID.side, 4)}}, {transparent: false, depthWrite: true}));
    ground.position.set(GRID.ahead / 2, 0, -0.01);
    group.add(ground);
    this.distances = [];
    for (let x = GRID.major; x <= GRID.ahead; x += GRID.major) {
      const label = new CSS2DObject(el("div", "bev-dist", x + " m"));
      label.position.set(x, -GRID.side - 1.5, 0);
      group.add(label);
      this.distances.push(label);
    }
    // the ego car, its front bumper at x = 0; its turn signals and brake lights, and their glow on the
    // road, follow the car's own (drawEgo())
    const [l, w] = EGO, x = -l / 2;
    this.ego = makeEgo(this.renderer);
    this.ego.position.x = x;
    this.egoGlow = new Instances(new THREE.PlaneGeometry(1, 1), shader(...FOOTPRINT), {order: 4, extra: {iSize: 3, iStyle: 4}});
    const shadow = new Instances(new THREE.PlaneGeometry(1, 1), shader(...FOOTPRINT), {order: 4, extra: {iSize: 3, iStyle: 4}});
    shadow.begin(1);
    shadow.add(x, 0, 0.006, 0, l + 1.2, w + 1.2, 1, BLACK, [l * 0.92, w * 0.9, 0.6], [0.5, 0, 0, 0.5]);
    shadow.end();
    // the halo takes the HUD's state colour (drawEgo())
    this.halo = new Instances(new THREE.PlaneGeometry(1, 1), shader(...FOOTPRINT), {order: 4, extra: {iSize: 3, iStyle: 4}});
    group.add(this.ego, shadow.object, this.halo.object, this.egoGlow.object);
    this.drawEgo(null);
    return group;
  }

  // -- camera -------------------------------------------------------------------------------------------
  view(name) {
    if (name === "top") {
      this.controls.target.set(45, 0, 0);
      this.camera.position.set(44.99, 0, 120);     // a hair behind, so forward is up on the screen
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

  // The scene drawn at the next display frame; frames that come in between are skipped.
  update(scene) {
    this.next = scene;
    this.requestRender();
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
      // fog starts beyond what the camera looks at, however far away it is
      const distance = this.camera.position.distanceTo(this.controls.target);
      this.scene.fog.near = distance + 40;
      this.scene.fog.far = distance + 160;
      SHARED.uFog.value.set(this.scene.fog.near, this.scene.fog.far);
      this.fadeLabels(distance);
      this.labels.domElement.classList.toggle("far", distance > 90);     // from far away, no kinds
      this.renderer.render(this.scene, this.camera);
      this.labels.render(this.scene, this.camera);
    });
  }

  // Labels well beyond what the camera looks at fade, so the near ones read.
  fadeLabels(distance) {
    for (const label of [...this.tags, ...this.distances]) {
      const beyond = this.camera.position.distanceTo(label.position) - distance;
      const opacity = (beyond < 40 ? 1 : Math.max(0.25, 1 - (beyond - 40) / 60)).toFixed(2);
      if (label.opacity !== opacity) label.element.style.opacity = label.opacity = opacity;
    }
  }

  // -- one frame ------------------------------------------------------------------------------------
  draw(scene) {
    this.plan = planSummary(scene.path, scene.planNow);
    this.container.classList.toggle("stale", scene.stale);
    this.drawLanes(scene.lanes);
    this.drawEdges(scene.edges);
    this.drawPath(scene);
    this.drawCurvature(scene);
    this.drawLead(scene);
    this.drawEgo(scene);
    this.drawHud(scene);
    this.drawDeparture(scene);
  }

  // Lane lines as bands, as sure as the HUD draws them; the ego lane faintly filled between its
  // lines while both are sure. Unlike the HUD, in laneless mode too (with the edges): the model
  // still sees them, and the 차선 button hides them.
  drawLanes(lanes) {
    const b = this.lanes, color = ink(THEME.lane);
    b.begin();
    if (lanes.length === 4 && lanes[1].probability >= 0.5 && lanes[2].probability >= 0.5) egoLane(b, lanes[1].points, lanes[2].points);
    for (const lane of lanes) {
      if (lane.probability < LANE_MIN_PROBABILITY) continue;
      band(b, lane.points, LANE_WIDTH, 0.03, color, 0.25 + 0.65 * lane.probability);
    }
    b.end();
  }

  // Road edges as low red curbs, as sure as the model is of them (1 - std).
  drawEdges(edges) {
    const b = this.curbs, color = ink(HUD.red);
    b.begin();
    for (const edge of edges) {
      const confidence = clamp(1 - edge.std, 0, 1);
      if (confidence >= EDGE_MIN_CONFIDENCE) curb(b, edge.points, CURB.width, CURB.height, color, 0.7 * confidence);
    }
    b.end();
  }

  // The plan as a corridor in the HUD's path colour, tinted where the model plans to slow down.
  drawPath(scene) {
    const path = scene.path;
    this.path.begin();
    this.speedTag.visible = false;
    if (path.length > 1) {
      const [hex, strength] = pathStyle(scene.state), now = Number.isFinite(scene.planNow) ? scene.planNow : 0;
      const slow = path.map(([, , speed]) => clamp((now - speed) / Math.max(now, SLOW.floor), 0, 1));
      const uniforms = this.corridorMaterial.uniforms;
      uniforms.uColor.value.copy(ink(hex));
      uniforms.uStrength.value = strength;
      uniforms.uLength.value = Math.max(corridor(this.path, path, PATH_WIDTH, 0.025, slow), 1);
      // the plan's slowest point (or STOP, or where it speeds up to) beside the path, but not with a
      // lead in view: in perspective it lands on the lead's label wherever it is, and the chip says it
      const plan = this.plan;
      if (plan && !(scene.lead && this.layers.lead.visible)) {
        const [nx, ny] = normalAt(path, plan.i), beside = PATH_WIDTH / 2 + 0.4;
        this.place(this.speedTag, path[plan.i][0] - nx * beside, path[plan.i][1] - ny * beside, 0, plan.text, "", plan.color);
      }
    }
    this.path.end();
  }

  // Arcs of the curvature controlsd asks for (white) and the one it measures (cyan), from the bumper
  // as far as the car goes in ARC.seconds, each ended by a tick across it: how far apart the ticks
  // are is where tracking that curvature would leave the car. Their curvature is + to the right; the
  // BEV's y is to the left.
  drawCurvature(scene) {
    const b = this.arcs, curvature = scene.curvature, speed = scene.speedKph / 3.6;
    b.begin();
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

  // The model's lead as a car turned along the path, ringed in the HUD's risk colour, with the gap
  // from our bumper to its rear and an arrow of its speed relative to ours.
  drawLead(scene) {
    const lead = scene.lead, parts = this.lead, model = carModel();
    // brake lights with hysteresis, so they do not flicker round the threshold
    const braking = this.braking = !!lead && Number.isFinite(lead.accel) && lead.accel <= (this.braking ? BRAKE.off : BRAKE.on);
    for (const part of [parts.body, parts.lamps]) part.begin(lead ? 1 : 0);
    parts.brake.begin(braking ? 1 : 0);
    parts.wheels.begin(lead ? model.wheels.length : 0);
    parts.prints.begin(lead ? (braking ? 3 : 2) : 0);
    parts.lines.begin();
    parts.tag.visible = parts.gap.visible = false;
    if (lead) {
      const [l, w, h] = LEAD, scale = fit(LEAD), yaw = headingAt(scene.path, lead.x);
      const c = Math.cos(yaw), s = Math.sin(yaw), x = lead.x + c * l / 2, y = lead.y + s * l / 2;
      const hex = riskColor(lead), color = ink(hex);
      parts.body.add(x, y, 0, yaw, ...scale, ink(THEME.model).clone().lerp(color, THEME.modelTint));
      parts.lamps.add(x, y, 0, yaw, ...scale, null);
      if (braking) {                            // and their glow on the road behind it
        parts.brake.add(x, y, 0, yaw, ...scale, null);
        parts.prints.add(lead.x - c * 0.9, lead.y - s * 0.9, 0.012, yaw, 3.4, w + 2.4, 1, ink(HUD.red), [0.4, w * 0.8, 1.1], [0, 0, 0, 0.55]);
      }
      for (const [wx, wy, wz, r, width] of model.wheels) {
        const px = wx * scale[0], py = wy * scale[1];
        parts.wheels.add(x + c * px - s * py, y + s * px + c * py, wz * scale[2], yaw, r * scale[2], width, r * scale[2]);
      }
      parts.prints.add(x, y, 0.006, yaw, l + 1.2, w + 1.2, 1, BLACK, [l * 0.92, w * 0.9, 0.6], [0.5, 0, 0, 0.5]);
      parts.prints.add(x, y, 0.016, yaw, l + 1.6, w + 1.6, 1, color, [l + 0.3, w + 0.3, 0.5], [0.12, 0.85, 0, 0.25]);
      if (lead.x > 1) {                         // a dimension line, ticked at both ends
        band(parts.lines, [[0.3, 0], [lead.x - 0.3, lead.y]], 0.05, 0.02, color, 0.6);
        band(parts.lines, [[0.3, -0.45], [0.3, 0.45]], 0.06, 0.02, color, 0.8);
        band(parts.lines, [[lead.x - 0.3, lead.y - 0.45], [lead.x - 0.3, lead.y + 0.45]], 0.06, 0.02, color, 0.8);
        this.place(parts.gap, lead.x / 2, lead.y / 2, 0.05, `${lead.gap.toFixed(1)} m`, "", hex);
      }
      if (lead.relative !== null && Math.abs(lead.relative) >= ARROW.slowest) {
        arrow(parts.lines, x, y, x + c * lead.relative * ARROW.seconds, y + s * lead.relative * ARROW.seconds, color);
      }
      this.place(parts.tag, x, y, h + 0.3, `${Math.round(lead.speed * 3.6)} km/h`, braking ? "BRAKE" : "LEAD", braking ? HUD.red : hex);
    }
    for (const part of [parts.body, parts.lamps, parts.brake, parts.wheels, parts.prints]) part.end();
    parts.lines.end();
  }

  // The ego car's halo in the HUD's state colour: green or blue while steering, turning amber as the
  // output nears its limit (the HUD's path does that; here amber in the path means slowing down),
  // gray while engaged. Then its lamps: turn signals and hazards, and brake lights.
  drawEgo(scene) {
    const [l, w] = EGO, state = scene && scene.state, hex = state && stateColor(state);
    const strain = state && state.steering ? clamp((Math.abs(state.output) - 0.7) / 0.3, 0, 1) : 0;
    this.halo.begin(1);
    this.halo.add(-l / 2, 0, 0.008, 0, l + 2.4, w + 2.4, 1, ink(hex || THEME.halo).clone().lerp(ink(HUD.amber), strain),
                  [l + 0.4, w + 0.4, 1.2], hex ? [0.06, 0.7, 0, 0.3] : [0.04, 0.5, 0, 0.18]);
    this.halo.end();
    // turn signals and hazards (both), lit for the first half of each BLINK.period; brake lights
    // (pedal pressed, or AUTO HOLD)
    const blinkers = scene && scene.blinkers, lit = (performance.now() / 1000) % BLINK.period < BLINK.period * BLINK.duty;
    const sides = [!!(blinkers && blinkers.left && lit), !!(blinkers && blinkers.right && lit)];
    const braking = !!scene && !!scene.brakeLights;
    this.ego.setLamps({left: sides[0], right: sides[1], brake: braking});
    // and their glow on the road: at the corners on the flashing side, behind the car when braking
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

  // The chips along the bottom: mode, speed, lane width, the gap, and how old the model frame is.
  drawHud(scene) {
    const lead = scene.lead;
    const relative = lead && lead.relative !== null && lead.relative * 3.6 <= -3 ? ` ${Math.round(lead.relative * 3.6)} km/h` : "";
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
    this.chip("model", scene.stale ? `<b>MODEL</b>${Number.isFinite(scene.modelAge) ? scene.modelAge.toFixed(1) + " s" : "없음"}` : "", true);
  }

  // -- standing: what the departure alert decides from ------------------------------------------------
  // A card while the car stands in D, and while an alert shows: the 2 s gas press probability against the
  // 0.3 that fires it, both inputs over the last DEPART.history s, and the plan's x at 10 s against
  // the band that arms the green light alert (±5 m for 1.5 s) and the 10 m that fires it.
  departureCard(parent) {
    const card = this.card = parent.appendChild(el("div", "bev-card"));
    card.hidden = true;
    const head = card.appendChild(el("div", "bev-card-head"));
    head.appendChild(el("b", "", "DEPART"));
    this.badge = head.appendChild(el("span", "bev-badge"));
    const row = (title, mark, zone = 0) => {
      const line = card.appendChild(el("div", "bev-row")), meter = el("div", "bev-meter");
      if (zone) meter.appendChild(el("i", "zone")).style.width = `${zone}%`;
      const fill = meter.appendChild(el("i", "fill"));
      meter.appendChild(el("i", "mark")).style.left = `${mark}%`;
      const value = el("em");
      line.append(el("span", "", title), meter, value);
      return {fill, value};
    };
    this.gasRow = row("GAS 2s", DEPART.gas * 100);
    const svg = (tag, attributes) => {
      const node = document.createElementNS("http://www.w3.org/2000/svg", tag);
      for (const [key, value] of Object.entries(attributes)) node.setAttribute(key, value);
      return node;
    };
    const spark = card.appendChild(svg("svg", {class: "bev-spark", viewBox: "0 0 100 30", preserveAspectRatio: "none"}));
    spark.appendChild(svg("line", {class: "limit", x1: 0, x2: 100, y1: 30 * (1 - DEPART.gas), y2: 30 * (1 - DEPART.gas)}));
    this.planLine = spark.appendChild(svg("polyline", {class: "plan"}));
    this.gasLine = spark.appendChild(svg("polyline", {class: "gas"}));
    this.planRow = row("PLAN 10s", DEPART.open / DEPART.planMax * 100, DEPART.arm / DEPART.planMax * 100);
    const foot = card.appendChild(el("div", "bev-card-foot"));
    for (const [title, color] of [["gas 2s", HUD.green], ["plan 10s", "#8f9aa8"], [`${DEPART.history} s`, null]]) {
      const item = foot.appendChild(el("span", "", title));
      if (color) item.insertBefore(el("i"), item.firstChild).style.background = color;
    }
  }

  drawDeparture(scene) {
    const card = this.card, departure = scene.departure, speed = scene.speedKph / 3.6;
    const standing = departure.drive && Number.isFinite(speed) && speed <= (card.hidden ? DEPART.stopped : DEPART.moving);
    card.hidden = !(standing || departure.alert !== 0);
    if (card.hidden) return;
    const [text, tone] = departure.alert === 2 ? ["GREEN", "go"] : departure.alert === 1 ? ["LEAD GO", "go"]
      : departure.armed ? ["ARMED", "armed"] : ["WAIT", ""];
    if (this.badge.textContent !== text) {
      this.badge.textContent = text;
      this.badge.className = `bev-badge ${tone}`.trim();
    }
    const gas = departure.gas.length > 1 ? departure.gas[1] : NaN, plan = departure.plan10;
    meter(this.gasRow, gas, 1, gas >= DEPART.gas, Number.isFinite(gas) ? gas.toFixed(2) : "–");
    meter(this.planRow, plan, DEPART.planMax, plan > DEPART.open, Number.isFinite(plan) ? `${plan.toFixed(1)} m` : "–");
    const history = departure.history, end = history.length ? history[history.length - 1][0] : 0;
    const line = (k, scale) => history.map((row) => `${((row[0] - end) / DEPART.history * 100 + 100).toFixed(1)},`
                                               + `${(30 * (1 - clamp(row[k] / scale, 0, 1))).toFixed(1)}`).join(" ");
    this.gasLine.setAttribute("points", line(1, 1));
    this.planLine.setAttribute("points", line(2, DEPART.planMax));
  }

  chip(name, html, warn = false) {
    const span = this.chips[name];
    if (span.dataset.html === html) return;
    span.dataset.html = html;
    span.innerHTML = html;
    span.classList.toggle("warn", warn);
  }
}

// -- the HUD's rules --------------------------------------------------------------------------------
// Where the plan slows down most ahead (STOP where it stops), or, speeding up instead, what it
// reaches at its end: {i (path index), text, color}, or null when it does neither by SLOW.labelKph.
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

// hud_draw.h state_color(): the colour of steering, null when not engaged.
function stateColor(state) {
  if (state.steering) return state.laneless ? HUD.blue : HUD.green;
  return state.engaged ? HUD.gray : null;
}

function modeText(state) {
  if (state.active) return state.laneless ? "LANELESS" : "LANE";
  return state.engaged || state.enabled ? "READY" : "OFF";
}

// path_color(): [colour, strength]: white faint when not engaged, gray when not steering, green or
// blue while steering. (Its amber near the output limit is on the ego halo here, drawEgo().)
function pathStyle(state) {
  if (!state.engaged) return [HUD.white, 0.6];
  if (!state.steering) return [HUD.gray, 0.85];
  return [state.laneless ? HUD.blue : HUD.green, 1];
}

// lead_risk(): near or closing fast towards 1; white, amber, then red. (The HUD takes the SCC
// radar's gap first; this car's never has a lead, so it is left out here.)
function riskColor(lead) {
  const relative = lead.relative ?? 0;
  const risk = clamp(clamp(1 - lead.gap / RISK.distance, 0, 1) + clamp(-relative / RISK.closing, 0, 1), 0, 1);
  if (risk > 0.75) return HUD.red;
  return "#" + ink(HUD.white).clone().lerp(ink(HUD.amber), risk / 0.75).getHexString();
}

// A departure card meter row: value against full, filled green when it fires.
function meter(row, value, full, go, text) {
  row.fill.style.width = `${(Number.isFinite(value) ? clamp(value / full, 0, 1) : 0) * 100}%`;
  row.fill.classList.toggle("go", go);
  if (row.value.textContent !== text) row.value.textContent = text;
}

// -- writing geometry -------------------------------------------------------------------------------------
// A band of width along points [[x, y], ...] at height z.
function band(b, points, width, z, color, alpha) {
  const n = points.length, h = width / 2;
  if (n < 2) return;
  const base = b.reserve(2 * n, 6 * (n - 1));
  for (let i = 0; i < n; i++) {
    const [px, py] = points[i], [nx, ny] = normalAt(points, i);
    b.put(px + nx * h, py + ny * h, z, color.r, color.g, color.b, alpha);
    b.put(px - nx * h, py - ny * h, z, color.r, color.g, color.b, alpha);
    if (i) { const k = base + 2 * i; b.tri(k - 2, k - 1, k); b.tri(k - 1, k + 1, k); }
  }
}

// The lane between left and right faintly filled, fading ahead.
function egoLane(b, left, right) {
  if (left.length < 2 || right.length < 2) return;
  const x0 = Math.max(left[0][0], right[0][0], 0), x1 = Math.min(left[left.length - 1][0], right[right.length - 1][0]);
  if (x1 - x0 < 2) return;
  const color = ink(THEME.egoLane);
  let base = -1;
  for (let x = x0; ; x = Math.min(x + 2, x1)) {
    const yl = lateralAt(left, x), yr = lateralAt(right, x);
    if (yl === null || yr === null) break;
    const alpha = 0.06 * (1 - smoothstep(25, 90, x));
    const v = b.reserve(2, 6);
    b.put(x, yl - 0.1, 0.012, color.r, color.g, color.b, alpha);
    b.put(x, yr + 0.1, 0.012, color.r, color.g, color.b, alpha);
    if (base >= 0) { b.tri(base, base + 1, v); b.tri(base + 1, v + 1, v); }
    base = v;
    if (x >= x1) break;
  }
}

// An arrow from (x0, y0) to (x1, y1) on the road.
function arrow(b, x0, y0, x1, y1, color) {
  const length = Math.hypot(x1 - x0, y1 - y0), ux = (x1 - x0) / length, uy = (y1 - y0) / length, head = Math.min(0.6, length * 0.4);
  band(b, [[x0, y0], [x1 - ux * head, y1 - uy * head]], 0.1, 0.07, color, 0.95);
  const base = b.reserve(3, 3);
  b.put(x1, y1, 0.07, color.r, color.g, color.b, 0.95);
  b.put(x1 - ux * head - uy * head * 0.55, y1 - uy * head + ux * head * 0.55, 0.07, color.r, color.g, color.b, 0.95);
  b.put(x1 - ux * head + uy * head * 0.55, y1 - uy * head - ux * head * 0.55, 0.07, color.r, color.g, color.b, 0.95);
  b.tri(base, base + 1, base + 2);
}

// A band along points for the path: across, along and slow[i] (the planned slowdown); its length.
function corridor(b, points, width, z, slow) {
  const n = points.length, h = width / 2, base = b.reserve(2 * n, 6 * (n - 1));
  let along = 0;
  for (let i = 0; i < n; i++) {
    const [px, py] = points[i], [nx, ny] = normalAt(points, i);
    if (i) along += Math.hypot(px - points[i - 1][0], py - points[i - 1][1]);
    b.put(px + nx * h, py + ny * h, z, 0, along, slow[i]);
    b.put(px - nx * h, py - ny * h, z, 1, along, slow[i]);
    if (i) { const k = base + 2 * i; b.tri(k - 2, k - 1, k); b.tri(k - 1, k + 1, k); }
  }
  return along;
}

// [[x, y], ...] of an arc from the origin heading along x, curving k (1/m, + to the left) for length m.
function arc(k, length, n = 24) {
  return Array.from({length: n + 1}, (_, i) => {
    const s = length * i / n;
    return Math.abs(k) < 1e-6 ? [s, 0] : [Math.sin(k * s) / k, (1 - Math.cos(k * s)) / k];
  });
}

// A road edge as a low curb: its top and both faces, lit.
function curb(b, points, width, height, color, alpha) {
  const n = points.length, h = width / 2;
  if (n < 2) return;
  const base = b.reserve(4 * n, 18 * (n - 1));
  points.forEach(([px, py], i) => {
    const [nx, ny] = normalAt(points, i);
    for (const [s, z] of [[1, 0], [1, height], [-1, height], [-1, 0]]) {
      b.put(px + s * nx * h, py + s * ny * h, z, color.r, color.g, color.b, alpha);
    }
    if (i) {
      const a = base + 4 * (i - 1), c = base + 4 * i;
      for (const [p, q] of [[0, 1], [1, 2], [2, 3]]) { b.tri(a + p, a + q, c + p); b.tri(a + q, c + q, c + p); }
    }
  });
}

// The y of a polyline at x, null where it does not reach.
function lateralAt(points, x) {
  for (let i = 1; i < points.length; i++) {
    const [x0, y0] = points[i - 1], [x1, y1] = points[i];
    if ((x0 <= x && x <= x1) || (x1 <= x && x <= x0)) return x1 === x0 ? y0 : y0 + (y1 - y0) * (x - x0) / (x1 - x0);
  }
  return null;
}

// The heading of a polyline where it passes x (its last piece beyond its end), 0 for a path too
// short to say (standing still).
function headingAt(points, x) {
  const n = points.length;
  if (n < 2 || points[n - 1][0] - points[0][0] < 5) return 0;
  let i = 1;
  while (i < n - 1 && points[i][0] < x) i++;
  return Math.atan2(points[i][1] - points[i - 1][1], points[i][0] - points[i - 1][0]);
}

// The unit normal, to the left, of a polyline at its point i.
function normalAt(points, i) {
  const [ax, ay] = points[Math.max(0, i - 1)], [bx, by] = points[Math.min(points.length - 1, i + 1)];
  const length = Math.hypot(bx - ax, by - ay) || 1;
  return [-(by - ay) / length, (bx - ax) / length];
}

// The BEV tab: the view and its stream, made once; start() and stop() follow the tab and the
// page's visibility. onStatus({live, text}) says how the stream is doing.
export function createBev(onStatus) {
  const element = el("div", "bev-panel");
  const view = new BevView(element);
  const stream = new BevStream({onScene: (scene) => view.update(scene), onStatus});
  return {element, start: () => stream.start(), stop: () => stream.stop()};
}
