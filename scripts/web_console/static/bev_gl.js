// BEV의 상태 없는 GPU 도구(bev.js에서 뗐다): 색 캐시, 매 프레임 다시 쓰는 삼각형 묶음(Batch)과 인스턴스 묶음
// (Instances), 안개·픽셀 비율·배경을 공유하는 셰이더 재질과 그 원본(풋프린트, 경로 띠, 바닥). 장면이나 스트림
// 상태는 갖지 않는다.
import * as THREE from "three";

// -- 색 --------------------------------------------------------------------------------------------
export const BLACK = new THREE.Color(0, 0, 0);
const inks = new Map();
export const ink = (hex) => inks.get(hex) || inks.set(hex, new THREE.Color(hex)).get(hex);

// -- 매 프레임 다시 쓰는 지오메트리 ---------------------------------------------------------------------
// 프레임 사이에 유지하는 버퍼 속 삼각형: position과 값 size개짜리 속성 하나 더(알파가 있는 색, uv, ...).
// 프레임은 begin(road)하고, 쓰고, end()한다. 그 프레임의 정점은 노면 road(bev_road.js) 위에 놓인다: put()의 z는
// 그 x의 노면에서 잰 높이다. end()가 새로 쓴 부분을 보낸다.
export class Batch {
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

  begin(road) {
    this.v = this.n = 0;
    this.road = road;
  }

  // 정점 vertices개와 인덱스 indices개를 더 넣을 자리. 첫 새 정점의 인덱스를 돌려준다.
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
    this.positions[3 * v] = x; this.positions[3 * v + 1] = y; this.positions[3 * v + 2] = z + this.road.at(x);
    values[o] = a;
    if (this.size > 1) values[o + 1] = b;
    if (this.size > 2) values[o + 2] = c;
    if (this.size > 3) values[o + 3] = d;
  }

  tri(a, b, c) { this.indices[this.n++] = a; this.indices[this.n++] = b; this.indices[this.n++] = c; }

  end() {
    const geometry = this.geometry;
    if (this.grown) {                       // 새 버퍼: 다음 렌더에서 GPU에 다시 만든다
      geometry.dispose();
      geometry.setAttribute("position", new THREE.BufferAttribute(this.positions, 3).setUsage(THREE.DynamicDrawUsage));
      geometry.setAttribute(this.name, new THREE.BufferAttribute(this.values, this.size).setUsage(THREE.DynamicDrawUsage));
      geometry.setIndex(new THREE.BufferAttribute(this.indices, 1).setUsage(THREE.DynamicDrawUsage));
      this.grown = false;
    } else if (this.n) {                    // 같은 버퍼: 이 프레임이 쓴 부분만
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

// 지오메트리 하나의 인스턴스를 프레임에 있는 만큼 한 번의 드로로 그린다: 각각 x, y, z에 놓고 yaw만큼 돌리고
// (addPitched는 그다음 앞을 pitch만큼 든다) sx, sy, sz만큼 늘이며, 제 색(colors)과 제 속성(extra, name: size)을
// 갖는다. 프레임에 메시가 담을 수 있는 것보다 많으면 메시를 더 크게 다시 만든다.
export class Instances {
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

  add(x, y, z, yaw, ...rest) {
    this.addPitched(x, y, z, yaw, 0, ...rest);
  }

  // 열은 늘인 앞(c·cp, s·cp, sp), 왼쪽(-s, c, 0), 위(-c·sp, -s·sp, cp) 축이다: 오르막에 놓인 차.
  addPitched(x, y, z, yaw, pitch, sx, sy, sz, color, ...extra) {
    const i = this.count++, m = this.mesh.instanceMatrix.array, o = 16 * i, c = Math.cos(yaw), s = Math.sin(yaw);
    const cp = Math.cos(pitch), sp = Math.sin(pitch);
    m[o] = c * cp * sx; m[o + 1] = s * cp * sx; m[o + 2] = sp * sx; m[o + 3] = 0;
    m[o + 4] = -s * sy; m[o + 5] = c * sy; m[o + 6] = 0; m[o + 7] = 0;
    m[o + 8] = -c * sp * sz; m[o + 9] = -s * sp * sz; m[o + 10] = cp * sz; m[o + 11] = 0;
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

// -- 재질 -------------------------------------------------------------------------------------------
// 공유 유니폼, 렌더마다 한 번 정한다: 안개(near, far, 뷰 깊이로)와 픽셀 비율. 배경색은 BevView가 테마로 정한다.
export const SHARED = {uFog: {value: new THREE.Vector2(70, 200)}, uPixel: {value: 1}, uBackground: {value: new THREE.Color()}};
const FOG_FADE = "float fogFade(float depth) { return 1.0 - smoothstep(uFog.x, uFog.y, depth); }";

export function shader(vertex, fragment, uniforms = {}, options = {}) {
  return new THREE.ShaderMaterial({
    uniforms: {...SHARED, ...uniforms}, transparent: true, depthWrite: false, ...options,
    vertexShader: vertex,
    fragmentShader: `uniform vec2 uFog; uniform float uPixel; uniform vec3 uBackground;\n${FOG_FADE}\n${fragment}`,
  });
}

// 조명을 받는 모델, 플랫 셰이딩: 제 색이고, 도색 부분에는 인스턴스 색.
export function modelMaterial() {
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

// 조명 없음, 알파가 있는 정점 색: 차선선, 자차 차로, 간격과 화살표.
// 도로 위에 겹쳐 그리는 띠(차선, 곡률 호, 앞차 선)가 같이 쓰는 재질. 그리는 순서는 메시마다
// 정한다(Batch의 order).
export const OVERLAY = new THREE.MeshBasicMaterial({vertexColors: true, transparent: true, depthWrite: false,
                                            side: THREE.DoubleSide, toneMapped: false});

// 도로 위 풋프린트, 모서리가 둥근 길이 x 폭 직사각형 둘레를 미터로: iStyle은 채움, 윤곽선,
// 모서리 괄호만(1)과 글로, iSize는 길이, 폭과 글로가 사라지는 여백이다. 검은색이면 부드러운 접지 그림자다.
export const FOOTPRINT = [`
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

// 계획 경로를 HUD 리본만큼 넓은 띠로: HUD 경로 색의 밝은 가장자리, 4 m마다 셰브런, 앞으로 갈수록 흐려진다.
// uStrength는 HUD가 조향하지 않을 때처럼 띠를 흐리게 한다. 안쪽은 모델이 감속을 계획한 곳에서 호박색, 이어
// 빨간색이 된다. corridor는 폭 방향 위치(0..1), 진행 거리(미터)와 그곳의 계획 감속(0..1, SLOW)이다.
export const CORRIDOR = [`
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

// 바닥: BEV 범위는 한 톤 밝게, 5 m와 20 m 격자선은 빽빽해져 모아레가 되기 전에 흐려지고, 자차 축, 모두
// 멀리서 배경으로 사라진다.
export const GROUND = [`
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
