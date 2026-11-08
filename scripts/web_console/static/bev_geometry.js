// BEV의 상태 없는 지오메트리 쓰기(bev.js에서 뗐다): 폴리라인을 따르는 띠, 경로 띠, 연석, 화살표, 자차 차로를
// Batch에 넣고, 폴리라인을 읽는다(x에서의 y와 방향, 길이, 법선). 원호의 점도 만든다. 높이 z는 Batch가 그 프레임에
// 받은 노면(bev_road.js)에서 잰 것이다. 띠와 연석은 노면을 따라 휘도록 점 사이를 STEP m 안으로 촘촘히 한다.
import {smoothstep} from "./bev_math.js";

const STEP = 2;

// -- 지오메트리 쓰기 --------------------------------------------------------------------------------------
// points [[x, y], ...]를 따라 높이 z에 놓는 폭 width의 띠.
export function band(b, points, width, z, color, alpha) {
  ribbon(b, densify(points), width / 2, (i, side, x, y) => b.put(x, y, z, color.r, color.g, color.b, alpha));
}

// left와 right 사이 차로를 color로 옅게 채우고, 앞으로 갈수록 흐려진다.
export function egoLane(b, left, right, color) {
  if (left.length < 2 || right.length < 2) return;
  const x0 = Math.max(left[0][0], right[0][0], 0), x1 = Math.min(left[left.length - 1][0], right[right.length - 1][0]);
  if (x1 - x0 < 2) return;
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

// 도로 위 (x0, y0)에서 (x1, y1)까지의 화살표.
export function arrow(b, x0, y0, x1, y1, color) {
  const length = Math.hypot(x1 - x0, y1 - y0), ux = (x1 - x0) / length, uy = (y1 - y0) / length, head = Math.min(0.6, length * 0.4);
  band(b, [[x0, y0], [x1 - ux * head, y1 - uy * head]], 0.1, 0.07, color, 0.95);
  const base = b.reserve(3, 3);
  b.put(x1, y1, 0.07, color.r, color.g, color.b, 0.95);
  b.put(x1 - ux * head - uy * head * 0.55, y1 - uy * head + ux * head * 0.55, 0.07, color.r, color.g, color.b, 0.95);
  b.put(x1 - ux * head + uy * head * 0.55, y1 - uy * head - ux * head * 0.55, 0.07, color.r, color.g, color.b, 0.95);
  b.tri(base, base + 1, base + 2);
}

// 경로용으로 points를 따르는 띠: 폭 방향 위치, 진행 거리와 slow[i](계획 감속). 그 길이를 돌려준다.
export function corridor(b, points, width, z, slow) {
  const fine = densify(points.map(([x, y], i) => [x, y, slow[i]]));
  const along = arcLengths(fine);
  ribbon(b, fine, width / 2, (i, side, x, y) => b.put(x, y, z, side, along[i], fine[i][2]));
  return along.length ? along[along.length - 1] : 0;
}

// 원점에서 x 방향으로 출발해 곡률 k(1/m, 왼쪽이 +)로 length m만큼 가는 원호의 [[x, y], ...].
export function arc(k, length, n = 24) {
  return Array.from({length: n + 1}, (_, i) => {
    const s = length * i / n;
    return Math.abs(k) < 1e-6 ? [s, 0] : [Math.sin(k * s) / k, (1 - Math.cos(k * s)) / k];
  });
}

// 도로 경계를 낮은 연석으로: 윗면과 양 옆면, 조명을 받는다.
export function curb(b, line, width, height, color, alpha) {
  const points = densify(line), n = points.length, h = width / 2;
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

// 폴리라인의 x에서 y, 닿지 않는 곳에서는 null.
function lateralAt(points, x) {
  for (let i = 1; i < points.length; i++) {
    const [x0, y0] = points[i - 1], [x1, y1] = points[i];
    if ((x0 <= x && x <= x1) || (x1 <= x && x <= x0)) return x1 === x0 ? y0 : y0 + (y1 - y0) * (x - x0) / (x1 - x0);
  }
  return null;
}

// 폴리라인이 x를 지나는 곳의 방향(끝 너머는 마지막 조각), 판단하기에 너무 짧은 경로(정차 중)는 0.
export function headingAt(points, x) {
  const n = points.length;
  if (n < 2 || points[n - 1][0] - points[0][0] < 5) return 0;
  let i = 1;
  while (i < n - 1 && points[i][0] < x) i++;
  return Math.atan2(points[i][1] - points[i - 1][1], points[i][0] - points[i - 1][0]);
}

// 폴리라인을 따라 반폭 h의 띠: 점마다 왼쪽(side 0)·오른쪽(1) 정점을 vertex(i, side, x, y)가 넣고, 이웃한 두 점
// 사이를 삼각형 둘로 잇는다(band, corridor). 점이 둘보다 적으면 넣지 않는다.
function ribbon(b, points, h, vertex) {
  const n = points.length;
  if (n < 2) return;
  const base = b.reserve(2 * n, 6 * (n - 1));
  for (let i = 0; i < n; i++) {
    const [px, py] = points[i], [nx, ny] = normalAt(points, i);
    vertex(i, 0, px + nx * h, py + ny * h);
    vertex(i, 1, px - nx * h, py - ny * h);
    if (i) { const k = base + 2 * i; b.tri(k - 2, k - 1, k); b.tri(k - 1, k + 1, k); }
  }
}

// 이웃한 두 점이 STEP m보다 먼 곳에 사이 점을 넣은 폴리라인: 점의 값(x, y와 그 뒤의 것)을 모두 직선으로 잇는다.
function densify(points) {
  const out = [];
  points.forEach((p, i) => {
    if (i) {
      const q = points[i - 1], n = Math.ceil(Math.hypot(p[0] - q[0], p[1] - q[1]) / STEP);
      for (let k = 1; k < n; k++) out.push(q.map((v, j) => v + (p[j] - v) * k / n));
    }
    out.push(p);
  });
  return out;
}

// 폴리라인 점마다 첫 점부터의 길이(m)
function arcLengths(points) {
  const along = points.length ? [0] : [];
  for (let i = 1; i < points.length; i++) {
    along.push(along[i - 1] + Math.hypot(points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1]));
  }
  return along;
}

// 폴리라인의 점 i에서 왼쪽을 향하는 단위 법선.
export function normalAt(points, i) {
  const [ax, ay] = points[Math.max(0, i - 1)], [bx, by] = points[Math.min(points.length - 1, i + 1)];
  const length = Math.hypot(bx - ax, by - ay) || 1;
  return [-(by - ay) / length, (bx - ax) / length];
}
