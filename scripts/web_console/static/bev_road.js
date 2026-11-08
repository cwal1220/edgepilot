// BEV의 노면 높이: bev.js가 차선선, 도로 경계, 경로, 앞차와 바닥을 그 위에 놓는다. 모델은 차선선, 도로 경계와 경로의
// 점마다 높이(카메라 기준 z, 아래가 +)를 내는데, 그것을 앞 거리 x 하나의 함수로 만든다. 선마다 자차 앞 범퍼(x = 0)
// 높이를 0으로 맞춘 상대 높이로 바꿔 무게 평균하고, 점 사이는 직선으로 읽는다.
// - 범퍼 기준인 까닭: 모델의 노면은 카메라 높이 설정과 프레임마다 어긋난다(2026-09~11 주행 프레임의 5 %가 12 cm 넘게
//   낮다). 그 어긋남은 앞 거리 전체에 같아서 상대 높이에서는 빠지고, 자차는 늘 노면 위에 선다.
// - 좌우 기울기는 보지 않는다: 두 자차 차선선의 높이 차는 95 %가 ±6 cm 안이다.
// 같은 주행들에서 48 m 앞 노면은 자차 노면보다 많게는 +3.6 m, -3.5 m였다(상위 1 %가 ±1 m를 넘는다).

// x에서 꺾은선 (xs, ys)의 값(xs는 커지는 순서, 점 둘 이상). xs 범위 밖이면 NaN이고, 값이 유한하지 않으면 그대로 퍼진다.
function lineAt(xs, ys, x) {
  const n = xs.length;
  if (!(x >= xs[0] && x <= xs[n - 1])) return NaN;
  let j = 0;
  while (j < n - 2 && xs[j + 1] < x) j++;
  const span = xs[j + 1] - xs[j];
  return span > 0 ? ys[j] + (ys[j + 1] - ys[j]) * (x - xs[j]) / span : ys[j];
}

// 앞 거리 xs(m, 0부터 커지는 순서)에서 높이 hs(m, 위가 +, hs[0] = 0)인 노면.
export class RoadProfile {
  constructor(xs, hs) {
    this.xs = xs;
    this.hs = hs;
  }

  // x에서의 높이: 범퍼 뒤는 0, 점 사이는 직선, 마지막 점 너머는 마지막 높이.
  at(x) {
    const xs = this.xs, last = xs.length - 1;
    if (x <= 0) return 0;
    if (x >= xs[last]) return this.hs[last];
    return lineAt(xs, this.hs, x);
  }

  // x에서의 오르막 기울기(높이 / 앞 거리): x가 든 구간의 것, 범퍼 뒤와 마지막 점 너머는 0.
  slope(x) {
    const xs = this.xs;
    if (x <= 0 || x >= xs[xs.length - 1]) return 0;
    let j = 0;
    while (xs[j + 1] < x) j++;
    return (this.hs[j + 1] - this.hs[j]) / (xs[j + 1] - xs[j]);
  }
}

export const FLAT = new RoadProfile([0], [0]);

// sources [{xs, zs, weight}]: 선 하나의 앞 거리(m, 범퍼 기준, 커지는 순서), 그 점의 모델 높이(z, 아래가 +)와 무게.
// knots: 노면 높이를 낼 앞 거리(0보다 크고 커지는 순서). knot마다 그 점을 덮는 선들의 상대 높이(범퍼에서의 z - 그 점의
// z)를 무게 평균한다. 무게가 없거나 범퍼를 덮지 않는 선은 빠지고, 어느 선도 덮지 않는 knot은 건너뛴다.
export function roadProfile(sources, knots) {
  const used = [];
  for (const {xs, zs, weight} of sources) {
    const base = weight > 0 && xs.length >= 2 ? lineAt(xs, zs, 0) : NaN;
    if (Number.isFinite(base)) used.push({xs, zs, weight, base});
  }
  const xs = [0], hs = [0];
  for (const x of knots) {
    let sum = 0, total = 0;
    for (const line of used) {
      const z = lineAt(line.xs, line.zs, x);
      if (Number.isFinite(z)) {
        sum += line.weight * (line.base - z);
        total += line.weight;
      }
    }
    if (total > 0) {
      xs.push(x);
      hs.push(sum / total);
    }
  }
  return new RoadProfile(xs, hs);
}
