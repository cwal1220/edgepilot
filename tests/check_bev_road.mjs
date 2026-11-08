// BEV의 노면 높이(scripts/web_console/static/bev_road.js): 모델 선들의 높이를 범퍼 기준 상대 높이로 무게 평균한
// 노면과 그 읽기. check_web_console.py가 jsc(macOS)나 node가 있으면 이 파일을 돌린다.
import {FLAT, RoadProfile, roadProfile} from "../scripts/web_console/static/bev_road.js";

const say = globalThis.print ?? console.log;
let failures = 0;
const check = (ok, what) => {
  if (!ok) failures++;
  say(`${ok ? "ok  " : "FAIL"} ${what}`);
};
const near = (a, b) => Math.abs(a - b) < 1e-9;

// 읽기: 범퍼 뒤는 0, 점 사이는 직선, 마지막 점 너머는 마지막 높이. 기울기는 그 구간의 것이다
{
  const road = new RoadProfile([0, 10, 30], [0, 1, 1.5]);
  check(road.at(-3) === 0 && near(road.at(5), 0.5) && near(road.at(20), 1.25) && road.at(80) === 1.5, "height");
  check(near(road.slope(5), 0.1) && near(road.slope(20), 0.025) && road.slope(-1) === 0 && road.slope(40) === 0, "slope");
  check(FLAT.at(-5) === 0 && FLAT.at(0) === 0 && FLAT.at(50) === 0 && FLAT.slope(50) === 0, "flat");
}

// 선 높이(z, 아래가 +)는 범퍼에서 0으로 맞춘다: 모델 노면이 카메라 높이 설정과 통째로 어긋나도 노면은 같다
const LINE_X = [-1.5, 0.5, 10, 20, 40];
const uphill = (x) => 1.22 - 0.05 * x;     // 5 % 오르막
{
  const one = roadProfile([{xs: LINE_X, zs: LINE_X.map(uphill), weight: 1}], [10, 20, 40]);
  const shifted = roadProfile([{xs: LINE_X, zs: LINE_X.map((x) => uphill(x) + 0.12), weight: 1}], [10, 20, 40]);
  check(one.xs.join() === "0,10,20,40" && near(one.at(20), 1) && near(one.at(40), 2), "relative to the bumper");
  check(one.hs.every((h, i) => near(h, shifted.hs[i])), "a whole-road offset drops out");
}

// 무게 평균: 확률 0.9와 0.1의 두 선은 9:1로. 무게가 0이거나 NaN인 선과 범퍼를 덮지 않는 선은 빠진다
{
  const flat = {xs: LINE_X, zs: LINE_X.map(() => 1.2), weight: 0.1};
  const rising = {xs: LINE_X, zs: LINE_X.map(uphill), weight: 0.9};
  const road = roadProfile([flat, rising, {...rising, weight: 0}, {...rising, weight: NaN},
                            {xs: [2, 40], zs: [5, -5], weight: 1}], [20]);
  check(near(road.at(20), 0.9 * 1 + 0.1 * 0), `weighted (${road.at(20).toFixed(3)} m)`);
}

// 한 선이 끝난 뒤의 knot은 남은 선으로, 어느 선도 덮지 않는 knot은 건너뛴다(그 너머는 마지막 높이)
{
  const short = {xs: [-1.5, 0.5, 10], zs: [1.22, 1.22, 0.72], weight: 1};
  const long = {xs: LINE_X, zs: LINE_X.map(uphill), weight: 1};
  const road = roadProfile([short, long], [10, 20, 60]);
  check(road.xs.join() === "0,10,20" && near(road.at(10), 0.5) && near(road.at(20), 1) && near(road.at(60), 1),
        "knots past every line are skipped");
  check(roadProfile([], [10, 20]).xs.join() === "0", "no lines: flat");
}

if (failures) throw new Error(`${failures} failed`);
say("all passed");
