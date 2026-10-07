// The ego car for the BEV (bev.js): a 2017 Kia K7 (YG, "All New K7") in Aurora Black Pearl, built in
// code from its published dimensions (4970 x 1870 x 1470 mm, wheelbase 2855, tracks 1602 / 1610,
// 245/45R18) and from measurements of Kia's studio photographs of it from the side, the front and
// the back (Kia Heritage, All-New K7), with owners' photographs for the details. Metres, x forward,
// y left, z up, centred on x and y, standing on z = 0.
//
// What makes it a K7 rather than a sedan:
//   - a high body under a slim glasshouse: the hood rises from 0.75 m at its nose to 1.04 m at the
//     windshield, and the sills run at 1.0 m and sweep up behind the rear doors
//   - the wide grille of vertical chrome blades, its top meeting the headlamps
//   - the headlamps swept up and back into the fenders, two projectors and a Z of running light each
//   - the four-cube fog lamps over a chrome hockey stick
//   - the chrome window surround, and the chrome strip low on the doors
//   - the fastback roofline peaking behind the B-pillar, with a small quarter window
//   - the tail lamps under a chrome line that runs across the whole tail
//   - the plate low on the bumper between red reflectors, the trapezoid exhaust tips, and the 18-inch
//     twenty-spoke wheels
//
// Unlike the other cars (bev_car.js) it is made once and drawn as itself, not as an instance. Its
// body is a surface with smooth normals, and it takes PBR materials that reflect a dark studio lit by
// light strips overhead and along the sides, so black paint reads as black paint, with the long
// highlights it has in the photographs, on the BEV's dark ground. Its tail lamps are lit, as at
// night; setLamps() flashes its turn signals and lights its brake lights.

import * as THREE from "three";

export const K7 = {length: 4.97, width: 1.87, height: 1.47};
const X = K7.length / 2, W = K7.width / 2;
const AXLES = [X - 0.975, X - 0.975 - 2.855];        // front overhang 975 mm (the rest of 4970 - 2855 behind)
const TRACKS = [1.602 / 2, 1.610 / 2];
const TYRE = {radius: 0.339, width: 0.245, rim: 0.2286};        // 245/45R18
const ARCH = 0.385;
// Along x, on the centre line: the windshield's foot, the roof's front, top and back, the deck (the
// backlight's foot). The side windows: the A-pillar's foot, the end of the black sail under the
// mirror, the B-pillar, the rear door glass's end, the quarter window's tail.
const COWL = 1.08, ROOF_FRONT = 0.15, ROOF_TOP = -0.45, ROOF_REAR = -1.25, DECK = -2.1;
const DLO = {front: 1.0, sail: 0.64, pillar: [-0.157, -0.075], divider: -1.0, tip: -1.365};

const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const lerp = (a, b, t) => a + (b - a) * t;
const ease = (a, b, x) => { const t = clamp((x - a) / (b - a), 0, 1); return t * t * (3 - 2 * t); };
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const unit = (a) => { const l = Math.hypot(...a) || 1; return a.map((v) => v / l); };
const vec = (a) => new THREE.Vector3(...a);

// A smooth curve through [x, value] knots, x rising: monotone between them (Fritsch-Carlson), level
// beyond the ends.
function curve(knots) {
  const n = knots.length, xs = knots.map(([x]) => x), vs = knots.map(([, v]) => v);
  const d = xs.slice(1).map((x, i) => (vs[i + 1] - vs[i]) / (x - xs[i]));
  const m = vs.map((_, i) => (i === 0 ? d[0] : i === n - 1 ? d[n - 2] : d[i - 1] * d[i] <= 0 ? 0 : (d[i - 1] + d[i]) / 2));
  for (let i = 0; i < n - 1; i++) {
    if (d[i] === 0) { m[i] = m[i + 1] = 0; continue; }
    const a = m[i] / d[i], b = m[i + 1] / d[i], s = a * a + b * b;
    if (s > 9) { m[i] = 3 * a * d[i] / Math.sqrt(s); m[i + 1] = 3 * b * d[i] / Math.sqrt(s); }
  }
  return (x) => {
    if (x <= xs[0]) return vs[0];
    if (x >= xs[n - 1]) return vs[n - 1];
    let i = 0;
    while (x > xs[i + 1]) i++;
    const h = xs[i + 1] - xs[i], t = (x - xs[i]) / h, t2 = t * t, t3 = t2 * t;
    return (2 * t3 - 3 * t2 + 1) * vs[i] + (t3 - 2 * t2 + t) * h * m[i] + (3 * t2 - 2 * t3) * vs[i + 1] + (t3 - t2) * h * m[i + 1];
  };
}

// Where a polyline of [y, z] points crosses |y| = a first: its z, or null.
function crossing(path, a) {
  for (let k = 1; k < path.length; k++) {
    const [y0, z0] = path[k - 1], [y1, z1] = path[k];
    if ((a - y0) * (a - y1) <= 0 && y0 !== y1) return lerp(z0, z1, (a - y0) / (y1 - y0));
  }
  return null;
}

// -- the body's shape ------------------------------------------------------------------------------
// The half width at the shoulder, rounding off at the nose and the tail.
const halfWidth = (x) => W - 0.17 * clamp((x - (X - 0.9)) / 0.9, 0, 1) ** 2 - 0.09 * clamp((-X + 0.62 - x) / 0.62, 0, 1) ** 2;
// The top of the lower body on the centre line and at its edges: the hood, high at the cowl and
// dipping at the nose below its corners; the sills under the windows; the deck out to its lip.
const hood = curve([[COWL, 1.035], [1.3, 1.025], [1.64, 0.99], [1.99, 0.935], [2.24, 0.87], [2.34, 0.835], [2.43, 0.79], [X, 0.752]]);
const fender = curve([[COWL, 0.99], [1.5, 0.955], [1.9, 0.918], [2.15, 0.895], [2.33, 0.882], [X, 0.875]]);
const belt = curve([[DECK, 1.075], [-1.3, 1.055], [-0.74, 1.03], [-0.17, 1.008], [0.675, 0.991], [COWL, 0.99]]);
const deck = curve([[-X, 1.075], [-2.38, 1.105], [DECK, 1.12]]);
const deckEdge = curve([[-X, 1.035], [-2.38, 1.06], [DECK, 1.075]]);
const top = (x) => (x >= COWL ? [hood(x), fender(x)] : x > DECK ? [belt(x), belt(x)] : [deck(x), deckEdge(x)]);
// The shoulder: a crease along the side from the headlamps, over the door handles, into the tail lamps.
const shoulder = curve([[-X, 0.94], [-1.875, 0.93], [-1.0, 0.895], [0.07, 0.885], [1.5, 0.86], [2.1, 0.84], [X, 0.80]]);

// The underside: the bumpers' lips higher than the sills, and the wheel arches.
function bottom(x) {
  let z = 0.165 + 0.095 * clamp((x - (X - 0.45)) / 0.45, 0, 1) ** 2 + 0.14 * clamp((-X + 0.6 - x) / 0.6, 0, 1) ** 2;
  for (const c of AXLES) {
    const dx = Math.abs(x - c);
    if (dx <= ARCH) z = Math.max(z, TYRE.radius + Math.sqrt(ARCH * ARCH - dx * dx));
  }
  return z;
}

// The lower body's section at x, half of it from the top centre round to the bottom centre: [y, z].
// The hood's two ridges (1), the shoulder (5) and the bottom edge (9) are creases; below the
// shoulder the door dips in and swells again where its chrome strip runs.
const ridge = (x) => 0.012 * ease(COWL, COWL + 0.15, x) * (1 - ease(X - 0.3, X - 0.08, x));
function lowerHalf(x) {
  const w = halfWidth(x), zb = bottom(x), [centre, edge] = top(x);
  const zs = clamp(shoulder(x), zb + 0.05, edge - 0.04);
  return [
    [0, centre],
    [w * 0.6, lerp(centre, edge, 0.3) + ridge(x)],
    [w - 0.085, edge],
    [w - 0.03, edge - Math.min(0.03, (edge - zs) * 0.4)],
    [w - 0.004, lerp(zs, edge, 0.3)],
    [w, zs],
    [w - 0.012, lerp(zs, zb, 0.45)],
    [w - 0.005, lerp(zs, zb, 0.78)],
    [w - 0.035, zb + Math.min(0.055, (zs - zb) * 0.15)],
    [w - 0.08, zb],
    [0, zb],
  ];
}

// The ends are not flat walls. In plan they curve back towards their corners, the front's lower
// corners deepest, round the fog lamps; in elevation they lean back: the nose under the hood and the
// chin, the trunk lid's face up to its lip and the bumper's foot. SETBACK(y, z) is how far, fully on
// the faces, fading out FADE in. Everything near the ends goes through bend(), the body and what lies
// on it alike.
const sweep = (y, far, y0, y1) => far * clamp((Math.abs(y) - y0) / (y1 - y0), 0, 1) ** 1.5;    // steepening out to the corners
const SETBACK = {
  front: (y, z) => sweep(y, lerp(0.30, 0.20, ease(0.42, 0.62, z)), 0.40, 0.84) + 0.06 * ease(0.62, 0.86, z) ** 1.5 + 0.045 * ease(0.40, 0.20, z) ** 1.5,
  rear: (y, z) => sweep(y, 0.16, 0.35, 0.85) + 0.14 * clamp((z - 0.6) / 0.5, 0, 1) ** 2 + 0.08 * ease(0.45, 0.27, z) ** 1.5,
};
const FADE = {front: 0.6, rear: 0.55};
function bend([x, y, z]) {
  return [x - SETBACK.front(y, z) * ease(X - FADE.front, X, x) + SETBACK.rear(y, z) * ease(-X + FADE.rear, -X, x), y, z];
}

// How far out the side is at height z (between the top's edge and the sill), for what lies on it.
function sideY(x, z) {
  const half = lowerHalf(x);
  for (let k = 2; k < 9; k++) {
    const [y0, z0] = half[k], [y1, z1] = half[k + 1];
    if (z <= z0 && z >= z1) return z0 === z1 ? y0 : lerp(y0, y1, (z0 - z) / (z0 - z1));
  }
  return z > half[2][1] ? half[2][0] : half[9][0];
}

// The glasshouse's top on the centre line: the windshield, the roof, the long backlight.
const roofArc = (x) => 1.47 - 0.111 * (x - ROOF_TOP) ** 2;
function roofRaw(x) {
  if (x >= ROOF_FRONT) {
    const s = (COWL - x) / (COWL - ROOF_FRONT);
    return hood(COWL) + (roofArc(ROOF_FRONT) - hood(COWL)) * s * (1.12 - 0.12 * s);
  }
  if (x >= ROOF_REAR) return roofArc(x);
  const s = (ROOF_REAR - x) / (ROOF_REAR - DECK);
  return roofArc(ROOF_REAR) - (roofArc(ROOF_REAR) - deck(DECK)) * s ** 1.15;
}

function roofLine(x) {          // filleted where the glass meets the roof, exact at the ends
  const r = Math.min(0.05, (x - DECK) / 2, (COWL - x) / 2);
  let sum = 0;
  for (let k = -2; k <= 2; k++) sum += roofRaw(x + k * r);
  return sum / 5;
}

// The side windows' opening: its top, along the A-pillar and the roof rail and down to the quarter
// window's tail, and its sill, which sweeps up behind the rear door to meet it there.
const dloTop = curve([[DLO.tip, 1.209], [-1.282, 1.247], [-1.0, 1.333], [-0.604, 1.392], [-0.233, 1.389], [-0.032, 1.37],
                      [0.311, 1.299], [0.668, 1.143], [DLO.front, 0.995]]);
const sweepUp = curve([[DLO.tip, 1.209], [-1.33, 1.16], [-1.3, 1.135], [-1.2, 1.105], [-1.074, 1.082], [-0.9, 1.06], [-0.74, 1.045]]);
const sill = (x) => Math.max(belt(x) + 0.015, x < -0.74 ? sweepUp(x) : 0);

// Where the glasshouse's top turns down into its side: above the windows' chrome; at the
// windshield's corners; and on the C-pillars, along the backlight's edges down to the deck.
function sideTop(x) {
  if (x > DLO.front) return lerp(dloTop(DLO.front) + 0.03, belt(COWL) + 0.004, ease(DLO.front, COWL, x));
  if (x >= DLO.tip) return dloTop(x) + 0.03;
  return lerp(belt(DECK) + 0.02, dloTop(DLO.tip) + 0.03, ((x - DECK) / (DLO.tip - DECK)) ** 0.85);
}

// The glasshouse's section at x, half of it from the roof's centre down to its foot on the sill:
// [y, z]. Its side leans in (tumblehome) and carries the windows between chrome: their top (5) and
// their sill (7).
function glassHalf(x) {
  const zb = belt(x), zt = Math.max(roofLine(x), zb + 0.001), zTop = clamp(sideTop(x), zb + 0.0005, zt - 0.0005);
  const wb = halfWidth(x) - 0.085 - 0.05 * ease(0.6, COWL, x), at = (z) => wb - (z - zb) * 0.42;
  const open = x > DLO.tip && x < DLO.front;
  const upper = open ? clamp(dloTop(x), zb, zTop - 0.014) : lerp(zb, zTop, 0.6), rail = Math.min(upper + 0.012, zTop);
  const lower = open ? clamp(sill(x), zb + 0.0002, upper) : lerp(zb, zTop, 0.4), chrome = Math.max(zb + 0.0001, lower - 0.013);
  const ye = at(zTop) - 0.01, drop = zt - zTop;
  const edge = x < ROOF_REAR ? lerp(0.9, 0.78, (ROOF_REAR - x) / (ROOF_REAR - DECK)) : 0.9;   // the backlight narrows to its foot
  return [[0, zt], [ye * 0.55, zt - drop * 0.18], [ye * edge, zt - drop * 0.62], [ye, zTop], [at(rail), rail], [at(upper), upper],
          [at((upper + lower) / 2), (upper + lower) / 2], [at(lower), lower], [at(chrome), chrome], [wb, zb]];
}

// What each band of the glasshouse is, by where it is: 0 and 1 the roof or the glass, 2 their edge,
// 3 the side above the windows, 4 the chrome over them, 5 and 6 the windows (or the pillars and the
// sail under the mirror), 7 the chrome on their sill, 8 the side below it.
function glassPart(band, x) {
  const roof = x < ROOF_FRONT + 0.02 && x > ROOF_REAR - 0.02, open = x > DLO.tip && x < DLO.front;
  if (band <= 1) return roof ? "paint" : "glass";
  if (band === 2) return roof || x < ROOF_REAR ? "paint" : "gloss";
  if (band === 4 || band === 7) return open ? "chrome" : "paint";
  if (band === 5 || band === 6) {
    if (!open) return "paint";
    const pillar = (x > DLO.pillar[0] && x < DLO.pillar[1]) || Math.abs(x - DLO.divider) < 0.015;
    return x > DLO.sail || pillar ? "gloss" : "glass";
  }
  return "paint";
}

// Stations from x0 to x1: at every mark inside, and at least every step.
function stations(x0, x1, step, marks = []) {
  const xs = [x0, x1, ...marks.filter((x) => x > x0 && x < x1)];
  for (let x = x0 + step; x < x1; x += step) xs.push(x);
  xs.sort((a, b) => a - b);
  return xs.filter((x, i) => i === 0 || x - xs[i - 1] > 2e-3);
}

const archMarks = (steps) => AXLES.flatMap((c) => Array.from({length: steps + 1}, (_, i) => c - ARCH * Math.cos(Math.PI * i / steps))
                                                    .concat([c - ARCH - 0.01, c + ARCH + 0.01]));

// -- collecting triangles by material ------------------------------------------------------------------
class Parts {
  constructor() { this.lists = new Map(); }

  list(name) {
    if (!this.lists.has(name)) this.lists.set(name, {p: [], n: []});
    return this.lists.get(name);
  }

  tri(name, a, b, c, na, nb, nc) {
    const list = this.list(name);
    list.p.push(...a, ...b, ...c);
    list.n.push(...na, ...nb, ...nc);
  }

  // A flat polygon of [x, y, z] corners (convex, or a fan from its first corner), facing toward.
  face(name, points, toward) {
    let n = unit(cross(sub(points[1], points[0]), sub(points[2], points[0])));
    if (dot(n, toward) < 0) { points = points.slice().reverse(); n = n.map((v) => -v); }
    for (let i = 1; i + 1 < points.length; i++) this.tri(name, points[0], points[i], points[i + 1], n, n, n);
  }

  // The same with a normal at each corner, for a piece of a curved surface.
  smooth(name, points, normals, toward) {
    const n = unit(cross(sub(points[1], points[0]), sub(points[2], points[0])));
    if (dot(n, toward) < 0) { points = points.slice().reverse(); normals = normals.slice().reverse(); }
    for (let i = 1; i + 1 < points.length; i++) this.tri(name, points[0], points[i], points[i + 1], normals[0], normals[i], normals[i + 1]);
  }

  // A three.js geometry with its normals, moved by matrix (no mirroring).
  geometry(name, geometry, matrix = null) {
    const g = geometry.index ? geometry.toNonIndexed() : geometry.clone();
    if (matrix) g.applyMatrix4(matrix);
    const list = this.list(name), p = g.attributes.position.array, n = g.attributes.normal.array;
    for (let i = 0; i < p.length; i++) { list.p.push(p[i]); list.n.push(n[i]); }
    geometry.dispose();
    g.dispose();
  }

  // A surface through rows of points, rows[i][j] = [x, y, z]: quads between rows i, i + 1 and
  // columns j, j + 1 (the rows closed into rings if closed), each in pick(i, j) (null leaves it
  // out), facing away from inside(i). Normals are smoothed over the surface, except across the
  // columns in creases.
  surface(rows, {closed = false, creases = [], pick, inside}) {
    const n = rows[0].length, m = closed ? n : n - 1, crease = new Set(creases);
    const normals = rows.map(() => Array.from({length: n}, () => [[0, 0, 0], [0, 0, 0]]));   // [left of the column, right of it]
    const faces = [];
    for (let i = 0; i + 1 < rows.length; i++) {
      const centre = inside(i);
      for (let j = 0; j < m; j++) {
        const k = (j + 1) % n, a = rows[i][j], b = rows[i + 1][j], c = rows[i + 1][k], d = rows[i][k];
        let f = cross(sub(c, a), sub(d, b));
        if (Math.hypot(...f) < 1e-10) continue;
        const middle = [0, 1, 2].map((q) => (a[q] + b[q] + c[q] + d[q]) / 4);
        if (dot(f, sub(middle, centre)) < 0) f = f.map((v) => -v);
        faces.push({i, j, k, f});
        for (const [r, col, side] of [[i, j, 1], [i + 1, j, 1], [i + 1, k, 0], [i, k, 0]]) {
          for (const s of crease.has(col) ? [side] : [0, 1]) {
            const sum = normals[r][col][s];
            sum[0] += f[0]; sum[1] += f[1]; sum[2] += f[2];
          }
        }
      }
    }
    for (const {i, j, k, f} of faces) {
      const name = pick(i, j);
      if (!name) continue;
      const corners = [[rows[i][j], unit(normals[i][j][1])], [rows[i + 1][j], unit(normals[i + 1][j][1])],
                       [rows[i + 1][k], unit(normals[i + 1][k][0])], [rows[i][k], unit(normals[i][k][0])]];
      for (const [p, q, r] of [[0, 1, 2], [0, 2, 3]]) {
        const [[a, na], [b, nb], [c, nc]] = [corners[p], corners[q], corners[r]];
        if (dot(cross(sub(b, a), sub(c, a)), f) >= 0) this.tri(name, a, b, c, na, nb, nc);
        else this.tri(name, a, c, b, na, nc, nb);
      }
    }
  }

  meshes(materials) {
    const group = new THREE.Group();
    for (const [name, list] of this.lists) {
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute("position", new THREE.Float32BufferAttribute(list.p, 3));
      geometry.setAttribute("normal", new THREE.Float32BufferAttribute(list.n, 3));
      geometry.computeBoundingSphere();
      const mesh = new THREE.Mesh(geometry, materials[name]);
      mesh.name = name;
      group.add(mesh);
    }
    return group;
  }
}

// -- the body -----------------------------------------------------------------------------------------
function body(parts) {
  // the lower body: a ring per station, the left half then the right
  const ring = (x, half) => [...half.map(([y, z]) => [x, y, z]), ...half.slice(1, -1).reverse().map(([y, z]) => [x, -y, z])];
  const xs = stations(-X, X, 0.05, [...archMarks(10), X - 0.15, X - 0.08, X - 0.04, X - 0.015, -X + 0.15, -X + 0.08, -X + 0.04, -X + 0.015,
                                    -2.38, COWL, DECK]);
  const rows = xs.map((x) => ring(x, lowerHalf(x)).map(bend));
  const middle = (row) => [row[0][0], 0, (row[0][2] + row[10][2]) / 2];
  parts.surface(rows, {closed: true, creases: [1, 19, 5, 15, 9, 11], inside: (i) => middle(rows[i]),
                       pick: (i, j) => (j === 9 || j === 10 ? "under" : "paint")});
  // the end faces, in cells that follow their curves
  for (const out of [1, -1]) {
    const half = lowerHalf(out * X);
    onFace(parts, "paint", out, [...half, ...half.slice(1, -1).reverse().map(([y, z]) => [-y, z])], 0, false, false);
  }

  // the glasshouse: from the left foot over the roof to the right foot
  const glassXs = stations(DECK, COWL, 0.04, [ROOF_FRONT - 0.02, ROOF_FRONT + 0.02, ROOF_REAR - 0.02, ROOF_REAR + 0.02, ...DLO.pillar,
                                              DLO.divider - 0.015, DLO.divider + 0.015, DLO.tip, DLO.tip + 0.01, -1.33, -1.3, DLO.sail, DLO.front]);
  const glassRows = glassXs.map((x) => {
    const half = glassHalf(x);
    return [...half.slice().reverse().map(([y, z]) => [x, y, z]), ...half.slice(1).map(([y, z]) => [x, -y, z])].map(bend);
  });
  parts.surface(glassRows, {inside: (i) => [glassXs[i], 0, belt(glassXs[i]) - 0.3],
                            pick: (i, j) => glassPart(j < 9 ? 8 - j : j - 9, (glassXs[i] + glassXs[i + 1]) / 2)});
}

// -- what lies on the body ------------------------------------------------------------------------------
// A polygon on the front (out 1) or rear (out -1) face from [y, z] corners, lifted off it, on both
// sides when mirrored; every y must cut it once. It goes on in cells 25 x 30 mm, so it follows the
// face's curves, and is clipped to the face's outline unless told not to be.
function onFace(parts, name, out, corners, lift = 0.004, mirrored = true, clip = true) {
  const ys = corners.map(([y]) => y), y0 = Math.min(...ys), y1 = Math.max(...ys), slices = Math.max(1, Math.ceil((y1 - y0) / 0.025));
  const outline = clip ? lowerHalf(out * X) : null;
  const span = (y) => {                                          // [lowest, highest] z of it at y, inside the face
    const zs = [];
    corners.forEach(([ya, za], k) => {
      const [yb, zb] = corners[(k + 1) % corners.length];
      if ((y - ya) * (y - yb) > 0) return;
      if (ya === yb) zs.push(za, zb);
      else zs.push(lerp(za, zb, (y - ya) / (yb - ya)));
    });
    if (!zs.length) return null;
    let lo = Math.min(...zs), hi = Math.max(...zs);
    if (outline) {
      const a = Math.abs(y), upper = crossing(outline.slice(0, 6), a), lower = crossing(outline.slice(5), a);
      if (upper === null || lower === null) return null;
      lo = Math.max(lo, lower + 0.004);
      hi = Math.min(hi, upper - 0.004);
    }
    return hi > lo - 1e-9 ? [lo, Math.max(lo, hi)] : null;
  };
  for (let k = 0; k < slices; k++) {
    const ya = lerp(y0, y1, k / slices), yb = lerp(y0, y1, (k + 1) / slices), a = span(ya), b = span(yb);
    if (!a || !b || Math.max(a[1] - a[0], b[1] - b[0]) < 1e-6) continue;  // a slice may narrow to a point at one end
    const [la, ha] = a, [lb, hb] = b;
    const rows = Math.max(1, Math.ceil(Math.max(ha - la, hb - lb) / 0.03));
    for (let r = 0; r < rows; r++) {
      const t0 = r / rows, t1 = (r + 1) / rows;
      for (const s of mirrored ? [1, -1] : [1]) {
        const quad = [[ya, lerp(la, ha, t0)], [yb, lerp(lb, hb, t0)], [yb, lerp(lb, hb, t1)], [ya, lerp(la, ha, t1)]]
          .filter(([y, z], i, all) => { const [py, pz] = all[(i + 3) % 4]; return Math.abs(y - py) + Math.abs(z - pz) > 1e-9; });
        if (quad.length < 3) continue;
        parts.smooth(name, quad.map(([y, z]) => bend([out * (X + lift), s * y, z])), quad.map(([y, z]) => faceNormal(out, s * y, z)), [out, 0, 0]);
      }
    }
  }
}

// Which way the front (out 1) or rear (out -1) face looks at [y, z].
function faceNormal(out, y, z) {
  const at = (dy, dz) => bend([out * X, y + dy, z + dz]);
  const n = unit(cross(sub(at(0.004, 0), at(-0.004, 0)), sub(at(0, 0.004), at(0, -0.004))));
  return n[0] * out < 0 ? n.map((c) => -c) : n;
}

// Where a point on the front (out 1) or rear (out -1) face is, lifted off it, and how the face lies
// there: {p, u, v, n}, a right-handed frame with v up the face and n out of it.
function facePlace(out, y, z, lift = 0) {
  const at = (dy, dz) => bend([out * X, y + dy, z + dz]);
  const n = faceNormal(out, y, z), rise = unit(sub(at(0, 0.005), at(0, -0.005)));
  const v = unit(sub(rise, n.map((c) => c * dot(rise, n))));
  return {p: at(0, 0).map((c, q) => c + n[q] * lift), u: cross(v, n), v, n};
}

const frame = ({p, u, v, n}) => new THREE.Matrix4().makeBasis(vec(u), vec(v), vec(n)).setPosition(...p);

// A band on the sides from x0 to x1, between heights low(x) and high(x), lifted off them.
function onSide(parts, name, x0, x1, low, high, lift = 0.004, steps = 10, sides = [1, -1]) {
  for (const s of sides) {
    for (let k = 0; k < steps; k++) {
      const xa = lerp(x0, x1, k / steps), xb = lerp(x0, x1, (k + 1) / steps), xm = (xa + xb) / 2;
      const corners = [[xa, low(xa)], [xb, low(xb)], [xb, high(xb)], [xa, high(xa)]];
      const out = 0.6 * (ease(X - 0.4, X, xm) - ease(-X + 0.4, -X, xm));       // the sides turn into the faces at the ends
      const at = (x, z) => bend([x, s * (sideY(x, z) + lift), z]);
      const normal = (x, z) => {
        const n = unit(cross(sub(at(x + 0.004, z), at(x - 0.004, z)), sub(at(x, z + 0.004), at(x, z - 0.004))));
        return dot(n, [out, s, 0]) < 0 ? n.map((c) => -c) : n;
      };
      parts.smooth(name, corners.map(([x, z]) => at(x, z)), corners.map(([x, z]) => normal(x, z)), [out, s, 0]);
    }
  }
}

// A thin band along a path of [y, z] points on a face, mitred at its bends, in pieces 20 mm long.
function lineOnFace(parts, name, out, path, width, lift = 0.006, mirrored = true) {
  const normals = path.slice(1).map(([y1, z1], k) => {                  // of each segment, in the face
    const [y0, z0] = path[k], length = Math.hypot(y1 - y0, z1 - z0) || 1;
    return [-(z1 - z0) / length, (y1 - y0) / length];
  });
  const offsets = path.map((_, k) => {                                   // half the width out, mitred
    const a = normals[Math.max(0, k - 1)], b = normals[Math.min(normals.length - 1, k)];
    const m = [a[0] + b[0], a[1] + b[1]], scale = width / 2 / Math.max(0.3, (m[0] * b[0] + m[1] * b[1]));
    return [m[0] * scale, m[1] * scale];
  });
  for (const s of mirrored ? [1, -1] : [1]) {
    for (let k = 1; k < path.length; k++) {
      const [y0, z0] = path[k - 1], [y1, z1] = path[k], [o0, p0] = offsets[k - 1], [o1, p1] = offsets[k];
      const pieces = Math.max(1, Math.ceil(Math.hypot(y1 - y0, z1 - z0) / 0.02));
      const edge = (t, side) => [lerp(y0 + side * o0, y1 + side * o1, t), lerp(z0 + side * p0, z1 + side * p1, t)];
      for (let i = 0; i < pieces; i++) {
        const quad = [edge(i / pieces, 1), edge((i + 1) / pieces, 1), edge((i + 1) / pieces, -1), edge(i / pieces, -1)];
        parts.smooth(name, quad.map(([y, z]) => bend([out * (X + lift), s * y, z])), quad.map(([y, z]) => faceNormal(out, s * y, z)), [out, 0, 0]);
      }
    }
  }
}

// An ellipse ring (and its inside) at centre, in the plane spanned by u and v (unit vectors), facing toward.
function emblem(parts, centre, u, v, a, b, toward) {
  const at = (r, t) => centre.map((c, q) => c + Math.cos(t) * (a - r) * u[q] + Math.sin(t) * (b - r) * v[q]);
  for (let k = 0; k < 24; k++) {
    const t0 = 2 * Math.PI * k / 24, t1 = 2 * Math.PI * (k + 1) / 24;
    parts.face("chrome", [at(0, t0), at(0, t1), at(0.008, t1), at(0.008, t0)], toward);
    parts.face("grille", [centre, at(0.008, t0), at(0.008, t1)], toward);
  }
}

function front(parts) {
  // the grille: wide at the top, where it meets the headlamps, its foot curving down to the middle; a
  // chrome frame round vertical blades, each a V of two facets
  const grille = [[0, 0.735], [0.2, 0.731], [0.4, 0.718], [0.515, 0.702], [0.497, 0.655], [0.472, 0.61], [0.452, 0.586], [0.40, 0.571],
                  [0.30, 0.554], [0.15, 0.537], [0, 0.528]];
  onFace(parts, "grille", 1, [...grille, ...grille.slice(1, -1).reverse().map(([y, z]) => [-y, z])], 0.002, false);
  lineOnFace(parts, "chrome", 1, grille, 0.022, 0.016);
  const upper = grille.slice(0, 4), lower = grille.slice(3).reverse();
  for (let y = -0.4875; y < 0.49; y += 0.0325) {
    const z1 = crossing(upper, Math.abs(y)) - 0.02, z0 = crossing(lower, Math.abs(y)) + 0.02;
    if (z1 - z0 < 0.03) continue;
    const place = facePlace(1, y, (z0 + z1) / 2, 0.009), length = Math.hypot(...sub(bend([X, y, z1]), bend([X, y, z0])));
    for (const side of [1, -1]) {
      const m = frame(place).multiply(new THREE.Matrix4().makeTranslation(side * 0.005, 0, 0)).multiply(new THREE.Matrix4().makeRotationY(side * 0.45));
      parts.geometry("chrome", new THREE.BoxGeometry(0.013, length, 0.004), m);
    }
  }
  // the KIA badge on the hood's nose
  const n = unit([0.53, 0, 0.85]);
  emblem(parts, bend([X - 0.05, 0, hood(X - 0.05) + 0.004]), [0, 1, 0], unit(cross(n, [0, 1, 0])).map((c) => -c), 0.064, 0.032, n);

  // the headlamps: from the grille's top corners up and out under the hood's edge, round into the
  // fenders, on a chrome line; two projectors each, and a Z of running light along the foot, up the
  // outer end, and in a block at the inner end
  onFace(parts, "lens", 1, [[0.496, 0.758], [0.52, 0.688], [0.65, 0.697], [0.78, 0.713], [0.80, 0.80], [0.78, 0.87], [0.62, 0.81]]);
  onSide(parts, "lens", X - 0.55, X, (x) => lerp(0.84, 0.72, ease(X - 0.55, X - 0.46, x)) - 0.02 * ease(X - 0.46, X, x),
         (x) => lerp(0.84, 0.868, ease(X - 0.5, X - 0.2, x)), 0.004, 14);
  for (const s of [1, -1]) {
    for (const [y, z, r] of [[0.74, 0.786, 0.027], [0.658, 0.778, 0.026]]) {
      const m = frame(facePlace(1, s * y, z, 0.007));
      parts.geometry("headlight", new THREE.CircleGeometry(r * 0.7, 20), m);
      parts.geometry("chrome", new THREE.RingGeometry(r * 0.7, r, 20), m.clone().multiply(new THREE.Matrix4().makeTranslation(0, 0, -0.001)));
    }
  }
  lineOnFace(parts, "drl", 1, [[0.53, 0.699], [0.758, 0.716], [0.766, 0.79]], 0.008, 0.008);
  onFace(parts, "drl", 1, [[0.506, 0.724], [0.59, 0.727], [0.59, 0.752], [0.506, 0.756]], 0.007);
  for (const [s, name] of [[1, "turnLeft"], [-1, "turnRight"]]) {              // turning, that block flashes amber
    onFace(parts, name, 1, [[s * 0.506, 0.724], [s * 0.59, 0.727], [s * 0.59, 0.752], [s * 0.506, 0.756]], 0.009, false);
  }
  onSide(parts, "drl", X - 0.45, X, (x) => lerp(0.728, 0.712, ease(X - 0.45, X, x)), (x) => lerp(0.737, 0.721, ease(X - 0.45, X, x)), 0.008, 12);
  lineOnFace(parts, "chrome", 1, [[0.515, 0.684], [0.768, 0.712]], 0.007);
  onSide(parts, "chrome", X - 0.47, X, (x) => lerp(0.712, 0.69, ease(X - 0.47, X, x)), (x) => lerp(0.719, 0.697, ease(X - 0.47, X, x)), 0.005, 12);

  // the bumper: the fog lamps' four cubes in a chrome bezel, the hockey stick under them, the plate,
  // the lower intake with its slats
  onFace(parts, "trim", 1, [[0.57, 0.305], [0.79, 0.305], [0.79, 0.455], [0.57, 0.455]], 0.002);
  onFace(parts, "chrome", 1, [[0.605, 0.321], [0.757, 0.321], [0.757, 0.433], [0.605, 0.433]], 0.004);
  onFace(parts, "under", 1, [[0.611, 0.327], [0.751, 0.327], [0.751, 0.427], [0.611, 0.427]], 0.005);
  for (const [ya, yb] of [[0.615, 0.673], [0.689, 0.747]]) {
    for (const [za, zb] of [[0.331, 0.373], [0.381, 0.423]]) onFace(parts, "fog", 1, [[ya, za], [yb, za], [yb, zb], [ya, zb]], 0.007);
  }
  lineOnFace(parts, "chrome", 1, [[0.452, 0.382], [0.51, 0.312], [0.70, 0.29]], 0.012);
  onSide(parts, "chrome", X - 0.36, X, () => 0.274, () => 0.288, 0.005);
  onFace(parts, "trim", 1, [[0.275, 0.372], [0.275, 0.503], [-0.275, 0.503], [-0.275, 0.372]], 0.003, false);
  onFace(parts, "plate", 1, [[0.26, 0.382], [0.26, 0.493], [-0.26, 0.493], [-0.26, 0.382]], 0.006, false);
  onFace(parts, "grille", 1, [[0.452, 0.368], [0.40, 0.262], [-0.40, 0.262], [-0.452, 0.368]], 0.002, false);
  for (const z of [0.296, 0.332]) onFace(parts, "trim", 1, [[0.43, z], [0.43, z + 0.009], [-0.43, z + 0.009], [-0.43, z]], 0.004, false);
}

function rear(parts) {
  // the tail lamps: under a chrome line that runs across the whole tail, from a rounded outer end on
  // the fender to a diagonal on the trunk lid, and round the corner along the fender to a point;
  // the turn signal (amber) and the reversing lamp (clear) along their tops, light guides below
  onFace(parts, "tailLens", -1, [[0.322, 0.985], [0.464, 0.847], [0.80, 0.86], [0.86, 0.90], [0.86, 0.975], [0.80, 0.995], [0.34, 0.995]]);
  const tailLow = (x) => lerp(0.864, 0.983, ease(-2.05, -1.875, x)), tailTop = (x) => lerp(0.995, 0.983, ease(-X, -1.875, x));
  onSide(parts, "tailLens", -X, -1.875, tailLow, tailTop);
  onFace(parts, "amberLens", -1, [[0.63, 0.936], [0.738, 0.936], [0.738, 0.978], [0.63, 0.978]], 0.005);
  for (const [s, name] of [[1, "turnLeft"], [-1, "turnRight"]]) {
    onFace(parts, name, -1, [[s * 0.63, 0.936], [s * 0.738, 0.936], [s * 0.738, 0.978], [s * 0.63, 0.978]], 0.007, false);
  }
  onFace(parts, "reverse", -1, [[0.414, 0.932], [0.557, 0.932], [0.557, 0.963], [0.414, 0.963]], 0.005);
  for (const z of [0.89, 0.909, 0.925]) lineOnFace(parts, "tail", -1, [[0.47, z - 0.002], [0.765, z + 0.002]], 0.006);
  onSide(parts, "tail", -X, -2.0, () => 0.905, () => 0.913, 0.006);
  onSide(parts, "tail", -X, -1.94, () => 0.95, () => 0.958, 0.006);
  lineOnFace(parts, "chrome", -1, [[0.79, 1.0], [0, 1.0]], 0.007);
  onFace(parts, "chrome", -1, [[0.322, 0.976], [0.322, 0.998], [-0.322, 0.998], [-0.322, 0.976]], 0.005, false);
  lineOnFace(parts, "chrome", -1, [[0.778, 0.863], [0.464, 0.843], [0.322, 0.976]], 0.005);
  // the trunk lid's lower edge, a U round the badges: KIA on the lid, K7 low on its left
  lineOnFace(parts, "under", -1, [[0.62, 0.845], [0.60, 0.74], [0.577, 0.687], [0.508, 0.637], [0, 0.637]], 0.004, 0.003);
  const badge = facePlace(-1, 0, 0.895, 0.005);
  emblem(parts, badge.p, badge.u, badge.v, 0.07, 0.035, badge.n);
  onFace(parts, "chrome", -1, [[0.44, 0.73], [0.497, 0.73], [0.497, 0.76], [0.44, 0.76]], 0.005, false);
  // the bumper: the plate low on it over a chrome strip, red reflectors running round its corners
  // into chrome spears along its sides, the exhaust tips under them, a silver line over the diffuser
  onFace(parts, "trim", -1, [[0.275, 0.442], [0.275, 0.566], [-0.275, 0.566], [-0.275, 0.442]], 0.003, false);
  onFace(parts, "plate", -1, [[0.26, 0.452], [0.26, 0.556], [-0.26, 0.556], [-0.26, 0.452]], 0.006, false);
  onFace(parts, "chrome", -1, [[0.37, 0.408], [0.37, 0.416], [-0.37, 0.416], [-0.37, 0.408]], 0.004, false);
  onFace(parts, "reflector", -1, [[0.543, 0.417], [0.86, 0.417], [0.86, 0.457], [0.543, 0.457]], 0.005);
  onSide(parts, "reflector", -X, -2.2, () => 0.418, () => 0.456, 0.005, 4);
  onSide(parts, "chrome", -2.2, -1.88, (x) => lerp(0.436, 0.40, (x + 2.2) / 0.32), (x) => lerp(0.45, 0.414, (x + 2.2) / 0.32), 0.005, 6);
  onFace(parts, "chrome", -1, [[0.519, 0.387], [0.727, 0.387], [0.71, 0.325], [0.535, 0.325]], 0.006);
  onFace(parts, "under", -1, [[0.533, 0.378], [0.713, 0.378], [0.699, 0.333], [0.548, 0.333]], 0.009);
  onFace(parts, "chrome", -1, [[0.39, 0.296], [0.39, 0.302], [-0.39, 0.302], [-0.39, 0.296]], 0.004, false);
  onFace(parts, "trim", -1, [[0.49, 0.27], [0.49, 0.296], [-0.49, 0.296], [-0.49, 0.27]], 0.002, false);
}

function sides(parts) {
  // the mirrors on the doors behind their sails: a shell rounded at the front, the glass behind, an
  // arm, and a repeater along the shell's outer edge
  for (const s of [1, -1]) {
    const y = s * 0.965;
    parts.geometry("paint", new THREE.SphereGeometry(1, 24, 14, Math.PI / 2, Math.PI), new THREE.Matrix4().makeTranslation(0.66, y, 1.058)
                                                                                     .scale(new THREE.Vector3(0.12, 0.085, 0.07)));
    parts.geometry("glass", new THREE.CircleGeometry(1, 24), new THREE.Matrix4().makeTranslation(0.659, y, 1.058).multiply(new THREE.Matrix4().makeRotationY(-Math.PI / 2))
                                                             .scale(new THREE.Vector3(0.062, 0.075, 1)));
    parts.geometry("gloss", new THREE.BoxGeometry(0.09, 0.07, 0.035), new THREE.Matrix4().makeTranslation(0.70, s * 0.885, 1.005));
    parts.geometry("amberLens", new THREE.BoxGeometry(0.075, 0.012, 0.012), new THREE.Matrix4().makeTranslation(0.72, s * 1.042, 1.03));
    parts.geometry(s > 0 ? "turnLeft" : "turnRight", new THREE.BoxGeometry(0.077, 0.014, 0.014), new THREE.Matrix4().makeTranslation(0.72, s * 1.042, 1.03));
    // the door handles, just under the shoulder
    for (const [x, z] of [[0.066, 0.846], [-0.996, 0.863]]) {
      parts.geometry("chrome", new THREE.BoxGeometry(0.17, 0.014, 0.028), new THREE.Matrix4().makeTranslation(x, s * (sideY(x, z) + 0.003), z));
    }
  }
  // the chrome strip low along the doors, rising to the rear
  onSide(parts, "chrome", -0.93, 0.87, (x) => lerp(0.386, 0.334, (x + 0.93) / 1.8), (x) => lerp(0.4, 0.348, (x + 0.93) / 1.8), 0.006, 12);
  // the fuel door, on the left behind the rear wheel
  const fuel = [[-1.74, -1.53], [0.78, 0.93]], line = 0.004;
  onSide(parts, "under", fuel[0][0], fuel[0][1], () => fuel[1][0], () => fuel[1][0] + line, 0.002, 4, [1]);
  onSide(parts, "under", fuel[0][0], fuel[0][1], () => fuel[1][1] - line, () => fuel[1][1], 0.002, 4, [1]);
  for (const x of fuel[0]) onSide(parts, "under", x - line / 2, x + line / 2, () => fuel[1][0], () => fuel[1][1], 0.002, 1, [1]);
  // the shark fin at the back of the roof, and the high brake light along the top of the backlight
  parts.geometry("gloss", new THREE.SphereGeometry(1, 20, 10), new THREE.Matrix4().makeTranslation(-1.145, 0, roofLine(-1.145) + 0.012)
                                                               .scale(new THREE.Vector3(0.085, 0.026, 0.06)));
  const hx = ROOF_REAR - 0.04, slope = (roofLine(hx + 0.02) - roofLine(hx - 0.02)) / 0.04;
  const up = unit([-slope, 0, 1]), hz = roofLine(hx) + 0.004;
  parts.face("stop", [[hx + 0.012, 0.28, hz + slope * 0.012], [hx - 0.012, 0.28, hz - slope * 0.012],
                      [hx - 0.012, -0.28, hz - slope * 0.012], [hx + 0.012, -0.28, hz + slope * 0.012]], up);
}

// A wheel standing at x, y (outer face out, + left), on the ground: the tyre, the rim's lip and
// twenty spokes in pairs over a dark well, the hub, and the brake disc and caliper behind.
function wheel(parts, x, y) {
  const out = Math.sign(y);
  const place = (geometry, local, name) => {
    const m = new THREE.Matrix4().makeTranslation(x, y, TYRE.radius);
    if (out < 0) m.multiply(new THREE.Matrix4().makeRotationZ(Math.PI));
    parts.geometry(name, geometry, m.multiply(local));
  };
  const r = TYRE.radius, rim = TYRE.rim, half = TYRE.width / 2;
  const profile = [[rim, -half + 0.008], [0.29, -half - 0.004], [0.325, -half + 0.006], [r, -half + 0.03], [r, half - 0.03],
                   [0.325, half - 0.006], [0.29, half + 0.004], [rim, half - 0.008]].map(([a, b]) => new THREE.Vector2(a, b));
  place(new THREE.LatheGeometry(profile, 40), new THREE.Matrix4(), "rubber");
  const face = new THREE.Matrix4().makeRotationX(-Math.PI / 2);                 // circles and rings face +y
  place(new THREE.RingGeometry(rim - 0.018, rim + 0.002, 40), new THREE.Matrix4().makeTranslation(0, half - 0.012, 0).multiply(face), "rim");
  place(new THREE.CircleGeometry(rim - 0.016, 40), new THREE.Matrix4().makeTranslation(0, half - 0.05, 0).multiply(face), "rimDark");
  for (let k = 0; k < 20; k++) {
    const angle = 2 * Math.PI * (Math.floor(k / 2) + (k % 2 ? 0.17 : -0.17)) / 10, radial = (rim - 0.018 + 0.058) / 2;
    const m = new THREE.Matrix4().makeRotationY(-angle).multiply(new THREE.Matrix4().makeTranslation(radial, half - 0.03, 0))
      .multiply(new THREE.Matrix4().makeRotationZ(0.1));
    place(new THREE.BoxGeometry(rim - 0.076, 0.016, 0.011), m, "rim");
  }
  place(new THREE.CylinderGeometry(0.058, 0.062, 0.03, 24), new THREE.Matrix4().makeTranslation(0, half - 0.042, 0), "rim");
  place(new THREE.CircleGeometry(0.032, 24), new THREE.Matrix4().makeTranslation(0, half - 0.026, 0).multiply(face), "gloss");
  place(new THREE.CylinderGeometry(0.165, 0.165, 0.028, 32), new THREE.Matrix4().makeTranslation(0, 0.02, 0), "disc");
  place(new THREE.BoxGeometry(0.11, 0.05, 0.08), new THREE.Matrix4().makeTranslation(-0.09, 0.04, 0.12).multiply(new THREE.Matrix4().makeRotationY(0.6)), "trim");
}

// -- materials ---------------------------------------------------------------------------------------
function materials([dark, light]) {
  const env = {envMap: dark, envMapIntensity: 1}, shine = {envMap: light, envMapIntensity: 1};
  const unlit = (color) => new THREE.MeshBasicMaterial({color, toneMapped: false}), turn = unlit("#ffa31a");
  return {
    // Aurora Black Pearl: black under a clear coat that mirrors the studio
    paint: new THREE.MeshPhysicalMaterial({color: "#0a0a0c", metalness: 0, roughness: 0.5, clearcoat: 1, clearcoatRoughness: 0.035, ...env}),
    gloss: new THREE.MeshPhysicalMaterial({color: "#040405", metalness: 0, roughness: 0.25, clearcoat: 1, clearcoatRoughness: 0.02, ...env}),
    trim: new THREE.MeshStandardMaterial({color: "#16181b", roughness: 0.8}),
    under: new THREE.MeshStandardMaterial({color: "#0b0c0d", roughness: 0.95}),
    grille: new THREE.MeshStandardMaterial({color: "#060708", roughness: 0.6}),
    glass: new THREE.MeshPhysicalMaterial({color: "#05080c", metalness: 0, roughness: 0.02, clearcoat: 1, ...env}),
    chrome: new THREE.MeshStandardMaterial({color: "#f2f4f7", metalness: 1, roughness: 0.1, ...shine, envMapIntensity: 1.2}),
    lens: new THREE.MeshStandardMaterial({color: "#9aa3ad", metalness: 0.85, roughness: 0.32, ...shine}),     // reflectors behind the glass
    reverse: new THREE.MeshStandardMaterial({color: "#c3c8ce", metalness: 0.9, roughness: 0.22, ...shine}),
    amberLens: new THREE.MeshStandardMaterial({color: "#6b3a0c", metalness: 0.3, roughness: 0.3, ...shine}),
    rubber: new THREE.MeshStandardMaterial({color: "#161718", roughness: 0.92}),
    rim: new THREE.MeshStandardMaterial({color: "#d0d4da", metalness: 0.9, roughness: 0.25, ...shine}),
    rimDark: new THREE.MeshStandardMaterial({color: "#22252a", metalness: 0.6, roughness: 0.45, ...shine}),
    disc: new THREE.MeshStandardMaterial({color: "#55595f", metalness: 0.7, roughness: 0.5}),
    plate: new THREE.MeshStandardMaterial({color: "#f1f1ee", roughness: 0.5}),
    drl: unlit("#f2f6ff"), headlight: unlit("#fff4dc"), fog: unlit("#c8cfd9"), amber: unlit("#ff9a1f"),
    tailLens: unlit(BRAKE_LAMPS.tailLens[0]), tail: unlit(BRAKE_LAMPS.tail[0]), stop: unlit(BRAKE_LAMPS.stop[0]),
    reflector: unlit("#7c0f0e"), turnLeft: turn, turnRight: turn,
  };
}

// The lamps braking changes: [tail lamps lit, braking]. The high brake light is dark until then.
const BRAKE_LAMPS = {tailLens: ["#4a0708", "#a8130e"], tail: ["#e8261c", "#ff7a66"], stop: ["#2e0606", "#ff3a2a"]};

// The studio the car reflects: a dark room (the BEV's) ringed by a band of bright windows at eye
// height, with two long light strips overhead and one along each side, as a showroom lights a black
// car: the hood, the roof and the deck stay black under two thin highlights. Upright faces seen from
// above mirror the floor: the paint and the glass get a dark one, so the sides stay black under the
// highlights; the chrome, the wheels and the lamps' reflectors a light one round a dark patch under
// the car, or they would read black from every view the BEV has. Made once per renderer, for each
// floor.
function studioScene(lightFloor) {
  const scene = new THREE.Scene();
  const room = new THREE.Mesh(new THREE.BoxGeometry(40, 40, 20), new THREE.MeshBasicMaterial({color: "#0b0e13", side: THREE.BackSide}));
  const floor = new THREE.Mesh(new THREE.RingGeometry(2.1, 40, 48), new THREE.MeshBasicMaterial({color: lightFloor ? "#7d848f" : "#20242b",
                                                                                                 side: THREE.DoubleSide}));
  const under = new THREE.Mesh(new THREE.CircleGeometry(2.1, 48), new THREE.MeshBasicMaterial({color: "#14171c", side: THREE.DoubleSide}));
  floor.position.z = under.position.z = -1.2;                     // the light floor from 30 degrees below the horizon outwards
  const canvas = Object.assign(document.createElement("canvas"), {width: 512, height: 8});
  const c = canvas.getContext("2d");
  c.fillStyle = "#1a1d22";
  c.fillRect(0, 0, 512, 8);
  c.fillStyle = "#ffffff";
  for (let k = 0; k < 14; k++) c.fillRect(k * 512 / 14 + 6, 0, 512 / 14 - 12, 8);       // windows between pillars
  const windows = new THREE.Mesh(new THREE.CylinderGeometry(14, 14, 2.4, 56, 1, true),
                                 new THREE.MeshBasicMaterial({map: new THREE.CanvasTexture(canvas), color: new THREE.Color(1.1, 1.1, 1.1),
                                                              side: THREE.BackSide}));
  windows.rotation.x = Math.PI / 2;                               // its axis up (z)
  windows.position.z = 1.4;
  scene.add(room, floor, under, windows);
  const panel = (w, h, position, rotation, intensity) => {
    const light = new THREE.Mesh(new THREE.PlaneGeometry(w, h), new THREE.MeshBasicMaterial({
      color: new THREE.Color(1, 1, 1).multiplyScalar(intensity), side: THREE.DoubleSide}));
    light.position.set(...position);
    light.rotation.set(...rotation);
    scene.add(light);
  };
  for (const s of [1, -1]) {
    panel(9, 0.4, [0, s * 0.9, 7], [0, 0, 0], 3);                     // overhead, along the car
    panel(14, 0.6, [0, s * 8, 2.6], [Math.PI / 2, 0, 0], 3.2);         // a strip along each side
    panel(0.6, 6, [s * 9, 0, 3], [0, Math.PI / 2, 0], 1.6);            // and fore and aft
  }
  return scene;
}

const studios = new WeakMap();
function studio(renderer) {
  if (!studios.has(renderer)) {
    const generator = new THREE.PMREMGenerator(renderer);
    studios.set(renderer, [false, true].map((light) => generator.fromScene(studioScene(light), 0.02).texture));
    generator.dispose();
  }
  return studios.get(renderer);
}

// The K7 as a group of meshes, one per material, for the renderer that draws it. Its lamps switch
// with group.setLamps({left, right, brake}): the turn signals' lit lenses show while they are on, and
// braking brightens the tail lamps and lights the high brake light. It returns whether anything changed.
export function makeK7(renderer) {
  const parts = new Parts();
  body(parts);
  front(parts);
  rear(parts);
  sides(parts);
  AXLES.forEach((x, a) => { for (const s of [1, -1]) wheel(parts, x, s * TRACKS[a]); });
  const lit = materials(studio(renderer)), group = parts.meshes(lit);
  group.name = "K7";
  const turns = ["turnLeft", "turnRight"].map((name) => group.getObjectByName(name));
  let now = null;
  group.setLamps = ({left = false, right = false, brake = false} = {}) => {
    const state = `${+left}${+right}${+brake}`;
    if (state === now) return false;
    now = state;
    turns[0].visible = left;
    turns[1].visible = right;
    for (const [name, colors] of Object.entries(BRAKE_LAMPS)) lit[name].color.set(colors[brake ? 1 : 0]);
    return true;
  };
  group.setLamps();
  return group;
}
