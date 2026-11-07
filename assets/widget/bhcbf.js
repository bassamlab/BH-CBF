// BH-CBF for one vehicle, as in the paper and its Python implementation
// (SigmaRL: sigmarl/backup_horizon_cbf.py, sigmarl/lane_sat_barrier.py, and
// the backup of sigmarl/cbf_qp_proxsuite.py). check_model.mjs compares this
// module with reference values of the Python code.
//
// Conventions: a state is [x, y, psi, v, delta], a control [a, omega], a pose
// [x, y, psi]. A rollout from step k returns the poses at t = 1, ..., N + 1 of
// the paper at the indices 0, ..., N.

/** Parameters of the paper's evaluation (full-scale vehicle). */
export const PARAMS = Object.freeze({
  dt: 0.1,
  aMin: -6.0, aMax: 3.0, // Acceleration [m/s^2]
  wMin: -1.0, wMax: 1.0, // Steering rate [rad/s]
  vMin: 0.0, vMax: 14.0, // Speed [m/s]
  dMin: -0.6108652381980153, dMax: 0.6108652381980153, // Steering angle (35 deg) [rad]
  lr: 1.35, lwb: 2.7, // Rear-axle distance, wheelbase [m]
  length: 4.6, halfL: 2.3, halfW: 1.0,
  epsLane: 0.02, epsVeh: 0.1, // Clearances [m]
  tol: 4e-4, // Tolerance of the exact check [m]
  kDelta: 4.0, // Steering-rate gain of the backup [1/s]
  lookGain: 0.2, // Look-ahead of the backup: max(length, 2 length + lookGain v) [m]
  windowBack: 5, windowFront: 6, // Boundary vertices around the lane window
  extension: 20.0, // Straight extension of the boundaries at the route ends [m]
  windowMargin: 1.0, // Extra route distance of the lane window [m]
  weight: [0.1, 1.0], // Diagonal of W
});

const TWO_PI = 2 * Math.PI;

/** Angle in [-pi, pi), as Python's (a + pi) % (2 pi) - pi. */
export function wrap(a) {
  let m = (a + Math.PI) % TWO_PI;
  if (m < 0) m += TWO_PI;
  return m - Math.PI;
}

const clip = (x, lo, hi) => Math.min(Math.max(x, lo), hi);

/** Index of the last element of a sorted array that is <= x (-1 if none). */
function lastAtOrBelow(sorted, x) {
  let lo = 0, hi = sorted.length; // First index with sorted[i] > x
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (sorted[mid] <= x) lo = mid + 1; else hi = mid;
  }
  return lo - 1;
}

/** numpy.interp for one value. */
function interp(x, xp, fp) {
  const n = xp.length;
  if (x <= xp[0]) return fp[0];
  if (x >= xp[n - 1]) return fp[n - 1];
  const j = lastAtOrBelow(xp, x);
  const slope = (fp[j + 1] - fp[j]) / (xp[j + 1] - xp[j]);
  return slope * (x - xp[j]) + fp[j];
}

function cumulative(points) {
  const cum = new Float64Array(points.length);
  for (let i = 1; i < points.length; i++) {
    cum[i] = cum[i - 1] + Math.hypot(points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1]);
  }
  return cum;
}

/** A route: its centerline (for the backup and the progress) and its left
 * and right boundaries (for the lane margins), aligned by index. */
export function makeRoute(center, left, right, isLoop = false) {
  const n = center.length;
  const cum = cumulative(center);
  const segX = new Float64Array(n - 1), segY = new Float64Array(n - 1), lenSq = new Float64Array(n - 1);
  for (let i = 0; i < n - 1; i++) {
    segX[i] = center[i + 1][0] - center[i][0];
    segY[i] = center[i + 1][1] - center[i][1];
    lenSq[i] = Math.max(segX[i] * segX[i] + segY[i] * segY[i], 1e-12);
  }
  return {
    center, left, right, isLoop, cum, total: cum[n - 1], segX, segY, lenSq,
    px: Float64Array.from(center, (p) => p[0]), py: Float64Array.from(center, (p) => p[1]),
  };
}

/** Bounds [[aLo, wLo], [aHi, wHi]] of the controls that keep the next speed
 * and steering angle within their limits. */
export function inputBounds(x, P = PARAMS) {
  return [
    [Math.max((P.vMin - x[3]) / P.dt, P.aMin), Math.max((P.dMin - x[4]) / P.dt, P.wMin)],
    [Math.min((P.vMax - x[3]) / P.dt, P.aMax), Math.min((P.dMax - x[4]) / P.dt, P.wMax)],
  ];
}

export function saturate(x, u, P = PARAMS) {
  const [lo, hi] = inputBounds(x, P);
  return [Math.min(Math.max(u[0], lo[0]), hi[0]), Math.min(Math.max(u[1], lo[1]), hi[1])];
}

/** One explicit-Euler step of the kinematic bicycle (in place into out). */
export function eulerStep(x, u, P = PARAMS, out = new Array(5)) {
  const tanS = Math.tan(x[4]);
  const beta = Math.atan((P.lr / P.lwb) * tanS);
  const heading = x[2] + beta;
  const dtv = P.dt * x[3];
  out[0] = x[0] + dtv * Math.cos(heading);
  out[1] = x[1] + dtv * Math.sin(heading);
  out[2] = x[2] + (dtv / P.lwb) * tanS * Math.cos(beta);
  out[3] = x[3] + P.dt * u[0];
  out[4] = x[4] + P.dt * u[1];
  return out;
}

/** Station [m] of the projection of a point on the closest centerline
 * segment (the first one on ties). */
export function closestStation(route, px, py) {
  let best = Infinity, bestSeg = 0, bestTau = 0;
  const { center, segX, segY, lenSq } = route;
  for (let i = 0; i < segX.length; i++) {
    const rx = px - center[i][0], ry = py - center[i][1];
    const tau = clip((rx * segX[i] + ry * segY[i]) / lenSq[i], 0, 1);
    const gx = rx - tau * segX[i], gy = ry - tau * segY[i];
    const d = gx * gx + gy * gy;
    if (d < best) { best = d; bestSeg = i; bestTau = tau; }
  }
  return route.cum[bestSeg] + bestTau * (route.cum[bestSeg + 1] - route.cum[bestSeg]);
}

/** Pure-pursuit steering angle of the backup: toward the centerline point
 * one look-ahead ahead of the vehicle's projection. */
export function pursuitAngle(x, route, P = PARAMS) {
  const v = Math.max(0, x[3]);
  const look = Math.max(P.length, 2 * P.length + P.lookGain * v);
  let target = closestStation(route, x[0], x[1]) + look;
  if (route.isLoop && route.total > 1e-9) target = ((target % route.total) + route.total) % route.total;
  target = clip(target, 0, route.total);
  const tx = interp(target, route.cum, route.px), ty = interp(target, route.cum, route.py);
  const alpha = wrap(Math.atan2(ty - x[1], tx - x[0]) - x[2]);
  return clip(Math.atan2(2 * P.lwb * Math.sin(alpha), look), P.dMin, P.dMax);
}

/** Backup control: maximum braking without crossing zero speed, and the
 * pure-pursuit angle as a steering rate with gain kDelta; saturated. */
export function backupControl(x, route, P = PARAMS) {
  const a = clip(-x[3] / P.dt, P.aMin, P.aMax);
  const w = clip(P.kDelta * wrap(pursuitAngle(x, route, P) - x[4]), P.wMin, P.wMax);
  return saturate(x, [a, w], P);
}

/** Steps after which a vehicle at speed v has stopped in every candidate
 * prediction (stopping horizon N of the paper). */
export function stoppingHorizon(v, P = PARAMS) {
  const steps = Math.max(Math.max(v, 0) / (-P.aMin * P.dt), Math.max(-v, 0) / (P.aMax * P.dt));
  return Math.ceil(steps) + 2;
}

/** Candidate prediction: apply u at x, then the backup for nSteps steps.
 * Returns poses [nSteps + 1][3] (t = 1, ..., N + 1) and the speeds. */
export function rollout(x0, u, route, nSteps, P = PARAMS) {
  let x = eulerStep(x0, saturate(x0, u, P), P);
  const poses = new Array(nSteps + 1), speeds = new Float64Array(nSteps + 1);
  for (let t = 0; t <= nSteps; t++) {
    poses[t] = [x[0], x[1], x[2]];
    speeds[t] = x[3];
    if (t === nSteps) break;
    x = eulerStep(x, backupControl(x, route, P), P);
  }
  return { poses, speeds };
}

// ---------------------------------------------------------------------------
// Margins

function dedupe(points) {
  const out = [points[0]];
  for (let i = 1; i < points.length; i++) {
    const p = points[i], q = out[out.length - 1];
    if (Math.hypot(p[0] - q[0], p[1] - q[1]) > 1e-9) out.push(p);
  }
  return out;
}

/** The boundary vertices [left, right] of the lane window around a route
 * progress [m], extended by `ahead` [m] (LaneSatBarrier.boundaries_at). */
export function boundariesAt(route, progress, ahead, P = PARAMS) {
  const { cum } = route, n = cum.length;
  if (route.isLoop) progress = ((progress % cum[n - 1]) + cum[n - 1]) % cum[n - 1];
  const i0 = lastAtOrBelow(cum, progress);
  let i1 = i0;
  if (ahead > 0) {
    let extended = cum;
    if (route.isLoop) {
      extended = Float64Array.from([...cum, ...Array.from(cum.subarray(1), (c) => cum[n - 1] + c)]);
    }
    i1 = Math.max(i0, lastAtOrBelow(extended, progress + ahead));
  }
  const first = i0 - P.windowBack, last = i1 + P.windowFront;
  return [route.left, route.right].map((polyline) => {
    const raw = [];
    for (let i = first; i <= last; i++) {
      raw.push(route.isLoop ? polyline[((i % (n - 1)) + (n - 1)) % (n - 1)] : polyline[clip(i, 0, n - 1)]);
    }
    let points = dedupe(raw);
    if (!route.isLoop) {
      if (first < 0) {
        const tx = points[1][0] - points[0][0], ty = points[1][1] - points[0][1], l = Math.hypot(tx, ty);
        points = [[points[0][0] - (P.extension * tx) / l, points[0][1] - (P.extension * ty) / l], ...points];
      }
      if (last > n - 1) {
        const m = points.length;
        const tx = points[m - 1][0] - points[m - 2][0], ty = points[m - 1][1] - points[m - 2][1], l = Math.hypot(tx, ty);
        points = [...points, [points[m - 1][0] + (P.extension * tx) / l, points[m - 1][1] + (P.extension * ty) / l]];
      }
    }
    return points;
  });
}

/** Facets of a boundary polyline with lane-pointing unit normals (side 0:
 * left boundary, 1: right) and their buffers: the clearance eps plus, if
 * resampling, the resampling buffer of the simulator's collision checker
 * (LaneSatBarrier.facets; the widget uses eps alone, as the paper). */
export function makeFacets(points, side, eps, resampling = false) {
  const f = { sx: [], sy: [], ex: [], ey: [], nx: [], ny: [], offset: [], buffer: [], side };
  const tangents = [], lengths = [];
  for (let i = 0; i + 1 < points.length; i++) {
    const dx = points[i + 1][0] - points[i][0], dy = points[i + 1][1] - points[i][1];
    const l = Math.sqrt(dx * dx + dy * dy); // As numpy.linalg.norm
    if (!(l > 1e-9)) continue;
    const tx = dx / l, ty = dy / l;
    f.sx.push(points[i][0]); f.sy.push(points[i][1]);
    f.ex.push(points[i][0] + dx); f.ey.push(points[i][1] + dy);
    if (side === 0) { f.nx.push(ty); f.ny.push(-tx); } else { f.nx.push(-ty); f.ny.push(tx); }
    tangents.push([tx, ty]); lengths.push(l);
  }
  const m = f.sx.length;
  const buffer = new Array(m).fill(eps);
  if (resampling && m > 1) {
    // Gap at the vertex between facets i and i + 1, charged to both facets
    const gaps = [];
    for (let i = 0; i + 1 < m; i++) {
      const [t0x, t0y] = tangents[i], [t1x, t1y] = tangents[i + 1];
      const turn = Math.acos(clip(t0x * t1x + t0y * t1y, -1, 1));
      const toLane = (t1x - t0x) * f.nx[i] + (t1y - t0y) * f.ny[i];
      const l1 = lengths[i], l2 = lengths[i + 1];
      gaps.push(toLane > 0 ? (2 * Math.sin(0.5 * turn) * l1 * l2) / (Math.sqrt(l1) + Math.sqrt(l2)) ** 2 : 0);
    }
    for (let i = 0; i < m; i++) buffer[i] += Math.max(i < m - 1 ? gaps[i] : 0, i > 0 ? gaps[i - 1] : 0);
  }
  for (let i = 0; i < m; i++) f.offset.push(f.nx[i] * f.sx[i] + f.ny[i] * f.sy[i] + buffer[i]);
  f.buffer = buffer;
  return f;
}

/** Lane margin h_s of one boundary side at a pose: the smallest facet margin,
 * each the largest of four projection gaps (paper, Sec. "Margins"). */
export function laneMargin(f, pose, P = PARAMS) {
  const c = Math.cos(pose[2]), s = Math.sin(pose[2]);
  const hl = P.halfL, hw = P.halfW;
  // Corners p + (+-hl) e_lon + (+-hw) e_lat, with e_lon = (c, s), e_lat = (-s, c)
  const cx = [pose[0] + hl * c - hw * s, pose[0] + hl * c + hw * s, pose[0] - hl * c - hw * s, pose[0] - hl * c + hw * s];
  const cy = [pose[1] + hl * s + hw * c, pose[1] + hl * s - hw * c, pose[1] - hl * s + hw * c, pose[1] - hl * s - hw * c];
  const sign = f.side === 0 ? 1 : -1;
  let h = Infinity;
  for (let j = 0; j < f.sx.length; j++) {
    const nx = f.nx[j], ny = f.ny[j], off = f.offset[j], buf = f.buffer[j];
    let normal = Infinity;
    for (let m = 0; m < 4; m++) normal = Math.min(normal, nx * cx[m] + ny * cy[m] - off);
    const rsx = f.sx[j] - pose[0], rsy = f.sy[j] - pose[1], rex = f.ex[j] - pose[0], rey = f.ey[j] - pose[1];
    const lonS = rsx * c + rsy * s, lonE = rex * c + rey * s;
    const latS = -rsx * s + rsy * c, latE = -rex * s + rey * c;
    const latMin = sign > 0 ? Math.min(latS, latE) : -Math.max(latS, latE);
    const value = Math.max(
      Math.max(normal, Math.min(lonS, lonE) - hl - buf),
      Math.max(-Math.max(lonS, lonE) - hl - buf, latMin - hw - buf),
    );
    if (value < h) h = value;
  }
  return h;
}

/** Corners [4][2] of the rectangle with half sizes half at a pose. */
export function corners(pose, half = [PARAMS.halfL, PARAMS.halfW]) {
  const c = Math.cos(pose[2]), s = Math.sin(pose[2]);
  return [[1, 1], [1, -1], [-1, -1], [-1, 1]].map(([a, b]) => [
    pose[0] + a * half[0] * c - b * half[1] * s,
    pose[1] + a * half[0] * s + b * half[1] * c,
  ]);
}

const SIGNS_L = [1, 1, -1, -1], SIGNS_W = [1, -1, -1, 1];

/** Corners of a rectangle into the arrays cx, cy [4] (counterclockwise). */
function cornersInto(pose, half, cx, cy) {
  const c = Math.cos(pose[2]), s = Math.sin(pose[2]);
  for (let m = 0; m < 4; m++) {
    const l = SIGNS_L[m] * half[0], w = SIGNS_W[m] * half[1];
    cx[m] = pose[0] + l * c - w * s;
    cy[m] = pose[1] + l * s + w * c;
  }
}

/** Shortest vector from the edges of polygon (qx, qy) to the corners (px,
 * py), times sign, compared against the best so far (out: [length, x, y]). */
function cornerToEdge(px, py, qx, qy, sign, out) {
  for (let e = 0; e < 4; e++) {
    const sx = qx[(e + 1) % 4] - qx[e], sy = qy[(e + 1) % 4] - qy[e];
    const lenSq = sx * sx + sy * sy;
    for (let c = 0; c < 4; c++) {
      const rx = px[c] - qx[e], ry = py[c] - qy[e];
      const tau = clip((rx * sx + ry * sy) / lenSq, 0, 1);
      const vx = rx - tau * sx, vy = ry - tau * sy, l = Math.sqrt(vx * vx + vy * vy);
      if (l < out[0]) { out[0] = l; out[1] = sign * vx; out[2] = sign * vy; }
    }
  }
}

const AX = new Float64Array(4), AY = new Float64Array(4), BX = new Float64Array(4), BY = new Float64Array(4);
const CLOSEST = new Float64Array(3);

/** Signed distance of two rectangles (equal half sizes) and the separating
 * normal from b to a: the Euclidean distance of disjoint rectangles, minus
 * the penetration depth of overlapping ones. */
export function signedDistance(poseA, poseB, half = [PARAMS.halfL, PARAMS.halfW]) {
  cornersInto(poseA, half, AX, AY);
  cornersInto(poseB, half, BX, BY);
  let best = -Infinity, wx = 0, wy = 0;
  for (const p of [poseA, poseB]) {
    const c = Math.cos(p[2]), s = Math.sin(p[2]);
    for (const [ax, ay] of [[c, s], [-s, c]]) {
      let minA = Infinity, maxA = -Infinity, minB = Infinity, maxB = -Infinity;
      for (let m = 0; m < 4; m++) {
        const pa = ax * AX[m] + ay * AY[m], pb = ax * BX[m] + ay * BY[m];
        if (pa < minA) minA = pa;
        if (pa > maxA) maxA = pa;
        if (pb < minB) minB = pb;
        if (pb > maxB) maxB = pb;
      }
      const aAhead = minA - maxB, bAhead = minB - maxA, gap = Math.max(aAhead, bAhead);
      if (gap > best) {
        best = gap;
        if (aAhead >= bAhead) { wx = ax; wy = ay; } else { wx = -ax; wy = -ay; }
      }
    }
  }
  if (!(best > 0)) return { distance: best, witness: [wx, wy] };
  // Disjoint: the Euclidean distance, attained between a corner and an edge
  CLOSEST[0] = Infinity;
  cornerToEdge(AX, AY, BX, BY, 1, CLOSEST);
  cornerToEdge(BX, BY, AX, AY, -1, CLOSEST);
  const d = Math.max(CLOSEST[0], 1e-12);
  return { distance: CLOSEST[0], witness: [CLOSEST[1] / d, CLOSEST[2] / d] };
}

/** Whether the rectangle at a pose intersects the segment [p, q] (for the
 * collision check of the simulation, without clearance). */
export function rectangleHitsSegment(pose, p, q, P = PARAMS) {
  const c = Math.cos(pose[2]), s = Math.sin(pose[2]);
  const local = (pt) => {
    const dx = pt[0] - pose[0], dy = pt[1] - pose[1];
    return [dx * c + dy * s, -dx * s + dy * c];
  };
  // Clip the segment (in the vehicle frame) against the box (Liang-Barsky)
  const [x0, y0] = local(p), [x1, y1] = local(q);
  let t0 = 0, t1 = 1;
  const dx = x1 - x0, dy = y1 - y0;
  for (const [pp, qq] of [[-dx, x0 + P.halfL], [dx, P.halfL - x0], [-dy, y0 + P.halfW], [dy, P.halfW - y0]]) {
    if (pp === 0) { if (qq < 0) return false; continue; }
    const r = qq / pp;
    if (pp < 0) { if (r > t1) return false; if (r > t0) t0 = r; } else { if (r < t0) return false; if (r < t1) t1 = r; }
  }
  return true;
}

// ---------------------------------------------------------------------------
// Filter problems

/** The filter problem of the vehicle at state x at step k.
 *
 * ctx: { route, progress [m], obstacles: poses of stopped vehicles, lambda,
 *        condition: "bh" (BH-CBF) or "one-step" }.
 * Constraints e: the left (L) and right (R) lane sides of the route and one
 * pair constraint (V) per stopped vehicle. A stopped vehicle stays at its
 * pose, so a pair constraint involves only the controlled vehicle.
 */
export function makeProblem(x, ctx, P = PARAMS) {
  const { route, obstacles = [], lambda } = ctx;
  const v1 = Math.max(0, x[3] + P.dt * P.aMax);
  const reach = 2 * v1 * P.dt + (v1 * v1) / (2 * -P.aMin) + P.windowMargin;
  const [left, right] = boundariesAt(route, ctx.progress, reach, P);
  const constraints = [
    { kind: "lane", label: "left boundary", facets: makeFacets(left, 0, P.epsLane) },
    { kind: "lane", label: "right boundary", facets: makeFacets(right, 1, P.epsLane) },
    ...obstacles.map((pose) => ({ kind: "pair", label: "stopped vehicle", pose })),
  ];
  const nSteps = stoppingHorizon(x[3], P);
  const problem = { x: x.slice(), ctx, P, constraints, nSteps, bounds: inputBounds(x, P), condition: ctx.condition };
  problem.backupU = backupControl(x, route, P);
  problem.backup = predict(problem, problem.backupU);
  for (let e = 0; e < constraints.length; e++) {
    const h = problem.backup.values[e];
    const B = Math.min(...h); // Backup value B_e(x_k)
    constraints[e].B = B;
    // Right-hand side of the condition: from t = 2 on (BH-CBF), or at t = 2
    // with the margin of the next pose (one-step)
    constraints[e].rhs = ctx.condition === "one-step" ? (1 - lambda) * h[0] : (1 - lambda) * B;
  }
  return problem;
}

/** Margin of constraint e at a pose. */
function margin(problem, e, pose) {
  const con = problem.constraints[e];
  if (con.kind === "lane") return laneMargin(con.facets, pose, problem.P);
  return signedDistance(pose, con.pose).distance - problem.P.epsVeh;
}

/** Candidate prediction of a control u and the margins [E][T] along it. */
export function predict(problem, u) {
  const { poses, speeds } = rollout(problem.x, u, problem.ctx.route, problem.nSteps, problem.P);
  const values = problem.constraints.map((_, e) => poses.map((pose) => margin(problem, e, pose)));
  return { u: saturate(problem.x, u, problem.P), poses, speeds, values };
}

/** Smallest slack of each constraint's inequalities for a candidate
 * prediction (negative: violated), and the step t (paper) where it occurs. */
export function slacks(problem, prediction) {
  return problem.constraints.map((con, e) => {
    const h = prediction.values[e];
    if (problem.condition === "one-step") return { slack: h[1] - con.rhs, t: 2 };
    let slack = Infinity, t = 2;
    for (let i = 1; i < h.length; i++) if (h[i] - con.rhs < slack) { slack = h[i] - con.rhs; t = i + 1; }
    return { slack, t };
  });
}

/** Smallest slack over all constraints of a control (and the binding constraint). */
export function evaluate(problem, u) {
  const prediction = predict(problem, u);
  const s = slacks(problem, prediction);
  let worst = 0;
  for (let e = 1; e < s.length; e++) if (s[e].slack < s[worst].slack) worst = e;
  return { prediction, slacks: s, slack: s[worst].slack, worst };
}

const feasible = (problem, result) => result.slack >= -problem.P.tol;

function distanceW(P, u, v) {
  return P.weight[0] * (u[0] - v[0]) ** 2 + P.weight[1] * (u[1] - v[1]) ** 2;
}

/** Exact smallest slack on a G x G grid over the admissible box (control
 * plane). Returns { a, w, slack: Float64Array(G * G) (row: a), worst }. */
export function slackGrid(problem, G) {
  const [lo, hi] = problem.bounds;
  const a = Array.from({ length: G }, (_, i) => lo[0] + ((hi[0] - lo[0]) * i) / (G - 1));
  const w = Array.from({ length: G }, (_, j) => lo[1] + ((hi[1] - lo[1]) * j) / (G - 1));
  const slack = new Float64Array(G * G), worst = new Int8Array(G * G);
  for (let i = 0; i < G; i++) {
    for (let j = 0; j < G; j++) {
      const r = evaluate(problem, [a[i], w[j]]);
      slack[i * G + j] = r.slack;
      worst[i * G + j] = r.worst;
    }
  }
  return { G, a, w, slack, worst };
}

/** The filter: the admissible control closest to u (weighted by W) whose
 * candidate prediction satisfies the condition exactly.
 *
 * The widget solves the two-dimensional filter problem by search: the exact
 * check of u, then a grid over the admissible box (grid: a slackGrid of this
 * problem, or the number of grid points per control), a finer local grid
 * around the closest feasible point, and a bisection toward u. Every
 * returned control passes the exact check, except the status "infeasible".
 * Without a feasible control, BH-CBF applies the backup control (which is
 * feasible from every recoverable state), and the one-step filter relaxes
 * its condition: it applies the control with the smallest violations.
 */
export function filter(problem, uNominal, grid = 21) {
  const P = problem.P;
  const u = saturate(problem.x, uNominal, P);
  const nominal = evaluate(problem, u);
  if (feasible(problem, nominal)) return { u, status: "accepted", result: nominal };
  const g = typeof grid === "number" ? slackGrid(problem, grid) : grid;
  let best = null, bestD = Infinity;
  for (let i = 0; i < g.G; i++) {
    for (let j = 0; j < g.G; j++) {
      if (g.slack[i * g.G + j] < -P.tol) continue;
      const d = distanceW(P, [g.a[i], g.w[j]], u);
      if (d < bestD) { bestD = d; best = [g.a[i], g.w[j]]; }
    }
  }
  if (!best) {
    if (problem.condition !== "one-step") {
      const backup = evaluate(problem, problem.backupU);
      return { u: problem.backupU, status: feasible(problem, backup) ? "fallback" : "unrecoverable", result: backup };
    }
    return relaxed(problem, u, g);
  }
  // Finer grid around the closest feasible grid point
  const da = (g.a[g.G - 1] - g.a[0]) / (g.G - 1), dw = (g.w[g.G - 1] - g.w[0]) / (g.G - 1);
  const [lo, hi] = problem.bounds;
  let bestResult = null;
  const K = 7;
  for (let i = 0; i < K; i++) {
    for (let j = 0; j < K; j++) {
      const c = [clip(best[0] + da * (i / (K - 1) - 0.5) * 2, lo[0], hi[0]), clip(best[1] + dw * (j / (K - 1) - 0.5) * 2, lo[1], hi[1])];
      const d = distanceW(P, c, u);
      if (d >= bestD && bestResult) continue;
      const r = evaluate(problem, c);
      if (feasible(problem, r) && (d < bestD || !bestResult)) { bestD = d; best = c; bestResult = r; }
    }
  }
  if (!bestResult) bestResult = evaluate(problem, best);
  // Bisection from the closest feasible point toward u
  let sLo = 0, sHi = 1; // u + s (best - u): s = 1 feasible
  for (let it = 0; it < 14; it++) {
    const s = 0.5 * (sLo + sHi);
    const c = [u[0] + s * (best[0] - u[0]), u[1] + s * (best[1] - u[1])];
    const r = evaluate(problem, c);
    if (feasible(problem, r)) { sHi = s; bestResult = r; } else sLo = s;
  }
  const s = sHi;
  return { u: [u[0] + s * (best[0] - u[0]), u[1] + s * (best[1] - u[1])], status: "modified", result: bestResult };
}

/** One-step filter without a feasible control: the control of the grid with
 * the smallest squared violations (the QP's heavily penalized slack). */
function relaxed(problem, u, g) {
  let best = null, bestCost = Infinity;
  for (let i = 0; i < g.G; i++) {
    for (let j = 0; j < g.G; j++) {
      const c = [g.a[i], g.w[j]];
      const s = slacks(problem, predict(problem, c));
      const cost = s.reduce((sum, { slack }) => sum + Math.min(slack, 0) ** 2, 0) * 1e6 + distanceW(problem.P, c, u);
      if (cost < bestCost) { bestCost = cost; best = c; }
    }
  }
  return { u: best, status: "infeasible", result: evaluate(problem, best) };
}
