// Closed loop of the widget: the visitor's driving commands as the nominal
// controls, the selected filter, the vehicle model, and the collision check.

import {
  PARAMS, boundariesAt, closestStation, eulerStep, filter, makeProblem,
  makeRoute, pursuitAngle, rectangleHitsSegment, saturate, signedDistance, slackGrid, wrap, evaluate,
} from "./bhcbf.js";

export const FILTERS = ["off", "one-step", "bh"];

/** Pose [x, y, psi] at a station [m] of the route's centerline. */
export function poseAt(route, station) {
  const { cum, center } = route;
  let i = 0;
  while (i < cum.length - 2 && cum[i + 1] < station) i++;
  const f = Math.min(Math.max((station - cum[i]) / (cum[i + 1] - cum[i]), 0), 1);
  const p = center[i], q = center[i + 1];
  return [p[0] + f * (q[0] - p[0]), p[1] + f * (q[1] - p[1]), Math.atan2(q[1] - p[1], q[0] - p[0])];
}

export class Simulation {
  /**
   * map: roundabout.json. options: start and obstacle stations [m] (obstacle
   * null: none), initial speed [m/s], and grid points per control of the
   * filter's search (see bhcbf.filter).
   */
  constructor(map, { start = 8, speed = 8, obstacle = null, grid = 21 } = {}) {
    this.route = makeRoute(map.route.center, map.route.left, map.route.right, map.route.is_loop);
    this.options = { start, speed, obstacle, grid };
    this.lambda = 0.5;
    this.mode = "bh";
    this.reset();
  }

  setObstacle(station) {
    this.options.obstacle = station;
    this.obstacles = station == null ? [] : [poseAt(this.route, station)];
  }

  reset() {
    const pose = poseAt(this.route, this.options.start);
    this.x = [pose[0], pose[1], pose[2], this.options.speed, 0];
    this.progress = this.options.start;
    this.k = 0;
    this.outcome = null; // { kind: "collision" | "stopped" | "end", ... }
    this.setObstacle(this.options.obstacle);
    this.history = [{ x: this.x.slice() }];
    this.current = null;
  }

  /** The visitor's nominal control: acceleration from the up and down keys
   * (none: keep the speed), and a steering angle that tracks the route
   * centerline (as the backup) plus an offset while left or right is held;
   * converted to a control as the paper converts the policy's actions. */
  nominal(keys) {
    const P = PARAMS;
    const a = keys.up ? P.aMax : keys.down ? P.aMin : 0;
    const offset = keys.left ? 0.35 : keys.right ? -0.35 : 0;
    const deltaRef = Math.min(Math.max(pursuitAngle(this.x, this.route) + offset, P.dMin), P.dMax);
    return saturate(this.x, [a, (deltaRef - this.x[4]) / P.dt]);
  }

  /** Filter problem at the current state for the selected filter (with the
   * BH-CBF condition for "off", for the displays). */
  problem() {
    return makeProblem(this.x, {
      route: this.route, progress: this.progress, obstacles: this.obstacles, lambda: this.lambda,
      condition: this.mode === "one-step" ? "one-step" : "bh",
    });
  }

  /** Filter problem, slack grid (control plane), and filter output at the
   * current state for a nominal control (default: that of keys). */
  analyze({ keys = {}, uNominal = null, grid = this.options.grid } = {}) {
    const problem = this.problem();
    const slack = slackGrid(problem, grid);
    // A control chosen in the control plane stays fixed while the bounds of
    // the state change, so saturate it at every step
    const uNom = uNominal ? saturate(this.x, uNominal) : this.nominal(keys);
    const out = this.mode === "off" ? { u: uNom, status: "off", result: evaluate(problem, uNom) } : filter(problem, uNom, slack);
    // Smallest backup value: nonnegative iff the state is recoverable
    const B = Math.min(...problem.constraints.map((con) => con.B));
    this.current = { problem, grid: slack, uNom, out, B };
    return this.current;
  }

  /** Apply the filter output of the current analysis (one step). */
  advance() {
    if (!this.current || (this.outcome && this.outcome.kind !== "stopped")) return;
    const { out, uNom } = this.current;
    const previous = this.x;
    this.x = eulerStep(this.x, out.u);
    this.x[2] = wrap(this.x[2]);
    this.k++;
    this.updateProgress();
    this.history.push({ x: this.x.slice(), u: out.u, uNom, status: out.status });
    this.outcome = this.check(previous[3]);
    this.current = null;
  }

  /** Analyze for the commands keys and apply the result. */
  step(keys) {
    this.analyze({ keys });
    const current = this.current;
    this.advance();
    return current;
  }

  /** Route progress: the closest station near the previous one. */
  updateProgress() {
    const s = closestStation(this.route, this.x[0], this.x[1]);
    if (Math.abs(s - this.progress) < 15) this.progress = s;
    else this.progress += this.x[3] * PARAMS.dt;
  }

  /** Collision with a boundary of the route or the stopped vehicle, a
   * standstill behind it, or the end of the route at the current pose. */
  check(speed) {
    const pose = this.x.slice(0, 3);
    for (const obstacle of this.obstacles) {
      if (signedDistance(pose, obstacle).distance < 0) return { kind: "collision", with: "stopped vehicle", pose, speed };
    }
    const [left, right] = boundariesAt(this.route, this.progress, 0);
    for (const [label, line] of [["left boundary", left], ["right boundary", right]]) {
      for (let i = 0; i + 1 < line.length; i++) {
        if (rectangleHitsSegment(pose, line[i], line[i + 1])) return { kind: "collision", with: label, pose, speed };
      }
    }
    if (this.progress > this.route.total - 6) return { kind: "end" };
    if (this.x[3] < 0.05 && this.obstacles.length) {
      const gap = signedDistance(pose, this.obstacles[0]).distance;
      if (gap < 8) return { kind: "stopped", gap };
    }
    return null;
  }
}
