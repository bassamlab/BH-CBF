// Compares bhcbf.js with reference values of the Python implementation
// (SigmaRL: sigmarl/evaluation_tits26/video_demo/widget_data.py reference).
//
// Usage: node assets/widget/check_model.mjs <widget_reference.json>

import { readFileSync } from "node:fs";
import {
  PARAMS, boundariesAt, laneMargin, makeFacets, makeRoute, rollout, signedDistance,
} from "./bhcbf.js";

const ref = JSON.parse(readFileSync(process.argv[2], "utf8"));
const map = JSON.parse(readFileSync(new URL("./roundabout.json", import.meta.url), "utf8"));
let failed = false;

function report(name, error, limit) {
  const ok = error <= limit;
  failed ||= !ok;
  console.log(`${ok ? "ok  " : "FAIL"} ${name}: largest deviation ${error.toExponential(2)} (limit ${limit.toExponential(0)})`);
}

// Rollouts (candidate predictions under the controls of the path sweeps and
// the backup control) and lane values along the Python poses
let poseError = 0, bufferError = 0, laneError = 0, n = 0;
for (const c of ref.cases) {
  const route = makeRoute(c.route, c.route, c.route, false);
  c.U.forEach((u, g) => {
    const { poses } = rollout(c.X, u, route, c.n_steps);
    poses.forEach((pose, t) => pose.forEach((v, d) => {
      poseError = Math.max(poseError, Math.abs(v - c.poses[g][t][d]));
    }));
    n++;
  });
  c.lanes.forEach((lane, l) => {
    const facets = makeFacets(lane.points, lane.side, PARAMS.epsLane, true);
    lane.buffer.forEach((b, j) => { bufferError = Math.max(bufferError, Math.abs(b - facets.buffer[j])); });
    // The path sweeps store their values in float32: relative deviation
    c.lane_values[l].forEach((values, g) => values.forEach((v, t) => {
      laneError = Math.max(laneError, Math.abs(laneMargin(facets, c.poses[g][t]) - v) / Math.max(1, Math.abs(v)));
    }));
  });
}
report(`rollouts (${n})`, poseError, 1e-9);
report("facet buffers", bufferError, 1e-12);
report("lane values (relative, float32 storage)", laneError, 1.2e-7);

// Lane windows of the widget's route
const route = makeRoute(map.route.center, map.route.left, map.route.right, map.route.is_loop);
let windowError = 0;
for (const w of ref.windows) {
  const [left, right] = boundariesAt(route, w.progress, w.ahead);
  for (const [mine, theirs] of [[left, w.left], [right, w.right]]) {
    if (mine.length !== theirs.length) { windowError = Infinity; continue; }
    mine.forEach((p, i) => { windowError = Math.max(windowError, Math.abs(p[0] - theirs[i][0]), Math.abs(p[1] - theirs[i][1])); });
  }
}
report(`lane windows (${ref.windows.length})`, windowError, 1e-12);

// Signed distances and separating normals
const sd = ref.signed_distance;
let sdError = 0, witnessError = 0;
sd.a.forEach((a, k) => {
  const { distance, witness } = signedDistance(a, sd.b[k]);
  sdError = Math.max(sdError, Math.abs(distance - sd.distance[k]));
  witnessError = Math.max(witnessError, Math.abs(witness[0] - sd.witness[k][0]), Math.abs(witness[1] - sd.witness[k][1]));
});
report(`signed distances (${sd.a.length})`, sdError, 1e-12);
report("separating normals", witnessError, 1e-9);
process.exit(failed ? 1 : 0);
