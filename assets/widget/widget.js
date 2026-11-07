// Interactive BH-CBF widget: drive a vehicle through the roundabout of the
// paper with no filter, a one-step CBF filter, or the BH-CBF filter, choose
// the control yourself in the control plane, and pause at any step to look
// inside the filter problem.

import { PARAMS, corners, evaluate, filter, saturate } from "./bhcbf.js";
import { Simulation } from "./sim.js";

const COLOR = {
  blue: "#00549F", blue50: "#8EBAE5", blue25: "#C7DDF2", blue10: "#E9F1FA",
  purple: "#7A6FAC", red: "#CC071E", redTint: "#F6DADD", green: "#57AB27",
  gray: "#8C8C8C", dark: "#5F6368", text: "#1A1A1A", turq: "#0098A1",
  road: "#ECECEC", roadEdge: "#A0A4A8", boundary: "#3C4043", box: "#F4F4F4",
};
const FILTER_NAMES = { off: "No Filter", "one-step": "One-Step CBF", bh: "BH-CBF" };
const FILTER_BUTTONS = { ...FILTER_NAMES, bh: "BH-CBF (Ours)" };
// Station [m] of the stopped vehicle on the route (in the exit lane)
const OBSTACLE_STATION = 130;
const DRIVE_GRID = 21; // Grid points per control while driving
const PAUSE_GRID = 41; // ... and while paused (finer control plane)
const VIEW = 46; // Default height of the map view [m]
const VIEW_RANGE = [16, 90]; // ... and its range (zoom)
const STEP_MS = 1000 * PARAMS.dt; // Duration of a step in real time [ms]
const SPEED_RANGE = [0.05, 2]; // Playback speed (multiple of real time)

export async function mount(root, mapUrl) {
  const map = await (await fetch(mapUrl)).json();
  return new Widget(root, map);
}

function el(tag, attrs = {}, children = []) {
  const node = document.createElement(tag);
  for (const [key, value] of Object.entries(attrs)) {
    if (key === "class") node.className = value;
    else if (key === "text") node.textContent = value;
    else if (key === "html") node.innerHTML = value;
    else node.setAttribute(key, value);
  }
  for (const child of children) node.append(child);
  return node;
}

/** Segmented buttons; group.set(key) selects one from code. */
function segmented(options, value, onChange, label) {
  const group = el("div", { class: "bw-seg", role: "group", "aria-label": label });
  const buttons = {};
  group.set = (key) => Object.entries(buttons).forEach(([k, b]) => b.setAttribute("aria-pressed", String(k === key)));
  for (const [key, text] of Object.entries(options)) {
    const button = el("button", { type: "button", text });
    button.addEventListener("click", () => { group.set(key); onChange(key); });
    buttons[key] = button;
    group.append(button);
  }
  group.set(value);
  return group;
}

/** A canvas that keeps its backing store at the device resolution. */
class Surface {
  constructor(canvas, onResize) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    new ResizeObserver(() => { this.resize(); onResize(); }).observe(canvas);
    this.resize();
  }

  resize() {
    const dpr = window.devicePixelRatio || 1;
    this.w = this.canvas.clientWidth;
    this.h = this.canvas.clientHeight;
    this.canvas.width = Math.round(this.w * dpr);
    this.canvas.height = Math.round(this.h * dpr);
    this.dpr = dpr;
  }

  begin() {
    const { ctx } = this;
    ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
    ctx.clearRect(0, 0, this.w, this.h);
    return ctx;
  }
}

const lerp = (a, b, f) => a + (b - a) * f;
const lerpAngle = (a, b, f) => a + Math.atan2(Math.sin(b - a), Math.cos(b - a)) * f;
const fmt = (v, digits = 2) => (v >= 0 ? "" : "−") + Math.abs(v).toFixed(digits);

class Widget {
  constructor(root, map) {
    this.root = root;
    this.map = map;
    this.sim = new Simulation(map, { start: 15, speed: 8, obstacle: OBSTACLE_STATION, grid: DRIVE_GRID });
    this.speed = 1; // Playback speed
    this.view = VIEW;
    this.keys = {};
    this.running = false;
    this.started = false;
    this.elapsed = 0;
    this.dragged = null; // Control chosen in the control plane (null: from the keys)
    this.build();
    this.camera = { angle: this.sim.x[2] };
    this.analyze();
    this.showCard("start");
    this.bind();
    this.last = performance.now();
    requestAnimationFrame((t) => this.loop(t));
  }

  // -------------------------------------------------------------------------
  // DOM

  build() {
    const root = this.root;
    root.classList.add("bw");
    root.tabIndex = 0;
    root.setAttribute("aria-label", "Interactive BH-CBF demo. Arrow keys drive, Space pauses and resumes, R restarts.");
    this.filterButtons = segmented(FILTER_BUTTONS, this.sim.mode, (mode) => this.setMode(mode), "Filter");
    const bar = el("div", { class: "bw-bar" }, [
      el("div", { class: "bw-group" }, [el("span", { text: "Filter" }), this.filterButtons]),
    ]);
    // Playback speed, on a log scale between SPEED_RANGE
    const [s0, s1] = SPEED_RANGE;
    const speed = el("input", { type: "range", min: "0", max: "1", step: "0.001", "aria-label": "Playback speed",
      value: String(Math.log(this.speed / s0) / Math.log(s1 / s0)) });
    const speedValue = el("span", { class: "bw-value" });
    const setSpeed = () => {
      this.speed = s0 * (s1 / s0) ** Number(speed.value);
      speedValue.textContent = `${this.speed.toFixed(2)}×`;
    };
    speed.addEventListener("input", setSpeed);
    setSpeed();
    bar.append(el("div", { class: "bw-group" }, [el("span", { text: "Playback speed" }), speed, speedValue]));

    this.mapCanvas = el("canvas", { role: "img", "aria-label": "Map of the roundabout with your vehicle and its predictions" });
    this.overlay = el("div", { class: "bw-overlay" });
    // Zoom buttons (also Ctrl + wheel or pinch on the map; double-click resets)
    const zoom = el("div", { class: "bw-zoom" });
    for (const [text, label, factor] of [["+", "Zoom in", 1 / 1.4], ["−", "Zoom out", 1.4], ["↺", "Reset zoom", null]]) {
      const button = el("button", { type: "button", text, "aria-label": label, title: label });
      button.addEventListener("click", () => this.zoom(factor));
      zoom.append(button);
    }
    const mapBox = el("div", { class: "bw-map" }, [this.mapCanvas, zoom, this.overlay]);
    const mapLegend = el("div", { class: "bw-legend bw-map-legend", html:
      `<span><i style="background:${COLOR.blue}"></i>your vehicle</span>` +
      `<span>${lineChip(COLOR.turq, "5 3")}route centerline (reference path)</span>` +
      `<span>${lineChip(COLOR.boundary)}route boundaries</span>` +
      `<span><i style="background:#D9D9D9;border:1px solid ${COLOR.gray}"></i>backup prediction (full braking)</span>` +
      `<span><i style="background:#fff;border:1.5px dashed ${COLOR.purple}"></i>prediction of your control, where the filter changes it</span>` +
      `<span><i style="background:#fff;border:1.5px solid ${COLOR.blue}"></i>prediction of the filter output (while paused)</span>` });

    this.planeCanvas = el("canvas", { class: "bw-plane", role: "img", "aria-label": "Control plane. Press and hold to choose the control." });
    this.plotCanvas = el("canvas", { class: "bw-plot", role: "img", "aria-label": "Margin along the predictions" });
    this.planeTitle = el("p", { class: "bw-title" });
    this.plotTitle = el("p", { class: "bw-title" });
    const side = el("div", { class: "bw-side" }, [
      el("div", { class: "bw-panel" }, [this.planeTitle, el("div", { class: "bw-fill bw-fill-plane" }, [this.planeCanvas]), el("div", { class: "bw-legend", html:
        `<span><i style="background:${COLOR.blue25}"></i>satisfies the condition</span>` +
        `<span><i style="background:${COLOR.redTint}"></i>violates it</span>` +
        `<span><i style="background:${COLOR.purple};border-radius:50%;width:9px"></i>your control</span>` +
        `<span><i style="background:${COLOR.blue};border-radius:50%;width:9px"></i>filter output</span>` +
        `<span><i style="background:${COLOR.gray};width:9px"></i>backup</span>` })]),
      el("div", { class: "bw-panel" }, [this.plotTitle, el("div", { class: "bw-fill" }, [this.plotCanvas]), this.plotLegend = el("div", { class: "bw-legend" })]),
    ]);
    const main = el("div", { class: "bw-main" }, [el("div", { class: "bw-left" }, [mapBox, mapLegend]), side]);

    this.playButton = el("button", { class: "bw-btn", type: "button" });
    this.restartButton = el("button", { class: "bw-btn ghost", type: "button", text: "Restart" });
    const keys = el("div", { class: "bw-keys", html:
      "<kbd>↑</kbd> speed up · <kbd>↓</kbd> brake · <kbd>←</kbd> <kbd>→</kbd> steer · <kbd>Space</kbd> pause · <kbd>R</kbd> restart" });
    const controls = el("div", { class: "bw-controls" }, [this.playButton, this.restartButton]);
    this.pad = el("div", { class: "bw-pad" });
    for (const [key, text, label] of [["left", "◀", "Steer left"], ["up", "▲", "Speed up"], ["down", "▼", "Brake"], ["right", "▶", "Steer right"]]) {
      const button = el("button", { type: "button", text, "aria-label": label });
      const set = (on) => { this.keys[key] = on; button.classList.toggle("on", on); this.onKeys(); };
      button.addEventListener("pointerdown", (event) => {
        event.preventDefault();
        button.setPointerCapture(event.pointerId);
        set(true);
        if (!this.started) this.play();
      });
      for (const type of ["pointerup", "pointercancel", "lostpointercapture"]) button.addEventListener(type, () => set(false));
      this.pad.append(button);
    }
    this.status = el("div", { class: "bw-status", "aria-live": "polite" });
    root.append(bar, keys, main, controls, this.pad, this.status);

    const redraw = () => this.draw();
    this.mapSurface = new Surface(this.mapCanvas, redraw);
    this.planeSurface = new Surface(this.planeCanvas, redraw);
    this.plotSurface = new Surface(this.plotCanvas, redraw);
  }

  bind() {
    const keyOf = { ArrowUp: "up", KeyW: "up", ArrowDown: "down", KeyS: "down", ArrowLeft: "left", KeyA: "left", ArrowRight: "right", KeyD: "right" };
    this.root.addEventListener("keydown", (event) => {
      const key = keyOf[event.code];
      if (key) {
        event.preventDefault();
        if (!this.keys[key]) { this.keys[key] = true; this.onKeys(); }
        if (!this.started) this.play();
      } else if (event.code === "Space") {
        event.preventDefault();
        if (event.target.tagName !== "BUTTON") this.toggle();
      } else if (event.code === "KeyR") this.restart();
    });
    this.root.addEventListener("keyup", (event) => {
      const key = keyOf[event.code];
      if (key) { this.keys[key] = false; this.onKeys(); }
    });
    // Pause when the keyboard focus leaves the widget (checked after the
    // focus has moved: removing a focused button also fires focusout)
    const leave = () => { this.keys = {}; if (this.running) this.pause(); };
    this.root.addEventListener("focusout", () => setTimeout(() => {
      if (!this.root.contains(document.activeElement)) leave();
    }, 0));
    window.addEventListener("blur", leave);
    this.playButton.addEventListener("click", () => this.toggle());
    this.restartButton.addEventListener("click", () => this.restart());
    this.mapCanvas.addEventListener("pointerdown", () => this.root.focus({ preventScroll: true }));
    this.mapCanvas.addEventListener("dblclick", () => this.zoom(null));
    this.mapCanvas.addEventListener("wheel", (event) => {
      if (!event.ctrlKey) return; // Plain wheel: scroll the page
      event.preventDefault();
      this.zoom(Math.exp(event.deltaY * 0.01));
    }, { passive: false });

    // Press and hold in the control plane to choose the control yourself:
    // while driving, the vehicle keeps driving with it; while paused, the
    // filter output for it is shown without a step
    const plane = this.planeCanvas;
    const toControl = (event) => {
      const r = plane.getBoundingClientRect();
      return [this.planeScale.a.inv(event.clientX - r.left), this.planeScale.w.inv(event.clientY - r.top)];
    };
    plane.addEventListener("pointerdown", (event) => {
      if (this.ended()) return;
      event.preventDefault();
      this.root.focus({ preventScroll: true });
      plane.setPointerCapture(event.pointerId);
      this.dragging = true;
      if (!this.started) this.play();
      this.setNominal(toControl(event));
    });
    plane.addEventListener("pointermove", (event) => { if (this.dragging && !this.ended()) this.setNominal(toControl(event)); });
    for (const type of ["pointerup", "pointercancel"]) {
      plane.addEventListener(type, () => {
        this.dragging = false;
        // Back to the keys while driving; keep it on display while paused
        if (this.running) { this.dragged = null; this.refilter(); }
      });
    }
  }

  // -------------------------------------------------------------------------
  // Simulation control

  ended() { return Boolean(this.sim.outcome); }

  analyze() {
    const grid = this.running ? DRIVE_GRID : PAUSE_GRID;
    const cur = this.sim.analyze({ keys: this.keys, uNominal: this.dragged, grid });
    cur.policy = evaluate(cur.problem, cur.uNom);
    this.draw();
  }

  /** New nominal control at the same state (control plane). */
  setNominal(u) {
    this.dragged = u;
    this.refilter();
  }

  /** The filter output for the current nominal control at the same state. */
  refilter() {
    const cur = this.sim.current;
    if (!cur) return;
    const u = this.dragged ? saturate(this.sim.x, this.dragged) : this.sim.nominal(this.keys);
    cur.uNom = u;
    cur.out = this.sim.mode === "off" ? { u, status: "off", result: evaluate(cur.problem, u) } : filter(cur.problem, u, cur.grid);
    cur.policy = evaluate(cur.problem, u);
    this.draw();
  }

  onKeys() {
    if (this.sim.current && !this.dragged) this.refilter();
  }

  toggle() {
    if (this.ended()) this.restart();
    else if (this.running) this.pause();
    else this.play();
  }

  play() {
    if (this.ended()) this.sim.reset();
    // Focus first: hiding the card removes its focused button
    this.root.focus({ preventScroll: true });
    this.started = true;
    this.running = true;
    if (!this.dragging) this.dragged = null;
    this.elapsed = 0;
    this.hideCard();
    this.analyze();
  }

  pause() {
    this.running = false;
    this.elapsed = 0;
    this.analyze();
  }

  /** Back to the start; drives off at once unless before the first start. */
  restart() {
    this.sim.reset();
    this.camera.angle = this.sim.x[2];
    this.dragged = null;
    this.running = false;
    if (this.started) this.play();
    else this.analyze();
  }

  setMode(mode) {
    this.sim.mode = mode;
    this.filterButtons.set(mode);
    if (this.ended()) this.restart();
    else this.analyze();
  }

  /** Zoom the map by a factor of its view height (null: back to the default). */
  zoom(factor) {
    this.view = factor == null ? VIEW : Math.min(Math.max(this.view * factor, VIEW_RANGE[0]), VIEW_RANGE[1]);
    this.drawMap();
  }

  /** After a step: the analysis of the new state, or the outcome. */
  afterStep() {
    const outcome = this.sim.outcome;
    if (outcome) {
      this.running = false;
      this.dragged = null;
      this.dragging = false;
    }
    if (outcome && outcome.kind === "collision") this.draw();
    else if (this.running) {
      const cur = this.sim.analyze({ keys: this.keys, uNominal: this.dragged, grid: DRIVE_GRID });
      cur.policy = evaluate(cur.problem, cur.uNom);
      this.drawPlane();
      this.drawPlot();
      this.updateStatus();
    } else this.analyze();
    if (outcome) this.showCard(outcome.kind);
  }

  /** Duration of a step at the playback speed [ms]. */
  stepMs() { return STEP_MS / this.speed; }

  loop(now) {
    const dt = Math.min(now - this.last, 250);
    this.last = now;
    if (this.running) {
      this.elapsed += dt;
      while (this.running && this.elapsed >= this.stepMs()) {
        this.elapsed -= this.stepMs();
        this.sim.advance();
        this.afterStep();
      }
    }
    this.drawMap();
    requestAnimationFrame((t) => this.loop(t));
  }

  // -------------------------------------------------------------------------
  // Cards and status

  showCard(kind) {
    const outcome = this.sim.outcome, mode = this.sim.mode;
    let cls = "", title = "", body = "";
    if (kind === "start") {
      title = "Can you make the vehicle collide?";
      body = "A vehicle is stopped in your exit lane ahead. Start, then hold <kbd>↑</kbd> to speed up and <kbd>←</kbd> or <kbd>→</kbd> to steer toward a boundary, or press and hold in the control plane to choose the control yourself.";
    } else if (kind === "collision") {
      cls = "bad";
      title = `Collision with the ${outcome.with} at ${Math.round(outcome.speed * 3.6)} km/h`;
      const infeasible = this.sim.history.filter((h) => h.status === "infeasible").length;
      body = mode === "off"
        ? "Without a filter, your control was applied as it was."
        : `The one-step condition looks only two steps ahead. It let the vehicle come so close that no admissible control could satisfy it (infeasible at ${infeasible} step${infeasible === 1 ? "" : "s"}), and ${outcome.with === "stopped vehicle" ? "braking" : "the correction"} came too late.`;
    } else if (kind === "stopped") {
      cls = "good";
      title = "Stopped without collision";
      body = `Gap to the stopped vehicle: ${outcome.gap.toFixed(2)} m (required clearance: ${PARAMS.epsVeh.toFixed(2)} m).` +
        (mode === "bh" ? " BH-CBF braked as soon as the backup prediction required it." : "");
    } else if (kind === "end") {
      cls = "good";
      title = "End of the route, no collision";
      body = "Try again and steer toward a boundary while holding <kbd>↑</kbd>.";
    }
    this.overlay.replaceChildren();
    const card = el("div", { class: `bw-card ${cls}`, html: `<strong>${title}</strong>${body}` });
    const buttons = el("div", { class: "bw-card-buttons" });
    if (kind === "start") {
      const start = el("button", { class: "bw-btn", type: "button", text: "Start driving" });
      start.addEventListener("click", () => this.play());
      buttons.append(start);
    } else {
      // Retry with the other filter: BH-CBF after a collision, the one-step
      // CBF filter after a run with BH-CBF
      const other = mode === "bh" ? "one-step" : "bh";
      const retry = el("button", { class: "bw-btn", type: "button", text: `Retry with ${FILTER_BUTTONS[other]}` });
      retry.addEventListener("click", () => this.setMode(other));
      const again = el("button", { class: "bw-btn ghost", type: "button", text: "Restart" });
      again.addEventListener("click", () => this.restart());
      buttons.append(retry, again);
    }
    card.append(buttons);
    this.overlay.append(card);
  }

  hideCard() { this.overlay.replaceChildren(); }

  updateStatus() {
    const cur = this.sim.current, mode = this.sim.mode;
    this.playButton.textContent = this.ended() ? "Restart" : this.running ? "Pause" : this.started ? "Resume" : "Start";
    this.restartButton.hidden = this.ended() || !this.started;
    if (this.ended() && this.sim.outcome.kind === "collision") {
      this.status.innerHTML = `<span class="tag" style="background:${COLOR.red}">Collision</span>The vehicle collided after ${(this.sim.k * PARAMS.dt).toFixed(1)} s.`;
      return;
    }
    if (!cur) return;
    const tags = {
      off: ["No filter", COLOR.gray], accepted: ["Accepted", COLOR.green], modified: ["Changed", COLOR.blue],
      fallback: ["Backup", COLOR.blue], infeasible: ["Infeasible", COLOR.red], unrecoverable: ["Not recoverable", COLOR.red],
    };
    const { status } = cur.out;
    const [tag, color] = tags[status];
    const label = cur.problem.constraints[cur.policy.worst].label;
    const text = {
      off: "Your control is applied as it is." + (cur.policy.slack < -PARAMS.tol ? ` BH-CBF would change it (too close to the ${label}).` : ""),
      accepted: "Your control satisfies the condition and is applied unchanged.",
      modified: `Your control would bring the vehicle too close to the ${label}. The filter applies the closest control that satisfies the condition.`,
      fallback: "No control near yours satisfies the condition, so the vehicle applies its backup control (full braking along the route).",
      infeasible: "No admissible control satisfies the one-step condition. The filter applies the one that violates it least, without a safety guarantee.",
      unrecoverable: "Not even the backup control satisfies the condition: the state is not recoverable.",
    }[status];
    const recoverable = cur.B >= -2 * PARAMS.tol
      ? `<span style="color:${COLOR.green}">recoverable</span>: full braking along the route avoids all collisions`
      : `<span style="color:${COLOR.red}">not recoverable</span>: even full braking along the route collides`;
    const paused = this.started && !this.running && !this.ended()
      ? "Paused. Press and hold in the control plane to try other controls. " : "";
    this.status.innerHTML = `<span class="tag" style="background:${color}">${tag}</span>${text}` +
      `<br><span style="color:${COLOR.dark}">${paused}State: ${recoverable}.</span>`;
  }

  // -------------------------------------------------------------------------
  // Drawing

  /** The analysis on display: the current one, or after a collision that
   * of the last step before it. */
  shown() {
    if (this.sim.current) this.lastShown = this.sim.current;
    return this.sim.current ?? (this.sim.outcome && this.sim.outcome.kind === "collision" ? this.lastShown : null);
  }

  draw() {
    this.drawMap();
    this.drawPlane();
    this.drawPlot();
    this.updateStatus();
  }

  /** The vehicle's drawn pose: on its way to the next pose while driving. */
  drawnPose() {
    const x = this.sim.x, cur = this.sim.current;
    if (!this.running || !cur) return x.slice(0, 3);
    const next = cur.out.result.prediction.poses[0];
    const f = Math.min(this.elapsed / this.stepMs(), 1);
    return [lerp(x[0], next[0], f), lerp(x[1], next[1], f), lerpAngle(x[2], next[2], f)];
  }

  drawMap() {
    const s = this.mapSurface, ctx = s.begin();
    if (!s.w) return;
    const pose = this.drawnPose();
    this.camera.angle = lerpAngle(this.camera.angle, pose[2], this.running ? 0.08 : 1);
    const scale = s.h / this.view;
    const angle = this.camera.angle;
    // Vehicle lower in the view when zoomed in, so that more of the predictions shows
    const anchor = 0.66 + 0.2 * Math.min(Math.max((VIEW - this.view) / (VIEW - VIEW_RANGE[0]), 0), 1);
    const toScreen = ([px, py]) => {
      const dx = px - pose[0], dy = py - pose[1];
      const c = Math.cos(Math.PI / 2 - angle), sn = Math.sin(Math.PI / 2 - angle);
      const rx = c * dx - sn * dy, ry = sn * dx + c * dy;
      return [s.w / 2 + rx * scale, s.h * anchor - ry * scale];
    };
    ctx.save();
    ctx.translate(s.w / 2, s.h * anchor);
    ctx.scale(scale, -scale);
    ctx.rotate(Math.PI / 2 - angle);
    ctx.translate(-pose[0], -pose[1]);
    const px = 1 / scale;
    // Road
    for (const part of this.map.road) {
      ctx.beginPath();
      for (const ring of [part.outer, ...part.holes]) {
        ring.forEach(([x, y], i) => (i ? ctx.lineTo(x, y) : ctx.moveTo(x, y)));
        ctx.closePath();
      }
      ctx.fillStyle = COLOR.road;
      ctx.fill("evenodd");
      ctx.strokeStyle = COLOR.roadEdge;
      ctx.lineWidth = px;
      ctx.stroke();
    }
    // Route: its boundaries (the lane constraints) and centerline
    const route = this.map.route;
    ctx.lineWidth = 2 * px;
    ctx.strokeStyle = COLOR.boundary;
    for (const line of [route.left, route.right]) polyline(ctx, line);
    ctx.setLineDash([6 * px, 6 * px]);
    ctx.lineWidth = 1.3 * px;
    ctx.strokeStyle = COLOR.turq;
    polyline(ctx, route.center);
    ctx.setLineDash([]);

    const cur = this.sim.current, dead = this.sim.outcome && this.sim.outcome.kind === "collision";
    if (cur && !dead) {
      // Backup prediction (gray); the prediction of your control where the
      // filter changes it (purple, dashed); the filter output (blue, paused)
      footprints(ctx, cur.problem.backup.poses, COLOR.gray, px, { fill: "rgba(140,140,140,0.10)", alpha: 0.55 });
      const changed = cur.out.status !== "accepted" && cur.out.status !== "off";
      if (changed || !this.running) {
        footprints(ctx, cur.policy.prediction.poses, COLOR.purple, px, { dash: [4 * px, 3 * px], alpha: 0.85 });
      }
      if (!this.running && cur.out.status !== "off") {
        footprints(ctx, cur.out.result.prediction.poses, COLOR.blue, px, { alpha: 0.7 });
      }
    }
    // Stopped vehicle and the vehicle
    for (const obstacle of this.sim.obstacles) vehicle(ctx, obstacle, COLOR.dark, px);
    vehicle(ctx, pose, dead ? COLOR.red : COLOR.blue, px);
    ctx.restore();

    // Labels in screen coordinates
    ctx.font = "600 13px -apple-system, BlinkMacSystemFont, 'Segoe UI', Helvetica, Arial, sans-serif";
    ctx.textAlign = "center";
    for (const obstacle of this.sim.obstacles) {
      const [x, y] = toScreen(obstacle);
      if (x > -50 && x < s.w + 50 && y > -20 && y < s.h + 20) halo(ctx, "stopped vehicle", x, y - 3.2 * scale, COLOR.dark);
    }
    if (dead) {
      const [x, y] = toScreen(this.sim.outcome.pose);
      star(ctx, x, y, 12, COLOR.red);
    }
    // Speed and filter
    ctx.textAlign = "left";
    ctx.font = "600 14px -apple-system, BlinkMacSystemFont, 'Segoe UI', Helvetica, Arial, sans-serif";
    const speedText = `${(this.sim.x[3] * 3.6).toFixed(0)} km/h`;
    box(ctx, `${FILTER_NAMES[this.sim.mode]} · ${speedText} · step ${this.sim.k}`, 10, 10);
  }

  /** The control plane: acceleration (horizontal) and steering rate
   * (vertical), the controls that satisfy the condition (exact on the grid),
   * your control, the filter output, and the backup control. */
  drawPlane() {
    const s = this.planeSurface, ctx = s.begin(), cur = this.shown();
    const P = PARAMS;
    const m = { l: 46, r: 10, t: 10, b: 38 };
    const ax = linear(P.aMin, P.aMax, m.l, s.w - m.r), wy = linear(P.wMin, P.wMax, s.h - m.b, m.t);
    this.planeScale = { a: ax, w: wy };
    const mode = this.sim.mode;
    this.planeTitle.innerHTML = `<span>Control plane: ${mode === "one-step" ? "one-step condition" : mode === "bh" ? "BH-CBF condition" : "BH-CBF condition (shown, not enforced)"}${this.sim.current ? "" : ", before the collision"}</span>`
      + `<span class="bw-hint">press and hold to choose your control</span>`;
    // Outside the admissible box
    ctx.fillStyle = COLOR.box;
    ctx.fillRect(m.l, m.t, s.w - m.l - m.r, s.h - m.t - m.b);
    if (cur) {
      const g = cur.grid;
      const [lo, hi] = cur.problem.bounds;
      const x0 = ax(lo[0]), x1 = ax(hi[0]), y0 = wy(hi[1]), y1 = wy(lo[1]);
      ctx.fillStyle = COLOR.redTint;
      ctx.fillRect(x0, y0, x1 - x0, y1 - y0);
      // Feasible region (slack >= 0), piecewise linear on the grid
      ctx.fillStyle = COLOR.blue25;
      ctx.strokeStyle = COLOR.blue;
      ctx.lineWidth = 1.2;
      const value = (i, j) => g.slack[i * g.G + j] + P.tol;
      const point = (i, j) => [ax(g.a[i]), wy(g.w[j])];
      const edges = [];
      ctx.beginPath();
      for (let i = 0; i + 1 < g.G; i++) {
        for (let j = 0; j + 1 < g.G; j++) {
          for (const tri of [[[i, j], [i + 1, j], [i + 1, j + 1]], [[i, j], [i + 1, j + 1], [i, j + 1]]]) {
            const poly = clipTriangle(tri.map(([a, b]) => [...point(a, b), value(a, b)]), edges);
            if (poly.length) {
              ctx.moveTo(poly[0][0], poly[0][1]);
              for (const q of poly.slice(1)) ctx.lineTo(q[0], q[1]);
              ctx.closePath();
            }
          }
        }
      }
      ctx.fill();
      ctx.beginPath();
      for (const [p, q] of edges) { ctx.moveTo(p[0], p[1]); ctx.lineTo(q[0], q[1]); }
      ctx.stroke();
      // Only full braking (the grid column at the lowest acceleration)
      // satisfies the condition: the region is a sliver at the left edge, so
      // draw it as a band and say so
      let column = false, beyond = false, jLo = g.G, jHi = -1;
      for (let i = 0; i < g.G; i++) {
        for (let j = 0; j < g.G; j++) {
          if (value(i, j) < 0) continue;
          if (i > 0) beyond = true;
          else { column = true; jLo = Math.min(jLo, j); jHi = Math.max(jHi, j); }
        }
      }
      const fallback = cur.out.status === "fallback";
      if ((column && !beyond) || (!column && !beyond && fallback)) {
        ctx.fillStyle = COLOR.blue;
        if (column) {
          const yA = wy(g.w[jHi]), yB = wy(g.w[jLo]);
          ctx.fillRect(x0, Math.min(yA, yB) - 1.5, 5, Math.abs(yB - yA) + 3);
        }
        ctx.font = "600 13px -apple-system, BlinkMacSystemFont, 'Segoe UI', Helvetica, Arial, sans-serif";
        halo(ctx, column ? "← only full braking satisfies it" : "← only the backup control satisfies it", x0 + 12, m.t + 18, COLOR.blue, "left");
      }
      ctx.strokeStyle = COLOR.dark;
      ctx.lineWidth = 1;
      ctx.strokeRect(x0, y0, x1 - x0, y1 - y0);
      // Markers
      const backup = cur.problem.backupU;
      ctx.fillStyle = COLOR.gray;
      ctx.fillRect(ax(backup[0]) - 5, wy(backup[1]) - 5, 10, 10);
      const u = cur.uNom, f = cur.out.u;
      if (Math.hypot(ax(u[0]) - ax(f[0]), wy(u[1]) - wy(f[1])) > 3) arrow(ctx, [ax(u[0]), wy(u[1])], [ax(f[0]), wy(f[1])], COLOR.text);
      dot(ctx, ax(u[0]), wy(u[1]), 6, COLOR.purple);
      if (cur.out.status !== "off") dot(ctx, ax(f[0]), wy(f[1]), 5, COLOR.blue);
      if (cur.out.status === "infeasible" || cur.out.status === "unrecoverable") {
        ctx.font = "600 13px -apple-system, BlinkMacSystemFont, 'Segoe UI', Helvetica, Arial, sans-serif";
        ctx.textAlign = "center";
        halo(ctx, cur.out.status === "infeasible" ? "No admissible control satisfies it" : "Not even the backup satisfies it", (x0 + x1) / 2, m.t + 18, COLOR.red);
      }
    }
    // Axes
    ctx.fillStyle = COLOR.dark;
    ctx.strokeStyle = COLOR.dark;
    ctx.font = "12px -apple-system, BlinkMacSystemFont, 'Segoe UI', Helvetica, Arial, sans-serif";
    ctx.textAlign = "center";
    ctx.textBaseline = "top";
    for (const a of [-6, -3, 0, 3]) ctx.fillText(String(a).replace("-", "−"), ax(a), s.h - m.b + 4);
    ctx.fillText("acceleration a [m/s²]", (m.l + s.w - m.r) / 2, s.h - m.b + 19);
    ctx.textAlign = "right";
    ctx.textBaseline = "middle";
    for (const w of [-1, 0, 1]) ctx.fillText(String(w).replace("-", "−"), m.l - 5, wy(w));
    ctx.save();
    ctx.translate(13, (m.t + s.h - m.b) / 2);
    ctx.rotate(-Math.PI / 2);
    ctx.textAlign = "center";
    ctx.fillText("steering rate ω [rad/s]", 0, 0);
    ctx.restore();
    ctx.textBaseline = "alphabetic";
  }

  /** Margin of one constraint along the backup prediction, the prediction of
   * your control, and that of the filter output, with the condition's bound. */
  drawPlot() {
    const s = this.plotSurface, ctx = s.begin(), cur = this.shown();
    const m = { l: 46, r: 10, t: 12, b: 38 };
    if (!cur) return;
    const constraints = cur.problem.constraints;
    const e = autoConstraint(cur);
    this.plotTitle.textContent = `Margin to the ${constraints[e].label} along the predictions`;
    const con = constraints[e];
    const series = [
      [cur.problem.backup.values[e], COLOR.gray, []],
      [cur.policy.prediction.values[e], COLOR.purple, [5, 3]],
    ];
    if (cur.out.status !== "off") series.push([cur.out.result.prediction.values[e], COLOR.blue, []]);
    const T = series[0][0].length;
    const oneStep = cur.problem.condition === "one-step";
    const chip = (color, dash, text) => `<span>${lineChip(color, dash)}${text}</span>`;
    this.plotLegend.innerHTML = chip(COLOR.gray, null, "backup prediction")
      + chip(COLOR.purple, "4 3", "prediction of your control")
      + (cur.out.status !== "off" ? chip(COLOR.blue, null, "prediction of the filter output") : "")
      + chip(COLOR.red, null, oneStep ? "bound (1−λ) h<sub>e</sub> at t = 1, checked at t = 2 only" : "bound (1−λ) B<sub>e</sub>, from t = 2 on");
    let lo = Math.min(0, con.rhs), hi = Math.max(0.5, con.rhs);
    for (const [values] of series) for (const v of values) { lo = Math.min(lo, v); hi = Math.max(hi, v); }
    hi = Math.min(hi, Math.max(con.B, con.rhs, 0) + 3);
    lo = Math.max(lo, -3);
    const pad = 0.08 * (hi - lo);
    const tx = linear(1, T, m.l, s.w - m.r), hy = linear(lo - pad, hi + pad, s.h - m.b, m.t);
    ctx.save();
    ctx.beginPath();
    ctx.rect(m.l, m.t, s.w - m.l - m.r, s.h - m.t - m.b);
    ctx.clip();
    // Zero and the bound of the condition
    ctx.strokeStyle = COLOR.dark;
    ctx.setLineDash([2, 3]);
    ctx.beginPath(); ctx.moveTo(m.l, hy(0)); ctx.lineTo(s.w - m.r, hy(0)); ctx.stroke();
    ctx.setLineDash([]);
    ctx.strokeStyle = COLOR.red;
    ctx.lineWidth = 2;
    ctx.beginPath();
    if (oneStep) { ctx.moveTo(tx(1.6), hy(con.rhs)); ctx.lineTo(tx(2.4), hy(con.rhs)); } else { ctx.moveTo(tx(2), hy(con.rhs)); ctx.lineTo(tx(T), hy(con.rhs)); }
    ctx.stroke();
    for (const [values, color, dash] of series) {
      ctx.strokeStyle = color;
      ctx.fillStyle = color;
      ctx.lineWidth = 1.8;
      ctx.setLineDash(dash);
      ctx.beginPath();
      values.forEach((v, i) => (i ? ctx.lineTo(tx(i + 1), hy(v)) : ctx.moveTo(tx(i + 1), hy(v))));
      ctx.stroke();
      ctx.setLineDash([]);
    }
    // The step that the condition checks (one-step: t = 2 only)
    if (oneStep) for (const [values, color] of series) dot(ctx, tx(2), hy(values[1]), 3.5, color);
    ctx.restore();
    // Labels and axes
    ctx.font = "12px -apple-system, BlinkMacSystemFont, 'Segoe UI', Helvetica, Arial, sans-serif";
    ctx.fillStyle = COLOR.red;
    ctx.textAlign = "right";
    ctx.fillStyle = COLOR.dark;
    ctx.textAlign = "center";
    ctx.textBaseline = "top";
    const step = T > 16 ? 5 : 2;
    for (let t = 1; t <= T; t++) if (t === 1 || t % step === 0) ctx.fillText(String(t), tx(t), s.h - m.b + 4);
    ctx.fillText("prediction step t", (m.l + s.w - m.r) / 2, s.h - m.b + 19);
    ctx.textAlign = "right";
    ctx.textBaseline = "middle";
    for (const v of niceTicks(lo - pad, hi + pad)) ctx.fillText(fmt(v, Math.abs(v) < 10 && v % 1 ? 1 : 0), m.l - 5, hy(v));
    ctx.save();
    ctx.translate(13, (m.t + s.h - m.b) / 2);
    ctx.rotate(-Math.PI / 2);
    ctx.textAlign = "center";
    ctx.fillText(`margin [m]`, 0, 0);
    ctx.restore();
    ctx.textBaseline = "alphabetic";
    ctx.strokeStyle = COLOR.dark;
    ctx.lineWidth = 1;
    ctx.strokeRect(m.l, m.t, s.w - m.l - m.r, s.h - m.t - m.b);
  }
}

// ---------------------------------------------------------------------------
// Drawing helpers

/** Constraint of the margin plot (auto): the one your control violates most,
 * or else the one with the smallest backup value. */
function autoConstraint(cur) {
  if (cur.policy.slack < -PARAMS.tol) return cur.policy.worst;
  const B = cur.problem.constraints.map((con) => con.B);
  return B.indexOf(Math.min(...B));
}

/** A short line (dashed if dash) as a legend chip. */
function lineChip(color, dash = null) {
  return `<svg class="bw-line" width="20" height="8" aria-hidden="true"><line x1="1" y1="4" x2="19" y2="4" stroke="${color}"`
    + ` stroke-width="2.2"${dash ? ` stroke-dasharray="${dash}"` : ""}/></svg>`;
}

function linear(d0, d1, r0, r1) {
  const f = (v) => r0 + ((v - d0) / (d1 - d0)) * (r1 - r0);
  f.inv = (p) => d0 + ((p - r0) / (r1 - r0)) * (d1 - d0);
  return f;
}

function niceTicks(lo, hi) {
  const span = hi - lo, raw = span / 4, mag = 10 ** Math.floor(Math.log10(raw));
  const step = [1, 2, 5, 10].map((k) => k * mag).find((k) => k >= raw);
  const ticks = [];
  for (let v = Math.ceil(lo / step) * step; v <= hi + 1e-9; v += step) ticks.push(Math.abs(v) < 1e-9 ? 0 : v);
  return ticks;
}

function polyline(ctx, points) {
  ctx.beginPath();
  points.forEach(([x, y], i) => (i ? ctx.lineTo(x, y) : ctx.moveTo(x, y)));
  ctx.stroke();
}

function rectPath(ctx, pose) {
  const c = corners(pose);
  ctx.beginPath();
  c.forEach(([x, y], i) => (i ? ctx.lineTo(x, y) : ctx.moveTo(x, y)));
  ctx.closePath();
}

function vehicle(ctx, pose, color, px) {
  rectPath(ctx, pose);
  ctx.fillStyle = color;
  ctx.fill();
  // Heading chevron
  const c = Math.cos(pose[2]), s = Math.sin(pose[2]);
  const at = (l, w) => [pose[0] + l * c - w * s, pose[1] + l * s + w * c];
  ctx.beginPath();
  for (const [i, [l, w]] of [[0.9, 0.55], [1.7, 0], [0.9, -0.55]].entries()) {
    const [x, y] = at(l, w);
    i ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
  }
  ctx.strokeStyle = "#fff";
  ctx.lineWidth = 2 * px;
  ctx.stroke();
}

function footprints(ctx, poses, color, px, { fill = null, dash = [], alpha = 0.7 } = {}) {
  ctx.save();
  ctx.globalAlpha = alpha;
  ctx.strokeStyle = color;
  ctx.lineWidth = 1 * px;
  ctx.setLineDash(dash);
  const stride = poses.length > 14 ? 2 : 1;
  poses.forEach((pose, t) => {
    if (t % stride) return;
    // Skip the standing-still repeats after the vehicle has stopped
    if (t > 0) {
      const q = poses[t - 1];
      if (Math.hypot(pose[0] - q[0], pose[1] - q[1]) < 1e-3) return;
    }
    rectPath(ctx, pose);
    if (fill) { ctx.fillStyle = fill; ctx.fill(); }
    ctx.stroke();
  });
  // The standstill footprint, outlined
  ctx.globalAlpha = 1;
  ctx.lineWidth = 1.6 * px;
  rectPath(ctx, poses[poses.length - 1]);
  ctx.stroke();
  ctx.restore();
}

function halo(ctx, text, x, y, color, align = null) {
  if (align) ctx.textAlign = align;
  ctx.lineWidth = 4;
  ctx.strokeStyle = "rgba(255,255,255,0.9)";
  ctx.lineJoin = "round";
  ctx.strokeText(text, x, y);
  ctx.fillStyle = color;
  ctx.fillText(text, x, y);
}

function box(ctx, text, x, y) {
  const w = ctx.measureText(text).width;
  ctx.fillStyle = "#fff";
  ctx.fillRect(x, y, w + 16, 24);
  ctx.fillStyle = COLOR.text;
  ctx.textBaseline = "middle";
  ctx.fillText(text, x + 8, y + 12);
  ctx.textBaseline = "alphabetic";
}

function dot(ctx, x, y, r, color) {
  ctx.beginPath();
  ctx.arc(x, y, r, 0, 2 * Math.PI);
  ctx.fillStyle = color;
  ctx.fill();
  ctx.strokeStyle = "#fff";
  ctx.lineWidth = 1.5;
  ctx.stroke();
}

function arrow(ctx, [x0, y0], [x1, y1], color) {
  const a = Math.atan2(y1 - y0, x1 - x0), h = 7;
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = 1.3;
  ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x1 - Math.cos(a) * h * 0.8, y1 - Math.sin(a) * h * 0.8); ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(x1 - h * Math.cos(a - 0.4), y1 - h * Math.sin(a - 0.4));
  ctx.lineTo(x1 - h * Math.cos(a + 0.4), y1 - h * Math.sin(a + 0.4));
  ctx.closePath();
  ctx.fill();
}

function star(ctx, x, y, r, color) {
  ctx.beginPath();
  for (let i = 0; i < 10; i++) {
    const rr = i % 2 ? r * 0.45 : r, a = -Math.PI / 2 + (i * Math.PI) / 5;
    i ? ctx.lineTo(x + rr * Math.cos(a), y + rr * Math.sin(a)) : ctx.moveTo(x + rr * Math.cos(a), y + rr * Math.sin(a));
  }
  ctx.closePath();
  ctx.fillStyle = color;
  ctx.fill();
  ctx.strokeStyle = "#fff";
  ctx.lineWidth = 1.5;
  ctx.stroke();
}

/** The part of a triangle [[x, y, value] x 3] where the linearly
 * interpolated value is nonnegative; pushes its boundary segment to edges. */
function clipTriangle(tri, edges) {
  const out = [], cut = [];
  for (let i = 0; i < 3; i++) {
    const p = tri[i], q = tri[(i + 1) % 3];
    if (p[2] >= 0) out.push(p);
    if ((p[2] >= 0) !== (q[2] >= 0)) {
      const f = p[2] / (p[2] - q[2]);
      const r = [p[0] + f * (q[0] - p[0]), p[1] + f * (q[1] - p[1]), 0];
      out.push(r);
      cut.push(r);
    }
  }
  if (cut.length === 2) edges.push(cut);
  return out;
}
