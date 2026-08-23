/* Taiphoon Agent 離線規劃模擬器的瀏覽器介面。 */

const DEFAULT_SCENARIO = {
  path_constraint: "fixed_z",
  sample_spacing_m: 0.5,
  self: {
    position_ned: [0, 0, -5],
    velocity_ned: [0, 0, 0],
    acceleration_ned: [0, 0, 0],
    goal_ned: [30, 0, -5]
  },
  planner: {
    control_point_count: 6,
    knot_interval_s: 1,
    sample_interval_s: 0.1,
    obstacle_clearance_m: 1,
    peer_collision_min_distance_m: 3,
    max_velocity_mps: 2.5,
    max_acceleration_mps2: 1.5,
    max_jerk_mps3: 3,
    optimizer_max_iterations: 40,
    optimizer_time_budget_ms: 50,
    candidate_offset_m: 3
  },
  obstacles: [
    { type: "box", center_ned: [12, 4, -3], size_m: [4, 4, 6] },
    { type: "cylinder", center_ned: [22, -5, -3], radius_m: 2, height_m: 6 }
  ],
  peers: [
    { agent_id: 2, mode: "hold", start_ned: [10, 7, -5], goal_ned: [10, 7, -5] }
  ]
};

let scenario = clone(DEFAULT_SCENARIO);
let lastResult = null;
let animationTimer = null;

function clone(value) {
  return JSON.parse(JSON.stringify(value));
}

function el(id) {
  return document.getElementById(id);
}

function numberValue(value, fallback = 0) {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
}

function format(value, digits = 2) {
  if (!Number.isFinite(Number(value))) return "—";
  return Number(value).toFixed(digits);
}

function makeNumberInput(value, data, step = "0.1") {
  const input = document.createElement("input");
  input.type = "number";
  input.step = step;
  input.value = value;
  Object.entries(data).forEach(([key, item]) => input.dataset[key] = item);
  return input;
}

function renderBaseInputs() {
  document.querySelectorAll("[data-self]").forEach((input) => {
    const key = input.dataset.self;
    const index = Number(input.dataset.index);
    input.value = scenario.self[key][index];
  });
  document.querySelectorAll("[data-planner]").forEach((input) => {
    input.value = scenario.planner[input.dataset.planner];
  });
  el("path-constraint").value = scenario.path_constraint;
  el("sample-spacing").value = scenario.sample_spacing_m;
}

function renderObstacles() {
  const list = el("obstacles-list");
  list.innerHTML = "";
  scenario.obstacles.forEach((obstacle, index) => {
    const item = document.createElement("div");
    item.className = "dynamic-item";
    const title = document.createElement("div");
    title.className = "dynamic-item-header";
    title.innerHTML = `<strong>${obstacle.type.toUpperCase()} #${index + 1}</strong>`;
    const remove = document.createElement("button");
    remove.type = "button";
    remove.textContent = "移除";
    remove.addEventListener("click", () => {
      scenario.obstacles.splice(index, 1);
      renderObstacles();
    });
    title.appendChild(remove);
    item.appendChild(title);

    const center = document.createElement("div");
    center.className = "field-grid three";
    ["N", "E", "D"].forEach((axis, axisIndex) => {
      const label = document.createElement("label");
      label.textContent = `中心 ${axis}`;
      const input = makeNumberInput(obstacle.center_ned[axisIndex], { obstacle: index, field: "center_ned", index: axisIndex });
      input.addEventListener("input", () => obstacle.center_ned[axisIndex] = numberValue(input.value));
      label.appendChild(input);
      center.appendChild(label);
    });
    item.appendChild(center);

    if (obstacle.type === "cylinder") {
      const geometry = document.createElement("div");
      geometry.className = "field-grid";
      [["半徑", "radius_m"], ["高度", "height_m"]].forEach(([caption, field]) => {
        const label = document.createElement("label");
        label.textContent = caption;
        const input = makeNumberInput(obstacle[field], { obstacle: index, field });
        input.addEventListener("input", () => obstacle[field] = numberValue(input.value));
        label.appendChild(input);
        geometry.appendChild(label);
      });
      item.appendChild(geometry);
    } else {
      const size = document.createElement("div");
      size.className = "field-grid three";
      ["X", "Y", "Z"].forEach((axis, axisIndex) => {
        const label = document.createElement("label");
        label.textContent = `尺寸 ${axis}`;
        const input = makeNumberInput(obstacle.size_m[axisIndex], { obstacle: index, field: "size_m", index: axisIndex });
        input.addEventListener("input", () => obstacle.size_m[axisIndex] = numberValue(input.value, 1));
        label.appendChild(input);
        size.appendChild(label);
      });
      item.appendChild(size);
    }
    list.appendChild(item);
  });
}

function renderPeers() {
  const list = el("peers-list");
  list.innerHTML = "";
  scenario.peers.forEach((peer, index) => {
    const item = document.createElement("div");
    item.className = "dynamic-item";
    const title = document.createElement("div");
    title.className = "dynamic-item-header";
    title.innerHTML = `<strong>fmu${peer.agent_id} / ${peer.mode === "hold" ? "懸停" : "移動"}</strong>`;
    const remove = document.createElement("button");
    remove.type = "button";
    remove.textContent = "移除";
    remove.addEventListener("click", () => {
      scenario.peers.splice(index, 1);
      renderPeers();
    });
    title.appendChild(remove);
    item.appendChild(title);
    const fields = document.createElement("div");
    fields.className = "field-grid";
    const idLabel = document.createElement("label");
    idLabel.textContent = "Agent ID";
    const idInput = makeNumberInput(peer.agent_id, { peer: index, field: "agent_id" }, "1");
    idInput.min = "1";
    idInput.addEventListener("input", () => peer.agent_id = Math.max(1, Math.round(numberValue(idInput.value, 2))));
    idLabel.appendChild(idInput);
    fields.appendChild(idLabel);
    const modeLabel = document.createElement("label");
    modeLabel.textContent = "模式";
    const select = document.createElement("select");
    select.innerHTML = "<option value=hold>懸停</option><option value=move>移動</option>";
    select.value = peer.mode;
    select.addEventListener("change", () => { peer.mode = select.value; renderPeers(); });
    modeLabel.appendChild(select);
    fields.appendChild(modeLabel);
    item.appendChild(fields);
    item.appendChild(peerVectorFields(peer, "start_ned", "起點"));
    if (peer.mode === "move") item.appendChild(peerVectorFields(peer, "goal_ned", "目標"));
    list.appendChild(item);
  });
}

function peerVectorFields(peer, key, caption) {
  const wrapper = document.createElement("div");
  wrapper.className = "field-grid three";
  ["N", "E", "D"].forEach((axis, axisIndex) => {
    const label = document.createElement("label");
    label.textContent = `${caption} ${axis}`;
    const input = makeNumberInput(peer[key][axisIndex], { field: key, index: axisIndex });
    input.addEventListener("input", () => peer[key][axisIndex] = numberValue(input.value));
    label.appendChild(input);
    wrapper.appendChild(label);
  });
  return wrapper;
}

function bindStaticInputs() {
  document.querySelectorAll("[data-self]").forEach((input) => input.addEventListener("input", () => {
    scenario.self[input.dataset.self][Number(input.dataset.index)] = numberValue(input.value);
  }));
  document.querySelectorAll("[data-planner]").forEach((input) => input.addEventListener("input", () => {
    const key = input.dataset.planner;
    scenario.planner[key] = key === "control_point_count" || key === "optimizer_max_iterations" ? Math.round(numberValue(input.value)) : numberValue(input.value);
  }));
  el("path-constraint").addEventListener("change", (event) => scenario.path_constraint = event.target.value);
  el("sample-spacing").addEventListener("input", (event) => scenario.sample_spacing_m = numberValue(event.target.value, .5));
  document.querySelectorAll("[data-add-obstacle]").forEach((button) => button.addEventListener("click", () => {
    const type = button.dataset.addObstacle;
    scenario.obstacles.push(type === "cylinder"
      ? { type, center_ned: [12, 0, -3], radius_m: 2, height_m: 6 }
      : { type, center_ned: [12, 0, -3], size_m: type === "wall" ? [0.5, 20, 6] : [4, 8, 6] });
    renderObstacles();
  }));
  el("add-hold-peer").addEventListener("click", () => {
    scenario.peers.push({ agent_id: nextPeerId(), mode: "hold", start_ned: [10, 5, -5], goal_ned: [10, 5, -5] });
    renderPeers();
  });
  el("add-moving-peer").addEventListener("click", () => {
    scenario.peers.push({ agent_id: nextPeerId(), mode: "move", start_ned: [10, 5, -5], goal_ned: [0, -10, -5] });
    renderPeers();
  });
  el("plan-button").addEventListener("click", requestPlan);
  el("reset-button").addEventListener("click", () => { scenario = clone(DEFAULT_SCENARIO); renderAll(); requestPlan(); });
  el("export-button").addEventListener("click", exportScenario);
  el("import-input").addEventListener("change", importScenario);
  el("time-slider").addEventListener("input", () => renderFrame(Number(el("time-slider").value)));
  el("animate-button").addEventListener("click", toggleAnimation);
}

function nextPeerId() {
  return scenario.peers.reduce((max, peer) => Math.max(max, peer.agent_id), 1) + 1;
}

function renderAll() {
  renderBaseInputs();
  renderObstacles();
  renderPeers();
}

async function requestPlan() {
  const message = el("message");
  message.className = "message";
  message.textContent = "規劃中…";
  try {
    const response = await fetch("/plan", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(scenario) });
    const data = await response.json();
    if (!response.ok || !data.ok) throw new Error(data.error || "planner 執行失敗");
    lastResult = data;
    el("time-slider").max = String(data.planner.duration_s || 1);
    el("time-slider").value = "0";
    renderMetrics(data);
    renderPlots(0);
    message.textContent = `完成：${data.planner.termination}，Local Goal ${formatVector(data.input.local_goal)}，安全檢查 ${data.safety.is_safe ? "通過" : "未通過"}`;
  } catch (error) {
    lastResult = null;
    message.className = "message error";
    message.textContent = error.message;
    renderMetrics(null);
    clearPlot(el("top-plot"));
    clearPlot(el("side-plot"));
  }
}

function renderMetrics(data) {
  const target = el("metrics");
  target.innerHTML = "";
  if (!data) { target.innerHTML = '<p class="muted">執行規劃後顯示。</p>'; return; }
  const localGoal = data.input.local_goal;
  const finalGoal = data.input.goal;
  const values = [
    ["Final Target", formatVector(finalGoal)],
    ["Local Goal", formatVector(localGoal)],
    ["Planner", data.planner.termination],
    ["安全檢查", data.safety.is_safe ? "PASS" : data.safety.reason],
    ["障礙最小距離", scenario.obstacles.length ? `${format(data.safety.minimum_obstacle_distance_m)} m` : "—"],
    ["Peer 最小距離", scenario.peers.length ? `${format(data.safety.minimum_peer_distance_m)} m` : "—"],
    ["規劃耗時", `${format(data.planner.planning_time_ms, 1)} ms`],
    ["迭代次數", data.planner.iteration_count],
    ["最大速度", `${format(data.safety.maximum_velocity_mps)} m/s`],
    ["軌跡時間", `${format(data.planner.duration_s)} s`]
  ];
  values.forEach(([label, value]) => {
    const card = document.createElement("div");
    card.className = "metric";
    card.innerHTML = `<div class="metric-label">${label}</div><div class="metric-value">${value}</div>`;
    target.appendChild(card);
  });
}

function formatVector(point) {
  if (!point || point.length !== 3) return "—";
  return `[${format(point[0])}, ${format(point[1])}, ${format(point[2])}]`;
}

function svgElement(name, attributes = {}) {
  const node = document.createElementNS("http://www.w3.org/2000/svg", name);
  Object.entries(attributes).forEach(([key, value]) => node.setAttribute(key, value));
  return node;
}

function clearPlot(svg) { svg.replaceChildren(); }

function allPositions() {
  const values = [scenario.self.position_ned, scenario.self.goal_ned];
  scenario.obstacles.forEach((obstacle) => values.push(obstacle.center_ned));
  scenario.peers.forEach((peer) => { values.push(peer.start_ned); values.push(peer.goal_ned); });
  if (lastResult) {
    lastResult.samples.forEach((sample) => values.push(sample.position));
    lastResult.control_points.forEach((point) => values.push(point));
    lastResult.peers.forEach((peer) => peer.samples.forEach((sample) => values.push(sample.position)));
  }
  return values;
}

function bounds(axis) {
  const values = allPositions().map((point) => [Number(point[axis[0]]), Number(point[axis[1]])]);
  scenario.obstacles.forEach((obstacle) => {
    const c = obstacle.center_ned;
    if (obstacle.type === "cylinder") {
      values.push([c[axis[0]] - obstacle.radius_m, c[axis[1]] - obstacle.radius_m]);
      values.push([c[axis[0]] + obstacle.radius_m, c[axis[1]] + obstacle.radius_m]);
    } else {
      const half = obstacle.size_m.map((item) => item / 2);
      values.push([c[axis[0]] - half[axis[0]], c[axis[1]] - half[axis[1]]]);
      values.push([c[axis[0]] + half[axis[0]], c[axis[1]] + half[axis[1]]]);
    }
  });
  const x = values.map((item) => item[0]);
  const y = values.map((item) => item[1]);
  const minX = Math.min(...x, 0), maxX = Math.max(...x, 0), minY = Math.min(...y, 0), maxY = Math.max(...y, 0);
  const padX = Math.max((maxX - minX) * .14, 3), padY = Math.max((maxY - minY) * .14, 3);
  return { minX: minX - padX, maxX: maxX + padX, minY: minY - padY, maxY: maxY + padY };
}

function makeProject(axis, width, height) {
  const b = bounds(axis);
  const sx = (width - 74) / Math.max(b.maxX - b.minX, 1);
  const sy = (height - 58) / Math.max(b.maxY - b.minY, 1);
  const scale = Math.min(sx, sy);
  const xOffset = 55 + ((width - 74) - (b.maxX - b.minX) * scale) / 2;
  const yOffset = 25 + ((height - 58) - (b.maxY - b.minY) * scale) / 2;
  return {
    project: (point) => [xOffset + (point[axis[0]] - b.minX) * scale, yOffset + (point[axis[1]] - b.minY) * scale],
    x: (value) => xOffset + (value - b.minX) * scale,
    y: (value) => yOffset + (value - b.minY) * scale,
    bounds: b
  };
}

function drawGrid(svg, projector, width, height, labels) {
  const { minX, maxX, minY, maxY } = projector.bounds;
  for (let i = 0; i <= 5; i += 1) {
    const x = minX + (maxX - minX) * i / 5;
    const y = minY + (maxY - minY) * i / 5;
    svg.appendChild(svgElement("line", { x1: projector.x(x), y1: 20, x2: projector.x(x), y2: height - 28, class: "grid" }));
    svg.appendChild(svgElement("line", { x1: 50, y1: projector.y(y), x2: width - 18, y2: projector.y(y), class: "grid" }));
    const xText = svgElement("text", { x: projector.x(x), y: height - 9, "text-anchor": "middle" });
    xText.textContent = format(x, 0);
    svg.appendChild(xText);
    const yText = svgElement("text", { x: 43, y: projector.y(y) + 4, "text-anchor": "end" });
    yText.textContent = format(y, 0);
    svg.appendChild(yText);
  }
  const xLabel = svgElement("text", { x: width - 16, y: height - 9, "text-anchor": "end" });
  xLabel.textContent = labels[0];
  svg.appendChild(xLabel);
  const yLabel = svgElement("text", { x: 54, y: 17 });
  yLabel.textContent = labels[1];
  svg.appendChild(yLabel);
}

function polyline(svg, points, projector, className) {
  if (!points || points.length === 0) return;
  const value = points.map((point) => projector.project(point).join(",")).join(" ");
  svg.appendChild(svgElement("polyline", { points: value, class: className }));
}

function drawObstacle(svg, obstacle, projector, axis) {
  const center = obstacle.center_ned;
  const [cx, cy] = projector.project(center);
  if (obstacle.type === "cylinder") {
    const radius = obstacle.radius_m;
    const radiusX = Math.abs(projector.project([center[0] + radius, center[1], center[2]])[0] - cx);
    const radiusY = axis[1] === 2
      ? Math.abs(projector.project([center[0], center[1], center[2] + obstacle.height_m / 2])[1] - cy)
      : Math.abs(projector.project([center[0], center[1] + radius, center[2]])[1] - cy);
    svg.appendChild(svgElement("ellipse", { cx, cy, rx: radiusX, ry: radiusY, class: "obstacle-shape" }));
    return;
  }
  const half = obstacle.size_m.map((item) => item / 2);
  const p1 = projector.project([center[0] - half[0], center[1] - half[1], center[2] - half[2]]);
  const p2 = projector.project([center[0] + half[0], center[1] + half[1], center[2] + half[2]]);
  svg.appendChild(svgElement("rect", { x: Math.min(p1[0], p2[0]), y: Math.min(p1[1], p2[1]), width: Math.abs(p2[0] - p1[0]), height: Math.abs(p2[1] - p1[1]), class: "obstacle-shape" }));
}

function nearestSample(samples, time) {
  if (!samples || samples.length === 0) return null;
  return samples.reduce((best, sample) => Math.abs(sample.t - time) < Math.abs(best.t - time) ? sample : best, samples[0]);
}

function drawScene(svg, axis, labels, width, height, time) {
  clearPlot(svg);
  const projector = makeProject(axis, width, height);
  drawGrid(svg, projector, width, height, labels);
  scenario.obstacles.forEach((obstacle) => drawObstacle(svg, obstacle, projector, axis));
  if (!lastResult) return;
  const planPoints = lastResult.samples.map((sample) => sample.position);
  polyline(svg, planPoints, projector, "planned-line");
  polyline(svg, lastResult.control_points, projector, "control-line");
  lastResult.control_points.forEach((point) => {
    const [x, y] = projector.project(point);
    svg.appendChild(svgElement("circle", { cx: x, cy: y, r: 4, class: "control-dot" }));
  });
  lastResult.peers.forEach((peer) => {
    polyline(svg, peer.samples.map((sample) => sample.position), projector, "peer-line");
    const peerInput = scenario.peers.find((item) => item.agent_id === peer.agent_id);
    const current = nearestSample(peer.samples, time) || { position: peerInput?.start_ned || [0, 0, 0] };
    const [x, y] = projector.project(current.position);
    svg.appendChild(svgElement("circle", { cx: x, cy: y, r: 7, class: "peer-dot" }));
  });
  const self = nearestSample(lastResult.samples, time);
  if (self) {
    const [x, y] = projector.project(self.position);
    svg.appendChild(svgElement("circle", { cx: x, cy: y, r: 8, class: "actual-dot" }));
  }
  const finalGoal = projector.project(scenario.self.goal_ned);
  svg.appendChild(svgElement("circle", { cx: finalGoal[0], cy: finalGoal[1], r: 7, class: "goal-dot" }));
  const localGoal = projector.project(lastResult.input?.local_goal || scenario.self.goal_ned);
  svg.appendChild(svgElement("circle", { cx: localGoal[0], cy: localGoal[1], r: 6, class: "local-goal-dot" }));
}

function renderPlots(time) {
  drawScene(el("top-plot"), [0, 1], ["North (m)", "East (m)"], 800, 520, time);
  drawScene(el("side-plot"), [0, 2], ["North (m)", "Down (m)"], 800, 360, time);
  el("time-value").textContent = `t = ${format(time)} s`;
}

function renderFrame(time) {
  const max = Number(el("time-slider").max || 1);
  const clamped = Math.max(0, Math.min(max, time));
  el("time-slider").value = clamped;
  if (lastResult) renderPlots(clamped);
}

function toggleAnimation() {
  if (animationTimer) {
    clearInterval(animationTimer);
    animationTimer = null;
    el("animate-button").textContent = "播放";
    return;
  }
  el("animate-button").textContent = "暫停";
  animationTimer = setInterval(() => {
    const slider = el("time-slider");
    const next = Number(slider.value) + 0.1;
    if (next > Number(slider.max)) { renderFrame(0); toggleAnimation(); return; }
    renderFrame(next);
  }, 100);
}

function exportScenario() {
  const blob = new Blob([JSON.stringify(scenario, null, 2)], { type: "application/json" });
  const link = document.createElement("a");
  link.href = URL.createObjectURL(blob);
  link.download = "taiphoon-planning-scenario.json";
  link.click();
  URL.revokeObjectURL(link.href);
}

function importScenario(event) {
  const file = event.target.files?.[0];
  if (!file) return;
  const reader = new FileReader();
  reader.onload = () => {
    try {
      const parsed = JSON.parse(reader.result);
      if (!parsed.self || !parsed.planner) throw new Error("缺少 self 或 planner");
      scenario = parsed;
      renderAll();
      requestPlan();
    } catch (error) {
      el("message").className = "message error";
      el("message").textContent = `JSON 載入失敗：${error.message}`;
    }
  };
  reader.readAsText(file);
}

bindStaticInputs();
renderAll();
requestPlan();
