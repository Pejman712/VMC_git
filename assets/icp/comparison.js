"use strict";
/* Evaluate in the shared first-scan frame. Ground truth never enters the solver. */
window.ICPComparison = (() => {
  const $ = (id) => document.getElementById(id);
  const position = (T) => [T[0][3], T[1][3], T[2][3]];
  let hitTargets = {}, select = () => {};

  function evaluate(truth, results) {
    const rows = results.map((r) => {
      const actual = truth[r.index], estimated = position(r.pose), expected = position(actual);
      const positionError = Math.hypot(...estimated.map((v, i) => v - expected[i]));
      // trace(R_true.T @ R_est) is the Frobenius inner product of the rotations.
      let trace = 0;
      for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) trace += actual[i][j] * r.pose[i][j];
      const rotationError = Math.acos(Math.max(-1, Math.min(1, (trace - 1) / 2))) * 180 / Math.PI;
      return { index: r.index, estimated, expected, positionError, rotationError };
    });
    return { rows, rmse: rows.length ? Math.sqrt(rows.reduce((s, r) => s + r.positionError ** 2, 0) / rows.length) : null };
  }

  function chart(id, xBounds, yBounds, xLabel, yLabel, equalAspect = false) {
    const canvas = $(id), box = canvas.getBoundingClientRect(), ratio = Math.min(devicePixelRatio || 1, 2);
    canvas.width = Math.round(box.width * ratio); canvas.height = Math.round(box.height * ratio);
    const ctx = canvas.getContext("2d"), w = box.width, h = box.height;
    ctx.scale(ratio, ratio);
    const style = getComputedStyle(document.documentElement);
    const ink = style.getPropertyValue("--ink").trim(), muted = style.getPropertyValue("--muted").trim();
    const border = style.getPropertyValue("--border").trim();
    ctx.fillStyle = style.getPropertyValue("--surface").trim(); ctx.fillRect(0, 0, w, h);
    const left = 54, top = 23, plotW = Math.max(1, w - 75), plotH = Math.max(1, h - 66);
    let [x0, x1] = xBounds, [y0, y1] = yBounds;
    if (equalAspect) {
      const scale = Math.min(plotW / (x1 - x0), plotH / (y1 - y0));
      const xc = (x0 + x1) / 2, yc = (y0 + y1) / 2;
      x0 = xc - plotW / scale / 2; x1 = xc + plotW / scale / 2;
      y0 = yc - plotH / scale / 2; y1 = yc + plotH / scale / 2;
    }
    const project = (x, y) => [left + (x - x0) / (x1 - x0) * plotW, top + (1 - (y - y0) / (y1 - y0)) * plotH];
    ctx.font = "10px monospace"; ctx.lineWidth = 1;
    for (let tick = 0; tick <= 4; tick++) {
      const rawX = x0 + (x1 - x0) * tick / 4;
      const x = xLabel === "Scan" ? Math.round(rawX) : rawX, y = y0 + (y1 - y0) * tick / 4;
      const px = project(x, y0)[0], py = project(x0, y)[1];
      ctx.strokeStyle = border; ctx.beginPath(); ctx.moveTo(px, top); ctx.lineTo(px, top + plotH);
      ctx.moveTo(left, py); ctx.lineTo(left + plotW, py); ctx.stroke();
      ctx.fillStyle = muted; ctx.textAlign = "center";
      ctx.fillText(xLabel === "Scan" ? Math.round(x).toString() : x.toFixed(1), px, top + plotH + 18);
      ctx.textAlign = "right"; ctx.fillText(y.toFixed(y1 - y0 < 1 ? 2 : 1), left - 8, py + 3);
    }
    ctx.fillStyle = ink; ctx.textAlign = "center"; ctx.fillText(xLabel, left + plotW / 2, h - 7);
    ctx.textAlign = "left"; ctx.fillText(yLabel, 12, 12);
    hitTargets[id] = [];
    function series(points, color, dashed = false) {
      ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.setLineDash(dashed ? [6, 4] : []);
      ctx.beginPath(); points.forEach((p, i) => { const xy = project(p[0], p[1]); i ? ctx.lineTo(...xy) : ctx.moveTo(...xy); });
      ctx.stroke(); ctx.setLineDash([]);
    }
    function marker(x, y, color, index, selected = false) {
      const xy = project(x, y);
      ctx.beginPath(); ctx.arc(...xy, selected ? 5 : 2.2, 0, Math.PI * 2);
      ctx.fillStyle = color; ctx.fill();
      if (selected) { ctx.strokeStyle = ink; ctx.lineWidth = 1; ctx.stroke(); }
      hitTargets[id].push({ x: xy[0], y: xy[1], index });
    }
    function message(text) {
      ctx.fillStyle = muted; ctx.textAlign = "center"; ctx.font = "12px sans-serif";
      ctx.fillText(text, left + plotW / 2, top + plotH / 2);
    }
    return { series, marker, message };
  }

  function render(truth, results, frame, dirty) {
    const { rows, rmse } = evaluate(truth, results), selected = rows.find((r) => r.index === frame);
    const last = rows.at(-1), count = truth.length;
    const gtColor = "#15947b", estimateColor = "#d16a10";
    $("comparisonStatus").textContent = rows.length ? `${rows.length} / ${count} computed poses${dirty ? " · code changed since run" : ""}` : "No estimated poses yet";
    $("positionError").textContent = selected ? selected.positionError.toFixed(3) + " m" : "—";
    $("rotationError").textContent = selected ? selected.rotationError.toFixed(2) + "°" : "—";
    $("trajectoryRmse").textContent = rmse == null ? "—" : rmse.toFixed(3) + " m";
    $("endError").textContent = last ? last.positionError.toFixed(3) + " m" : "—";
    $("endErrorLabel").textContent = rows.length === count ? "Final position error" : "Latest position error";
    $("comparisonNote").textContent = !rows.length ? "Run your registration or import results to compare them with the known poses. The ground-truth route is shown now." :
      `Statistics use ${rows.length} computed poses, including the fixed origin. Selected scan: ${frame + 1}${selected ? "" : " (not computed yet)"}. These plots always show your last computed results, including while previewing the expected map.${dirty ? " Run the edited code to refresh them." : ""}`;

    const gt = truth.map(position), all = [...gt, ...rows.map((r) => r.estimated)];
    const xs = all.map((p) => p[0]), ys = all.map((p) => p[1]);
    const trajectory = chart("trajectoryPlot", [Math.min(...xs) - 1, Math.max(...xs) + 1],
      [Math.min(...ys) - 1, Math.max(...ys) + 1], "X (m)", "Y (m)", true);
    trajectory.series(gt.map((p) => p.slice(0, 2)), gtColor, true);
    gt.forEach((p, i) => trajectory.marker(p[0], p[1], gtColor, i, i === frame));
    trajectory.series(rows.map((r) => r.estimated.slice(0, 2)), estimateColor);
    rows.forEach((r) => trajectory.marker(r.estimated[0], r.estimated[1], estimateColor, r.index, r.index === frame));

    const errorMax = Math.max(.1, ...rows.map((r) => r.positionError)) * 1.15;
    const error = chart("errorPlot", [1, Math.max(2, count)], [0, errorMax], "Scan", "3D position error (m)");
    error.series([[1, 0], [count, 0]], gtColor, true);
    error.series(rows.map((r) => [r.index + 1, r.positionError]), estimateColor);
    rows.forEach((r) => error.marker(r.index + 1, r.positionError, estimateColor, r.index, r.index === frame));
    if (!rows.length) error.message("Run Python to measure error");

    // A textual equivalent of the selected points is available without canvas.
    $("comparisonValues").textContent = `Scan ${frame + 1} ground truth XYZ: ${gt[frame].map((v) => v.toFixed(3)).join(", ")} m. ` +
      (selected ? `Estimated XYZ: ${selected.estimated.map((v) => v.toFixed(3)).join(", ")} m.` : "Estimated XYZ: not available.");
  }

  function init(onSelect) {
    select = onSelect;
    for (const id of ["trajectoryPlot", "errorPlot"]) {
      $(id).onclick = (event) => {
        const rect = event.currentTarget.getBoundingClientRect(), x = event.clientX - rect.left, y = event.clientY - rect.top;
        const targets = hitTargets[id] || [];
        const nearest = targets.reduce((best, p) => {
          const distance = (p.x - x) ** 2 + (p.y - y) ** 2;
          return !best || distance < best.distance ? { ...p, distance } : best;
        }, null);
        if (nearest) select(nearest.index);
      };
    }
  }
  return { evaluate, render, init };
})();
