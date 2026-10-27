"use strict";
(() => {
  const $ = (id) => document.getElementById(id);
  const identity = () => [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]];
  const multiply = (a, b) => a.map((row) => b[0].map((_, j) => row.reduce((s, v, k) => s + v * b[k][j], 0)));
  const transform = (T, p) => T.slice(0, 3).map((r) => r[0] * p[0] + r[1] * p[1] + r[2] * p[2] + r[3]);
  const inverse = (T) => {
    const out = identity();
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) out[i][j] = T[j][i];
      out[i][3] = -out[i].slice(0, 3).reduce((s, v, k) => s + v * T[k][3], 0);
    }
    return out;
  };
  const pad = (n) => String(n + 1).padStart(2, "0");
  const KEY = "vmc-icp-python-v1";
  let dataset, expectedPoses, starter, sample, frame = 0, mode = "input", results = [];
  let worker = null, timeout = null, playback = null, revision = "", dirty = false;
  const transformedClouds = new WeakMap();
  const scanCount = () => dataset.scans.length;
  const camera = { yaw: -.52, pitch: .83, zoom: 1 };

  function log(message, error = false) {
    $("console").textContent = ($( "console").textContent + "\n" + message).slice(-16000);
    $("console").classList.toggle("error", error);
    $("console").scrollTop = $("console").scrollHeight;
  }
  function status(message) { $("status").textContent = message; }
  function download(name, text, type = "application/json") {
    const url = URL.createObjectURL(new Blob([text], { type }));
    const a = document.createElement("a");
    a.href = url; a.download = name; a.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  }
  function baseResult() { return { index: 0, relative: identity(), pose: identity(), rmse: null }; }
  function shownResults() {
    if (mode === "expected") return expectedPoses.map((pose, index) => ({ index, pose,
      relative: index ? multiply(inverse(expectedPoses[index - 1]), pose) : identity(), rmse: null }));
    return results.length ? results : [baseResult()];
  }
  function haltPlayback() {
    clearInterval(playback); playback = null; $("playBtn").textContent = "▶ Play sequence";
  }
  function selectFrame(index) {
    frame = Math.max(0, Math.min(scanCount() - 1, index));
    $("frameRange").value = frame;
    render();
  }
  function render() {
    if (!dataset) return;
    const visible = shownResults(), result = visible[frame];
    const mapped = visible.slice(0, frame + 1);
    $("frameRange").value = frame;
    $("frameCount").textContent = `${pad(frame)} / ${scanCount()}`;
    $("inputLabel").textContent = `Scan ${pad(frame)} · ${dataset.scans[frame].points.length} points`;
    $("modeBadge").textContent = mode === "expected" ? "Expected map · known poses" : mode === "result" ? (dirty ? "My result · code changed" : "My registration") : "Input preview";
    $("frameState").textContent = result ? (mode === "expected" ? "Known pose · comparison only" : frame === 0 ? "Map origin" : "Registered") : "Not registered yet · run your code";
    $("mapLabel").textContent = `Scans 01–${pad(mapped.length - 1)} · ${mode === "expected" ? "expected poses" : "map frame = scan 01"}`;
    $("mappedCount").textContent = `${mapped.length} / ${scanCount()}`;
    $("rmse").textContent = result?.rmse == null ? "—" : `${result.rmse.toFixed(4)} m`;
    const matrix = result?.[$("matrixType").value];
    $("matrix").textContent = matrix ? matrix.map((r) => "[ " + r.map((v) => (Math.abs(v) < .00005 ? 0 : v).toFixed(4).padStart(8)).join(" ") + " ]").join("\n") : "No transformation for this scan.\nRun your registration first.";
    $("matrixNote").textContent = mode === "expected" ? "Known synthetic pose, shown for comparison. This is not output from your code." : frame === 0 ? "Scan 01 fixes the map origin with an identity matrix." : dirty ? "Code has changed since this result. Run again to update the matrix and map." : "Returned transforms are composed in scan order. The robot follows your estimated poses.";
    $("poseReadout").textContent = result ? "Robot position: " + ["x", "y", "z"].map((axis, i) => `${axis} ${result.pose[i][3].toFixed(3)}`).join(" · ") + " m" : "Robot remains at the last registered pose.";
    $("expectedBtn").setAttribute("aria-pressed", mode === "expected");
    $("resultBtn").setAttribute("aria-pressed", mode === "result");
    $("timeline").querySelectorAll("button").forEach((b, i) => {
      b.classList.toggle("selected", i === frame);
      b.classList.toggle("processed", i < visible.length);
      b.setAttribute("aria-pressed", i === frame);
      b.title = `Scan ${pad(i)}${i < visible.length ? " · mapped" : " · input only"}`;
    });
    const selectedButton = $("timeline").children[frame];
    const timeline = $("timeline");
    if (selectedButton && timeline.scrollHeight > timeline.clientHeight) {
      const top = selectedButton.getBoundingClientRect().top - timeline.getBoundingClientRect().top;
      if (top < 0 || top + selectedButton.offsetHeight > timeline.clientHeight) timeline.scrollTop += top - timeline.clientHeight / 2;
    }
    draw($("inputCanvas"), [{ points: dataset.scans[frame].points, current: true }], [identity()], false);
    const clouds = mapped.map((r) => {
      if (!transformedClouds.has(r.pose)) transformedClouds.set(r.pose, dataset.scans[r.index].points.map((p) => transform(r.pose, p)));
      return { points: transformedClouds.get(r.pose), current: r.index === frame };
    });
    draw($("mapCanvas"), clouds, mapped.map((r) => r.pose), true);
    ICPComparison.render(expectedPoses, results, frame, dirty);
  }

  // Lightweight orthographic 3D canvas renderer: the measurements remain XYZ.
  // Both views share a camera so source/map geometry is easy to compare.
  function draw(canvas, clouds, poses, isMap) {
    const rect = canvas.getBoundingClientRect(), ratio = Math.min(devicePixelRatio || 1, 2);
    canvas.width = Math.round(rect.width * ratio); canvas.height = Math.round(rect.height * ratio);
    const ctx = canvas.getContext("2d");
    ctx.scale(ratio, ratio);
    const w = rect.width, h = rect.height, top = $("projection").value === "top";
    ctx.fillStyle = "#0b1422"; ctx.fillRect(0, 0, w, h);
    const bounds = dataset.map_bounds;
    const center = isMap ? bounds.min.map((v, i) => (v + bounds.max[i]) / 2) : [0, 0, 0];
    const span = isMap ? Math.max(bounds.max[0] - bounds.min[0], bounds.max[1] - bounds.min[1]) : dataset.sensor_range * 2;
    const scale = Math.min(w / (span * 1.18), h / (span * .88)) * camera.zoom;
    const c = Math.cos(camera.yaw), s = Math.sin(camera.yaw);
    function project(p) {
      const x = p[0] - center[0], y = p[1] - center[1], z = p[2];
      if (top) return [w / 2 + x * scale, h / 2 - y * scale, z];
      const a = x * c - y * s, b = x * s + y * c;
      return [w / 2 + a * scale, h / 2 + 30 - (b * Math.sin(camera.pitch) + z * Math.cos(camera.pitch)) * scale,
        z * Math.sin(camera.pitch) - b * Math.cos(camera.pitch)];
    }
    function line(a, b, color, width = 1) {
      const pa = project(a), pb = project(b);
      ctx.strokeStyle = color; ctx.lineWidth = width; ctx.beginPath(); ctx.moveTo(pa[0], pa[1]); ctx.lineTo(pb[0], pb[1]); ctx.stroke();
    }
    const grid = isMap ? bounds : { min: [-dataset.sensor_range, -dataset.sensor_range], max: [dataset.sensor_range, dataset.sensor_range] };
    for (let i = Math.floor(grid.min[0]); i <= grid.max[0]; i++) {
      line([i, grid.min[1], -.65], [i, grid.max[1], -.65], "#1c2a3b");
    }
    for (let i = Math.floor(grid.min[1]); i <= grid.max[1]; i++) {
      line([grid.min[0], i, -.65], [grid.max[0], i, -.65], "#1c2a3b");
    }
    const points = [];
    clouds.forEach((cloud) => cloud.points.forEach((p) => points.push({ screen: project(p), current: cloud.current })));
    points.sort((a, b) => a.screen[2] - b.screen[2]);
    points.forEach(({ screen: p, current }) => {
      ctx.fillStyle = current ? "#60dce9" : "#708da878";
      const r = current ? 2.1 : 1.5;
      ctx.fillRect(p[0] - r / 2, p[1] - r / 2, r, r);
    });
    for (let i = 1; i < poses.length; i++) {
      line(transform(poses[i - 1], [0, 0, -.35]), transform(poses[i], [0, 0, -.35]), "#ffc76a", 2.5);
    }
    const pose = poses[poses.length - 1] || identity();
    // Small differential-drive robot: body, wheels, LiDAR and heading arrow.
    const robot = (p) => project(transform(pose, p));
    function polygon(vertices, fill, stroke) {
      ctx.beginPath(); vertices.forEach((v, i) => { const p = robot(v); i ? ctx.lineTo(...p.slice(0, 2)) : ctx.moveTo(...p.slice(0, 2)); });
      ctx.closePath(); ctx.fillStyle = fill; ctx.fill(); ctx.strokeStyle = stroke; ctx.lineWidth = 1.3; ctx.stroke();
    }
    [-1, 1].forEach((side) => polygon([[-.26, side * .24, -.4], [.23, side * .24, -.4], [.23, side * .37, -.4], [-.26, side * .37, -.4]], "#15263b", "#a2b9d1"));
    polygon([[-.33, -.23, -.25], [.29, -.23, -.25], [.4, 0, -.25], [.29, .23, -.25], [-.33, .23, -.25]], "#ffc76a", "#ffe7bc");
    const lidar = robot([0, 0, 0]);
    ctx.beginPath(); ctx.arc(lidar[0], lidar[1], 3.5, 0, Math.PI * 2); ctx.fillStyle = "#fff2d3"; ctx.fill();
    line(transform(pose, [.3, 0, -.2]), transform(pose, [.95, 0, -.2]), "#ffe4aa", 2);
    line(transform(pose, [.95, 0, -.2]), transform(pose, [.72, .13, -.2]), "#ffe4aa", 2);
    line(transform(pose, [.95, 0, -.2]), transform(pose, [.72, -.13, -.2]), "#ffe4aa", 2);
    // Axes at the first sensor/map origin.
    [["X", [1, 0, 0], "#f08080"], ["Y", [0, 1, 0], "#72c795"], ["Z", [0, 0, 1], "#80aaff"]].forEach(([label, endpoint, color]) => {
      if (top && label === "Z") return;
      line([0, 0, 0], endpoint, color, 1.4);
      const p = project(endpoint); ctx.fillStyle = color; ctx.font = "10px monospace"; ctx.fillText(label, p[0] + 4, p[1]);
    });
    ctx.fillStyle = "#8da5be"; ctx.font = "10px monospace";
    ctx.fillText("1 m grid · " + (top ? "XY view" : "XYZ view"), 14, h - 14);
    if (isMap && mode === "expected") { ctx.fillStyle = "#ffc76a"; ctx.fillText("EXPECTED · known synthetic poses", 14, 21); }
  }

  function finishWorker() {
    clearTimeout(timeout); timeout = null;
    if (worker) worker.terminate(); worker = null;
    $("runBtn").disabled = false; $("stopBtn").disabled = true;
    $("starterBtn").disabled = false; $("solutionBtn").disabled = false; $("code").readOnly = false;
    $("importFile").disabled = false; $("playBtn").disabled = false;
    $("exportBtn").disabled = results.length !== scanCount();
    $("resultBtn").disabled = results.length < 2;
    render();
  }
  function armTimeout(ms, message) {
    clearTimeout(timeout);
    timeout = setTimeout(() => { finishWorker(); status(message); log(message, true); }, ms);
  }
  function run() {
    if (!dataset || worker) return;
    haltPlayback(); mode = "result"; frame = 0; results = []; dirty = false; revision = $("code").value;
    $("console").textContent = "Starting a fresh Python run…"; $("console").classList.remove("error");
    status("Loading Python. The first run may take a moment.");
    $("runBtn").disabled = true; $("stopBtn").disabled = false;
    $("starterBtn").disabled = true; $("solutionBtn").disabled = true; $("code").readOnly = true;
    $("importFile").disabled = true; $("exportBtn").disabled = true;
    $("resultBtn").disabled = true; $("playBtn").disabled = true;
    try {
      worker = new Worker("assets/icp/worker.js", { type: "module" });
      worker.onmessage = ({ data }) => {
        if (data.type === "status") { status(data.message); log(data.message); }
        if (data.type === "stdout") log(data.message);
        if (data.type === "running") {
          status("Running registration on consecutive scans…");
          armTimeout(30000, "Stopped: Python exceeded 30 seconds. Check your loop and run again.");
        }
        if (data.type === "frame") {
          results.push(data.result); mode = "result";
          selectFrame(data.result.index);
          status(`Registered ${results.length} / ${scanCount()} scans${data.result.index ? ` · scan ${pad(data.result.index)} → ${pad(data.result.index - 1)}` : " · origin initialized"}`);
          if (data.result.index) log(`Scan ${pad(data.result.index)} → ${pad(data.result.index - 1)} | NN RMSE ${data.result.rmse.toFixed(4)} m`);
          armTimeout(30000, "Stopped: a registration call exceeded 30 seconds. Check convergence or reduce iterations.");
        }
        if (data.type === "done") {
          finishWorker();
          const baseline = results.every((r) => r.relative.every((row, i) => row.every((v, j) => Math.abs(v - (i === j ? 1 : 0)) < 1e-9)));
          status(baseline ? "Identity baseline complete. The robot stays still; implement ICP to align the moving scans." : `Run complete: ${scanCount()} scans mapped. Compare your map and trajectory with the expected preview.`);
          log(baseline ? "All returned matrices are identity. Valid format; registration still needs implementing." : `Complete. All ${scanCount() - 1} returned matrices passed rigid-transform validation.`);
        }
        if (data.type === "error") { finishWorker(); status("Python error. Check the output below your code, then run again."); log(data.message, true); }
      };
      worker.onerror = (event) => {
        event.preventDefault(); finishWorker();
        status("Could not run Python. Check your connection and serve this page over HTTP(S).");
        log(event.message || "Worker failed. Download the kit to run Python locally.", true);
      };
      armTimeout(120000, "Python could not start within two minutes. Check the connection, retry, or download the local kit.");
      worker.postMessage({ code: revision, clouds: dataset.scans.map((s) => s.points) });
    } catch (error) { finishWorker(); status("Python could not start. Try serving the page over HTTP(S)."); log(error.message, true); }
    render();
  }

  function saveCode() {
    dirty = results.length > 1 && $("code").value !== revision;
    try { localStorage.setItem(KEY, $("code").value); $("saveStatus").textContent = "Saved in this browser"; }
    catch (_) { $("saveStatus").textContent = "Browser storage unavailable · keep a local copy"; }
    render();
  }
  $("code").addEventListener("input", saveCode);
  $("code").addEventListener("keydown", (event) => {
    if ((event.ctrlKey || event.metaKey) && event.key === "Enter") { event.preventDefault(); run(); }
    if (event.key === "Tab" && !event.shiftKey && !$("code").readOnly) {
      event.preventDefault(); const el = event.target;
      el.setRangeText("    ", el.selectionStart, el.selectionEnd, "end"); saveCode();
    }
  });
  $("runBtn").onclick = run;
  $("stopBtn").onclick = () => { finishWorker(); status("Run stopped. Completed scans are kept; run again to restart from scan 01."); log("Stopped by you."); };
  $("starterBtn").onclick = () => {
    if (!confirm("Replace your current Python code with the starter? Downloaded files are unaffected.")) return;
    $("code").value = starter; saveCode();
  };
  $("solutionBtn").onclick = () => {
    if (worker) return;
    if ($("code").value !== starter && $("code").value !== sample &&
        !confirm("Replace your current code with the sample ICP solution? Copy any edits you want to keep first.")) return;
    $("code").value = sample; saveCode();
    status("Sample ICP solution loaded. Run it to calculate all transforms from the point clouds.");
    log("Loaded the NumPy point-to-point ICP sample. It estimates poses without using ground truth.");
  };
  $("frameRange").oninput = (e) => { haltPlayback(); selectFrame(Number(e.target.value)); };
  $("nextBtn").onclick = () => { haltPlayback(); selectFrame((frame + 1) % scanCount()); };
  $("playBtn").onclick = () => {
    if (playback) { haltPlayback(); return; }
    if (frame === scanCount() - 1) selectFrame(0);
    $("playBtn").textContent = "Ⅱ Pause";
    playback = setInterval(() => { if (frame >= scanCount() - 1) haltPlayback(); else selectFrame(frame + 1); }, 700);
  };
  $("expectedBtn").onclick = () => {
    haltPlayback(); mode = "expected";
    status("Expected preview uses the known synthetic poses. Your code and results are preserved."); render();
  };
  $("resultBtn").onclick = () => { haltPlayback(); mode = "result"; status(dirty ? "Showing the last run. Your edited code has not been run yet." : "Showing your computed registration results."); render(); };
  $("matrixType").onchange = render;
  $("projection").onchange = render;
  $("resetView").onclick = () => { Object.assign(camera, { yaw: -.52, pitch: .83, zoom: 1 }); render(); };
  $("exportBtn").onclick = () => download("transformations.json", JSON.stringify({ dataset: dataset.id, results }, null, 2));

  function validateMatrix(T) {
    if (!Array.isArray(T) || T.length !== 4 || T.some((r) => !Array.isArray(r) || r.length !== 4 || r.some((v) => typeof v !== "number" || !Number.isFinite(v)))) throw new Error("Each transformation must be a finite numeric 4 × 4 matrix.");
    if (T[3].some((v, j) => Math.abs(v - (j === 3 ? 1 : 0)) > 1e-6)) throw new Error("A matrix has an invalid bottom row.");
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) {
      const dot = [0, 1, 2].reduce((s, k) => s + T[k][i] * T[k][j], 0);
      if (Math.abs(dot - (i === j ? 1 : 0)) > 1e-4) throw new Error("A rotation is not orthonormal.");
    }
    const det = T[0][0] * (T[1][1] * T[2][2] - T[1][2] * T[2][1]) - T[0][1] * (T[1][0] * T[2][2] - T[1][2] * T[2][0]) + T[0][2] * (T[1][0] * T[2][1] - T[1][1] * T[2][0]);
    if (Math.abs(det - 1) > 1e-4) throw new Error("A rotation determinant is not +1.");
  }
  $("importFile").onchange = async (e) => {
    const file = e.target.files[0]; if (!file) return;
    try {
      if (file.size > 1000000) throw new Error("Choose a transformations.json file smaller than 1 MB.");
      const data = JSON.parse(await file.text());
      if (data.dataset !== dataset.id || !Array.isArray(data.results) || data.results.length !== scanCount()) throw new Error(`Expected ${scanCount()} results for ${dataset.id}.`);
      let pose = identity();
      const imported = data.results.map((r, i) => {
        if (r.index !== i) throw new Error(`Scan indices must be in order from 0 to ${scanCount() - 1}.`);
        validateMatrix(r.relative);
        if (!i && r.relative.some((row, j) => row.some((v, k) => Math.abs(v - (j === k ? 1 : 0)) > 1e-6))) throw new Error("The first scan must use an identity transformation.");
        pose = multiply(pose, r.relative);
        // Recompute poses instead of trusting a second, possibly inconsistent matrix.
        return { index: i, relative: r.relative, pose, rmse: null };
      });
      haltPlayback(); results = imported; revision = ""; dirty = false; mode = "result";
      $("resultBtn").disabled = false; $("exportBtn").disabled = false; selectFrame(scanCount() - 1);
      status(`Imported ${scanCount()} poses. Pairwise NN RMSE is unavailable; ground-truth pose errors are shown above.`);
      log(`Imported local Python results; map poses recomputed from the ${scanCount() - 1} relative transforms.`);
    } catch (error) { status("Import failed: " + error.message); log(error.message, true); }
    e.target.value = "";
  };

  for (const canvas of [$("inputCanvas"), $("mapCanvas")]) {
    let previous = null;
    canvas.addEventListener("pointerdown", (e) => { previous = [e.clientX, e.clientY]; canvas.setPointerCapture(e.pointerId); });
    canvas.addEventListener("pointermove", (e) => {
      if (!previous) return;
      camera.yaw += (e.clientX - previous[0]) * .008;
      camera.pitch = Math.min(1.45, Math.max(.15, camera.pitch + (e.clientY - previous[1]) * .006));
      previous = [e.clientX, e.clientY]; render();
    });
    canvas.addEventListener("pointerup", () => { previous = null; });
    canvas.addEventListener("pointercancel", () => { previous = null; });
    canvas.addEventListener("wheel", (e) => { e.preventDefault(); camera.zoom = Math.max(.4, Math.min(4, camera.zoom * Math.exp(-e.deltaY * .001))); render(); }, { passive: false });
    canvas.addEventListener("keydown", (e) => {
      if (!["ArrowLeft", "ArrowRight", "ArrowUp", "ArrowDown", "+", "=", "-"].includes(e.key)) return;
      e.preventDefault();
      if (e.key === "ArrowLeft") camera.yaw -= .1;
      if (e.key === "ArrowRight") camera.yaw += .1;
      if (e.key === "ArrowUp") camera.pitch = Math.min(1.45, camera.pitch + .1);
      if (e.key === "ArrowDown") camera.pitch = Math.max(.15, camera.pitch - .1);
      if (e.key === "+" || e.key === "=") camera.zoom = Math.min(4, camera.zoom * 1.1);
      if (e.key === "-") camera.zoom = Math.max(.4, camera.zoom / 1.1);
      render();
    });
  }
  new ResizeObserver(render).observe($("inputCanvas"));
  document.addEventListener("themechange", render);
  ICPComparison.init((index) => { haltPlayback(); selectFrame(index); });

  async function init() {
    try {
      const responses = await Promise.all(["sequence.json", "expected-poses.json", "registration.py", "sample_solution.py"].map(async (name) => {
        const res = await fetch("assets/icp/" + name);
        if (!res.ok) throw new Error(`Could not load ${name} (HTTP ${res.status}).`);
        return name.endsWith("json") ? res.json() : res.text();
      }));
      [dataset, expectedPoses, starter, sample] = responses;
      // The editor contains only the function/imports; the ZIP retains the local CLI.
      starter = starter.split('if __name__ == "__main__":')[0].trimEnd() + "\n";
      try { $("code").value = localStorage.getItem(KEY) ?? starter; }
      catch (_) { $("code").value = starter; }
      sample = sample.split('if __name__ == "__main__":')[0].trimEnd() + "\n";
      $("frameRange").max = scanCount() - 1;
      for (let i = 0; i < scanCount(); i++) {
        const button = document.createElement("button"); button.textContent = pad(i);
        button.setAttribute("aria-label", `Inspect scan ${pad(i)}`);
        button.onclick = () => { haltPlayback(); selectFrame(i); };
        $("timeline").appendChild(button);
      }
      ["runBtn", "starterBtn", "solutionBtn", "expectedBtn", "frameRange", "playBtn", "nextBtn", "code", "importFile"].forEach((id) => { $(id).disabled = false; });
      $("console").textContent = "Ready. The starter returns identity matrices. Implement ICP, then run your registration.";
      status("Inspect the scans or run the identity baseline. Scan 01 is already at the map origin."); render();
    } catch (error) {
      status("Workshop files could not load. Serve this folder over HTTP(S) and reload.");
      log(error.message + "\nLocal preview: python3 -m http.server 8000", true);
    }
  }
  init();
})();
