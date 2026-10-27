# ROS Ecosystem Explorer

An interactive teaching companion for the **Vehicle Mechatronics** ROS lecture.
A static collection of teaching pages — no build step.

## What's inside
- **ROS at a glance** — headline ecosystem stats (2025).
- **Ecosystem / demographics** — distro timeline, download share, the ROS 1 → ROS 2 migration, global reach, community.
- **Sensors & hardware** — representative ROS 2 drivers grouped by the standard message they publish.
- **Interactive Pub/Sub** — publish messages, change the rate, add/remove listeners, watch callbacks fire.
- **Tools** — `rqt_graph`, `RViz`, and `ros2 bag`, all wired to the live demo state.
- **[ICP Workshop](icp-workshop.html)** — 100 consecutive synthetic indoor LiDAR
  scans, an editable Python `registration(source, target)` exercise, and linked 3D
  input/map views with a robot trajectory. Includes scan playback, expected-pose
  comparison, rigid-matrix validation, and a downloadable local Python kit.

## ICP Workshop

Serve the repository over HTTP (opening the file directly cannot load the dataset
or Python worker):

```bash
python3 -m http.server 8000
```

Open `http://localhost:8000/icp-workshop.html`. The first Python run downloads
[Pyodide](https://pyodide.org/en/stable/usage/index.html) and NumPy from jsDelivr;
the rest of the workshop assets are local. Student Python runs in a Web Worker
with Stop and a 30-second limit per registration call. Edits persist in browser
local storage. The initial identity implementation deliberately leaves ICP for
the student to complete. Use **Load sample solution** for a complete NumPy
point-to-point ICP example, then **Run my registration** to calculate its map.
The sample uses nearest-neighbour matching, outlier rejection, SVD, and convergence
checking; it does not use the known synthetic poses.

`registration(source, target)` returns a 4×4 transform from the **current scan
to the previous scan**. The runner calls it 99 times for scans 02–100, uses scan
01 as the map frame, and composes `T_map_current = T_map_previous @ T`.
The 24 × 16 m lab has three connected bays, with a roughly 16 m path, 100 scans,
12 m sensor range, and 6 mm measurement noise. The expected-map preview uses known
synthetic poses, independently of student results.

The **Estimated trajectory vs ground truth** section plots both XY paths and 3D
position error by scan. Click a point to select its scan. Metrics include selected
position/rotation error, position RMSE over computed poses (including the origin),
and the latest/final position error. Poses are compared directly in scan 01's frame;
there is no fitted alignment. Previewing the expected map does not replace computed
results in these plots. Partial runs and imported results are supported. NN RMSE
measures scan alignment and is different from ground-truth pose error.

The **Implementation guide** explains the function contract, nearest-neighbour
matching, distance rejection, SVD and reflection correction, matrix composition,
convergence, mapping, and common mistakes, with NumPy snippets matching the sample.

For local Python, download `assets/icp/icp-workshop.zip`, unzip it, install NumPy,
and edit/run `registration.py`. Import its `transformations.json` into the page.
The kit contains the same 100 scans as both JSON and individual XYZ CSV files.
The ZIP also includes `sample_solution.py`; run it with `python sample_solution.py`
to generate `transformations.json` using the worked example.
The renderer recomputes map poses from imported relative transforms.

Dataset generation and verification (tests require NumPy):

```bash
python3 assets/icp/generate_dataset.py
python3 -m unittest discover -s tests -v
```

To check the plot calculations in the workshop browser console, run
`eval(await (await fetch('tests/icp_comparison.test.js')).text())`.
These tests cover empty/partial runs, 3D translation error, rotation error,
RMSE normalization, and numerical clamping.

Regenerate the kit after editing `registration.py`, `sample_solution.py`, `runner.py`, or the generator.
The dataset uses ray/box intersections in a synthetic room, with occlusion, range
limits, shuffled point ordering, dropout, and independent range noise. Ground-truth
poses are kept separately for the comparison preview and excluded from the kit.

## Deploy to GitHub Pages
1. Create a repo (e.g. `ros-ecosystem-explorer`) and add these files at the root.
   ```bash
   git init
   git add index.html README.md
   git commit -m "ROS Ecosystem Explorer"
   git branch -M main
   git remote add origin https://github.com/<you>/ros-ecosystem-explorer.git
   git push -u origin main
   ```
2. On GitHub: **Settings → Pages → Build and deployment**, set **Source = Deploy from a branch**, **Branch = `main` / `(root)`**, Save.
3. The site goes live at `https://<you>.github.io/ros-ecosystem-explorer/` within a minute or two.

> Prefer a `/docs` folder or a `gh-pages` branch? Either works — just point Pages at it. The page is fully static, so nothing else is required.

## Editing the data
All figures live in the `<script>` block near the top (`DISTROS`, `DISTRO_SHARE`,
`COUNTRIES`, `SENSORS`). Update the arrays and the charts/timeline redraw automatically.

## Data sources
- [2025 ROS Metrics Report](https://discourse.openrobotics.org/t/2025-ros-metrics-report/52575) (Open Robotics)
- [index.ros.org / stats](https://index.ros.org/stats/)
- [metrics.ros.org](https://metrics.ros.org/)

Figures are Oct 2025 / Jan 2026 snapshots. Educational, non-official resource.
