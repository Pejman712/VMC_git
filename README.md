# ROS Ecosystem Explorer

An interactive teaching companion for the **Vehicle Mechatronics** ROS lecture.
A single self-contained `index.html` — no build step, no dependencies.

## What's inside
- **ROS at a glance** — headline ecosystem stats (2025).
- **Ecosystem / demographics** — distro timeline, download share, the ROS 1 → ROS 2 migration, global reach, community.
- **Sensors & hardware** — representative ROS 2 drivers grouped by the standard message they publish.
- **Interactive Pub/Sub** — publish messages, change the rate, add/remove listeners, watch callbacks fire.
- **Tools** — `rqt_graph`, `RViz`, and `ros2 bag`, all wired to the live demo state.

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
