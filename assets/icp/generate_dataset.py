"""Rebuild the deterministic synthetic indoor scan sequence and student ZIP.

Run from any directory: python3 assets/icp/generate_dataset.py
Only the Python standard library is needed. Measurements use ray/box intersections
with occlusion, limited range, per-scan dropout and small independent range noise.
"""
import csv
import io
import json
import math
from pathlib import Path
import random
import zipfile

ROOT = Path(__file__).resolve().parent
SCAN_COUNT = 50
BOXES = [
    ((-.1, -.1, 0), (12.1, 0, 2.8)),  # outer walls
    ((-.1, 8, 0), (12.1, 8.1, 2.8)),
    ((-.1, 0, 0), (0, 8, 2.8)),
    ((12, 0, 0), (12.1, 8, 2.8)),
    ((0, 0, -.1), (12, 8, 0)),  # floor
    ((0, 0, 2.8), (12, 8, 2.9)),  # ceiling
    ((6.8, 0, 0), (7, 3.3, 2.8)),  # partition, with an opening above y=3.3
    ((1.1, 5.5, 0), (3.7, 6.6, .85)),  # workbench
    ((8.3, 5.4, 0), (10.6, 6.7, 1.1)),
    ((.2, .6, 0), (1.0, 2.0, 1.8)),  # cabinet
    ((5.4, 6.2, 0), (5.85, 6.65, 2.8)),  # pillar
]


def intersection(origin, direction, low, high):
    near, far = 0.0, 9.0
    for o, d, lo, hi in zip(origin, direction, low, high):
        if abs(d) < 1e-10:
            if o < lo or o > hi:
                return None
        else:
            a, b = sorted(((lo - o) / d, (hi - o) / d))
            near, far = max(near, a), min(far, b)
            if near > far:
                return None
    return near if near > .05 else None


def generate():
    rng = random.Random(20261027)
    scans, poses = [], []
    for i in range(SCAN_COUNT):
        # Follow a gentle curve through the opening, clear of walls/furniture.
        x, y = 3 + i * .12, 3.4 + .55 * (1 - math.cos(i * .05))
        yaw = math.atan2(.55 * .05 * math.sin(i * .05), .12)
        c, s = math.cos(yaw), math.sin(yaw)
        points = []
        # Avoid a floor-dominated ring whose near-identical returns at every pose
        # overwhelm useful wall/furniture correspondences in introductory ICP.
        for elevation in [-.06, .08, .2, .32]:
            for ray in range(240):
                angle = 2 * math.pi * ray / 240 + i * .0007
                local = (math.cos(angle) * math.cos(elevation),
                         math.sin(angle) * math.cos(elevation), math.sin(elevation))
                direction = (c * local[0] - s * local[1], s * local[0] + c * local[1], local[2])
                hits = [t for lo, hi in BOXES
                        if (t := intersection((x, y, .65), direction, lo, hi)) is not None]
                if hits and rng.random() > .035:
                    distance = min(hits) + rng.gauss(0, .006)
                    points.append([round(distance * v, 5) for v in local])
        rng.shuffle(points)
        scans.append({"name": f"scan_{i:02d}.csv", "points": points})
        # The first sensor frame is the world/map frame, including its height.
        poses.append([[c, -s, 0, round(x - 3, 8)], [s, c, 0, round(y - 3.4, 8)],
                      [0, 0, 1, 0], [0, 0, 0, 1]])
    data = {"id": "vmc-indoor-50-v1", "units": "metres", "axes": "x forward, y left, z up",
            "description": "Synthetic raycast office: walls, partition, benches, cabinet and pillar. "
                           "360 degree LiDAR, four elevation rings, 9 m range, 6 mm range noise.",
            "scans": scans}
    (ROOT / "sequence.json").write_text(json.dumps(data, separators=(",", ":")) + "\n")
    (ROOT / "expected-poses.json").write_text(json.dumps(poses, separators=(",", ":")) + "\n")
    with zipfile.ZipFile(ROOT / "icp-workshop.zip", "w", zipfile.ZIP_DEFLATED) as archive:
        for name in ["sequence.json", "registration.py", "sample_solution.py", "runner.py"]:
            archive.write(ROOT / name, name)
        for scan in scans:
            buffer = io.StringIO()
            writer = csv.writer(buffer)
            writer.writerow(["x", "y", "z"])
            writer.writerows(scan["points"])
            archive.writestr("scans/" + scan["name"], buffer.getvalue())
        archive.writestr("README.txt", "ICP Workshop\n\nInstall Python 3 and NumPy (python -m pip install numpy).\n"
                         "Complete registration(source, target) in registration.py.\n"
                         "Run: python registration.py\n"
                         "Or run the included NumPy ICP sample: python sample_solution.py\n"
                         "Import transformations.json into the ICP Workshop page.\n\n"
                         "sequence.json and scans/*.csv contain the same 50 ordered scans.\n"
                         "Coordinates: metres; x forward, y left, z up, in each sensor frame.\n"
                         "Return a 4x4 rigid transform from current scan to previous scan.\n"
                         "Map frame = scan 00; T_map_current = T_map_previous @ T_previous_current.\n"
                         "The identity baseline runs, but does not correct for robot motion.\n")
    print("Generated", len(scans), "scans:", [len(s["points"]) for s in scans])


if __name__ == "__main__":
    generate()
