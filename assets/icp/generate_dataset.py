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
SCAN_COUNT = 100
ROOM_SIZE = (24, 16, 3.2)
MAX_RANGE = 12.0
BOXES = [
    ((-.1, -.1, 0), (24.1, 0, 3.2)),  # outer walls
    ((-.1, 16, 0), (24.1, 16.1, 3.2)),
    ((-.1, 0, 0), (0, 16, 3.2)),
    ((24, 0, 0), (24.1, 16, 3.2)),
    ((0, 0, -.1), (24, 16, 0)),  # floor and ceiling
    ((0, 0, 3.2), (24, 16, 3.3)),
    ((8, 0, 0), (8.2, 5.0, 3.2)),  # west partition with a wide doorway
    ((8, 9.5, 0), (8.2, 16, 3.2)),
    ((16, 0, 0), (16.2, 6.8, 3.2)),  # east partition
    ((16, 10.5, 0), (16.2, 16, 3.2)),
    ((1.2, 2.5, 0), (4.8, 3.8, .85)),  # workbenches and cabinets
    ((2, 10, 0), (5.5, 11.4, 1.1)),
    ((.2, 13, 0), (1.2, 15, 2.2)),
    ((10, 2.5, 0), (13.8, 4.5, .9)),
    ((11.5, 11, 0), (14.8, 12.3, 1.6)),
    ((18.2, 11, 0), (21.6, 12.5, 1.1)),
    ((21.5, 3, 0), (23.2, 5.5, 2.0)),
    ((5.5, 7.8, 0), (6.1, 8.4, 3.2)),  # pillars away from the robot path
    ((10.2, 9.5, 0), (10.8, 10.1, 3.2)),
    ((19.3, 4.5, 0), (19.9, 5.1, 3.2)),
]


def intersection(origin, direction, low, high):
    near, far = 0.0, MAX_RANGE
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
        x, y = 3 + i * .16, 6 + 1.2 * (1 - math.cos(i * .025))
        yaw = math.atan2(1.2 * .025 * math.sin(i * .025), .16)
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
        poses.append([[c, -s, 0, round(x - 3, 8)], [s, c, 0, round(y - 6, 8)],
                      [0, 0, 1, 0], [0, 0, 0, 1]])
    data = {"id": "vmc-indoor-100-v1", "units": "metres", "axes": "x forward, y left, z up",
            "room_size": ROOM_SIZE, "sensor_range": MAX_RANGE,
            "map_bounds": {"min": [-3, -6, -.65], "max": [21, 10, 2.55]},
            "description": "Synthetic 24 x 16 m indoor lab: three connected bays, benches, cabinets and pillars. "
                           "360 degree LiDAR, four elevation rings, 12 m range, 6 mm range noise.",
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
                         "sequence.json and scans/*.csv contain the same 100 ordered scans.\n"
                         "Environment: 24 x 16 m indoor lab with three connected bays.\n"
                         "Coordinates: metres; x forward, y left, z up, in each sensor frame.\n"
                         "Return a 4x4 rigid transform from current scan to previous scan.\n"
                         "Map frame = scan 00; T_map_current = T_map_previous @ T_previous_current.\n"
                         "The identity baseline runs, but does not correct for robot motion.\n")
    print("Generated", len(scans), "scans:", [len(s["points"]) for s in scans])


if __name__ == "__main__":
    generate()
