"""Rebuild the deterministic synthetic indoor scan sequence and student ZIP.

Run from any directory: python3 assets/icp/generate_dataset.py
Only the Python standard library is needed. Measurements use ray/box intersections
with occlusion, limited range, per-scan dropout and small independent range noise.
"""
import csv
import bisect
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
ROUTE_WAYPOINTS = [(3, 6), (4.5, 6), (6, 4.9), (9, 6.6), (11.8, 8.6),
                   (15, 8.2), (18, 8.2), (20, 9.5), (20, 10)]
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


def robot_route():
    """Sample a smooth, obstacle-free route with real left/right yaw changes.

    Component-wise monotone Hermite tangents avoid overshooting the waypoints.
    Sample density increases with heading change so turns don't jump between scans.
    The robot always faces the path tangent; it does not slide sideways.
    """
    tangents = []
    for i, point in enumerate(ROUTE_WAYPOINTS):
        if i == 0:
            tangent = [ROUTE_WAYPOINTS[1][j] - point[j] for j in range(2)]
        elif i == len(ROUTE_WAYPOINTS) - 1:
            tangent = [point[j] - ROUTE_WAYPOINTS[i - 1][j] for j in range(2)]
        else:
            tangent = []
            for j in range(2):
                before = point[j] - ROUTE_WAYPOINTS[i - 1][j]
                after = ROUTE_WAYPOINTS[i + 1][j] - point[j]
                tangent.append(2 * before * after / (before + after) if before * after > 0 else 0)
        tangents.append(tangent)

    dense = []
    for i in range(len(ROUTE_WAYPOINTS) - 1):
        p, q = ROUTE_WAYPOINTS[i:i + 2]
        a, b = tangents[i:i + 2]
        for step in range(300):
            t = step / 300
            xy = [(2*t**3 - 3*t*t + 1)*p[j] + (t**3 - 2*t*t + t)*a[j]
                  + (-2*t**3 + 3*t*t)*q[j] + (t**3 - t*t)*b[j] for j in range(2)]
            velocity = [(6*t*t - 6*t)*p[j] + (3*t*t - 4*t + 1)*a[j]
                        + (-6*t*t + 6*t)*q[j] + (3*t*t - 2*t)*b[j] for j in range(2)]
            yaw = math.atan2(velocity[1], velocity[0])
            if dense:
                yaw = dense[-1][2] + math.atan2(math.sin(yaw - dense[-1][2]), math.cos(yaw - dense[-1][2]))
            dense.append((*xy, yaw))
    dense.append((*ROUTE_WAYPOINTS[-1], math.atan2(tangents[-1][1], tangents[-1][0])))

    progress = [0.0]
    for previous, current in zip(dense, dense[1:]):
        distance = math.hypot(current[0] - previous[0], current[1] - previous[1])
        turn = abs(current[2] - previous[2])
        progress.append(progress[-1] + max(distance, 4 * turn))
    route = []
    for i in range(SCAN_COUNT):
        value = progress[-1] * i / (SCAN_COUNT - 1)
        right = min(len(progress) - 1, max(1, bisect.bisect_left(progress, value)))
        fraction = (value - progress[right - 1]) / (progress[right] - progress[right - 1])
        route.append(tuple(dense[right - 1][j] + fraction * (dense[right][j] - dense[right - 1][j]) for j in range(3)))
    return route


def generate():
    rng = random.Random(20261027)
    scans, poses = [], []
    route = robot_route()
    for i, (x, y, yaw) in enumerate(route):
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
    data = {"id": "vmc-indoor-100-v2", "units": "metres", "axes": "x forward, y left, z up",
            "room_size": ROOM_SIZE, "sensor_range": MAX_RANGE,
            "map_bounds": {"min": [-3, -6, -.65], "max": [21, 10, 2.55]},
            "route": {"description": "S-shaped route with left and right turns, finishing facing north.",
                      "length_m": round(sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(route, route[1:])), 3),
                      "heading_range_deg": [round(math.degrees(min(p[2] for p in route)), 2), round(math.degrees(max(p[2] for p in route)), 2)]},
            "description": "Synthetic 24 x 16 m indoor lab: three connected bays, benches, cabinets and pillars. "
                           "S-shaped route with left/right turns; 360 degree LiDAR, four elevation rings, 12 m range, 6 mm range noise.",
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
                         "Route: about 20 m with left/right bends and a final north-facing pose.\n"
                         "Coordinates: metres; x forward, y left, z up, in each sensor frame.\n"
                         "Return a 4x4 rigid transform from current scan to previous scan.\n"
                         "Map frame = scan 00; T_map_current = T_map_previous @ T_previous_current.\n"
                         "The identity baseline runs, but does not correct for robot motion.\n")
    print("Generated", len(scans), "scans:", [len(s["points"]) for s in scans])


if __name__ == "__main__":
    generate()
