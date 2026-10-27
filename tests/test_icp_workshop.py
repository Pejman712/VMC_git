"""Run with: python3 -m unittest discover -s tests -v"""
import importlib.util
import json
from pathlib import Path
import unittest
import zipfile

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "assets" / "icp"
spec = importlib.util.spec_from_file_location("runner", ASSETS / "runner.py")
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)
sample_spec = importlib.util.spec_from_file_location("sample_solution", ASSETS / "sample_solution.py")
sample = importlib.util.module_from_spec(sample_spec)
sample_spec.loader.exec_module(sample)
generator_spec = importlib.util.spec_from_file_location("generator", ASSETS / "generate_dataset.py")
generator = importlib.util.module_from_spec(generator_spec)
generator_spec.loader.exec_module(generator)


class WorkshopTest(unittest.TestCase):
    def test_route_turns_smoothly_and_clears_obstacles(self):
        route = np.asarray(generator.robot_route())
        self.assertEqual(route.shape, (100, 3))
        heading = np.unwrap(route[:, 2])
        self.assertLess(np.degrees(heading.min()), -45)
        self.assertGreater(np.degrees(heading.max()), 85)
        self.assertAlmostEqual(heading[0], 0)
        self.assertAlmostEqual(heading[-1], np.pi / 2)
        changes = np.degrees(np.diff(heading))
        self.assertLess(changes.min(), -2)
        self.assertGreater(changes.max(), 2)
        self.assertLess(np.abs(changes).max(), 5)
        steps = np.linalg.norm(np.diff(route[:, :2], axis=0), axis=1)
        self.assertLess(steps.max(), .32)
        self.assertGreater(steps.sum(), 19)
        # Check the robot footprint between scans too, not only at their centres.
        for start, end in zip(route, route[1:]):
            for t in np.linspace(0, 1, 10):
                x, y = (start + t * (end - start))[:2]
                for low, high in generator.BOXES:
                    if not low[2] < .65 < high[2]:
                        continue  # Floor/ceiling do not intersect the robot body.
                    dx = max(low[0] - x, 0, x - high[0])
                    dy = max(low[1] - y, 0, y - high[1])
                    self.assertGreater(np.hypot(dx, dy), .55)
        truth = np.asarray(json.loads((ASSETS / "expected-poses.json").read_text()))
        np.testing.assert_allclose(truth[:, :2, 3], route[:, :2] - [3, 6], atol=1e-8)
        np.testing.assert_allclose(np.arctan2(truth[:, 1, 0], truth[:, 0, 0]), heading, atol=1e-8)

    def test_dataset_and_local_kit(self):
        data = json.loads((ASSETS / "sequence.json").read_text())
        self.assertEqual(len(data["scans"]), 100)
        self.assertEqual(data["room_size"], [24, 16, 3.2])
        self.assertEqual(data["id"], "vmc-indoor-100-v2")
        for i, scan in enumerate(data["scans"]):
            points = np.asarray(scan["points"])
            self.assertEqual(scan["name"], f"scan_{i:02d}.csv")
            self.assertEqual(points.shape[1], 3)
            self.assertGreater(len(points), 800)
            self.assertTrue(np.isfinite(points).all())
            self.assertLess(np.linalg.norm(points, axis=1).max(), 12.05)
        with zipfile.ZipFile(ASSETS / "icp-workshop.zip") as archive:
            self.assertEqual(len([n for n in archive.namelist() if n.endswith(".csv")]), 100)
            for name in ["registration.py", "sample_solution.py", "runner.py", "sequence.json"]:
                self.assertEqual(archive.read(name), (ASSETS / name).read_bytes())

    def test_composition_direction_and_input_isolation(self):
        poses = [np.asarray(p) for p in json.loads((ASSETS / "expected-poses.json").read_text())]
        world = np.array([[0, 0, 0], [1, 0, 0], [0, 2, 0], [0, 0, 3], [1, 2, 3]], dtype=float)
        clouds = [(world - p[:3, 3]) @ p[:3, :3] for p in poses]
        saved = [p.copy() for p in clouds]
        calls, reported = [], []

        def register(source, target):
            i = len(calls) + 1
            np.testing.assert_allclose(source, clouds[i])
            np.testing.assert_allclose(target, clouds[i - 1])
            calls.append(i)
            source[:] = 999  # Should not corrupt data or error measurements.
            target[:] = 999
            return np.linalg.inv(poses[i - 1]) @ poses[i]

        results = runner.run_sequence(clouds, register, reported.append)
        self.assertEqual(calls, list(range(1, 100)))
        self.assertEqual(len(reported), 100)
        for i, result in enumerate(results):
            np.testing.assert_allclose(result["pose"], poses[i], atol=1e-9)
            np.testing.assert_array_equal(clouds[i], saved[i])
            if i:
                self.assertLess(result["rmse"], 1e-9)

    def test_invalid_returns(self):
        nan = np.eye(4); nan[0, 3] = np.nan
        shear = np.eye(4); shear[0, 1] = .1
        reflection = np.eye(4); reflection[0, 0] = -1
        bottom = np.eye(4); bottom[3, 0] = 1
        for matrix in [None, np.eye(3), nan, shear, reflection, bottom]:
            with self.subTest(matrix=matrix):
                with self.assertRaises((ValueError, TypeError)):
                    runner.validate_transform(matrix)

    def test_error_identifies_scan_pair(self):
        with self.assertRaisesRegex(ValueError, "Scan 02 → 01"):
            runner.run_sequence([np.zeros((4, 3))] * 2, lambda a, b: np.eye(3))

    def test_expected_alignment_improves_on_identity(self):
        clouds = [np.asarray(s["points"]) for s in json.loads((ASSETS / "sequence.json").read_text())["scans"]]
        poses = [np.asarray(p) for p in json.loads((ASSETS / "expected-poses.json").read_text())]
        for i in range(1, 100):
            T = np.linalg.inv(poses[i - 1]) @ poses[i]
            aligned = clouds[i] @ T[:3, :3].T + T[:3, 3]
            self.assertLess(runner.nearest_rmse(aligned, clouds[i - 1]), runner.nearest_rmse(clouds[i], clouds[i - 1]))

    def test_sample_recovers_known_rigid_transform(self):
        target = np.random.default_rng(42).uniform(-2, 2, (80, 3))
        angle = .025
        T = np.eye(4)
        T[:3, :3] = [[np.cos(angle), -np.sin(angle), 0],
                     [np.sin(angle), np.cos(angle), 0], [0, 0, 1]]
        T[:3, 3] = [.04, -.03, .02]
        source = (target - T[:3, 3]) @ T[:3, :3]
        np.random.default_rng(7).shuffle(source)
        np.testing.assert_allclose(sample.registration(source, target), T, atol=1e-6)

    def test_sample_maps_all_100_scans(self):
        data = json.loads((ASSETS / "sequence.json").read_text())
        clouds = [np.asarray(s["points"]) for s in data["scans"]]
        expected = np.asarray(json.loads((ASSETS / "expected-poses.json").read_text()))
        results = runner.run_sequence(clouds, sample.registration)
        self.assertEqual(len(results), 100)
        poses = np.asarray([r["pose"] for r in results])
        for result in results[1:]:
            i = result["index"]
            self.assertLess(result["rmse"], runner.nearest_rmse(clouds[i], clouds[i - 1]))
        # Pairwise ICP accumulates drift. It should improve on a stationary robot.
        errors = np.linalg.norm(poses[:, :3, 3] - expected[:, :3, 3], axis=1)
        baseline_errors = np.linalg.norm(expected[:, :3, 3], axis=1)
        self.assertLess(np.sqrt(np.mean(errors ** 2)), np.sqrt(np.mean(baseline_errors ** 2)) / 3)
        estimated_yaw = np.arctan2(poses[:, 1, 0], poses[:, 0, 0])
        true_yaw = np.arctan2(expected[:, 1, 0], expected[:, 0, 0])
        yaw_error = np.arctan2(np.sin(estimated_yaw - true_yaw), np.cos(estimated_yaw - true_yaw))
        self.assertLess(np.degrees(np.abs(yaw_error)).max(), 10)


if __name__ == "__main__":
    unittest.main()
