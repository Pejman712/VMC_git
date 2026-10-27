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


class WorkshopTest(unittest.TestCase):
    def test_dataset_and_local_kit(self):
        data = json.loads((ASSETS / "sequence.json").read_text())
        self.assertEqual(len(data["scans"]), 10)
        for i, scan in enumerate(data["scans"]):
            points = np.asarray(scan["points"])
            self.assertEqual(scan["name"], f"scan_{i:02d}.csv")
            self.assertEqual(points.shape[1], 3)
            self.assertGreater(len(points), 800)
            self.assertTrue(np.isfinite(points).all())
            self.assertLess(np.linalg.norm(points, axis=1).max(), 9.05)
        with zipfile.ZipFile(ASSETS / "icp-workshop.zip") as archive:
            self.assertEqual(len([n for n in archive.namelist() if n.endswith(".csv")]), 10)
            for name in ["registration.py", "runner.py", "sequence.json"]:
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
        self.assertEqual(calls, list(range(1, 10)))
        self.assertEqual(len(reported), 10)
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
        for i in range(1, 10):
            T = np.linalg.inv(poses[i - 1]) @ poses[i]
            aligned = clouds[i] @ T[:3, :3].T + T[:3, 3]
            self.assertLess(runner.nearest_rmse(aligned, clouds[i - 1]), runner.nearest_rmse(clouds[i], clouds[i - 1]))


if __name__ == "__main__":
    unittest.main()
