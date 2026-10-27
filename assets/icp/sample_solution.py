"""Sample point-to-point ICP using only NumPy; no known poses or correspondences."""
import numpy as np


def registration(source, target):
    """Return T_previous_current: source (current scan) -> target (previous scan).

    This is a teaching example for small motions and overlapping scans. ICP is
    a local optimizer: nearest-neighbour RMSE and map drift can remain nonzero.
    """
    source = np.asarray(source, dtype=float)
    target = np.asarray(target, dtype=float)
    for points in (source, target):
        if points.ndim != 2 or points.shape[1] != 3 or len(points) < 3:
            raise ValueError("Each cloud must have shape (N, 3), with N >= 3.")
        if not np.isfinite(points).all():
            raise ValueError("Point coordinates must be finite.")

    T = np.eye(4)
    previous_error = float("inf")
    for iteration in range(60):
        moved = source @ T[:3, :3].T + T[:3, 3]

        # 1. Find nearest neighbours in small batches to limit memory usage.
        indices = np.empty(len(source), dtype=int)
        squared_errors = np.empty(len(source))
        for start in range(0, len(source), 128):
            batch = moved[start:start + 128]
            distances = ((batch[:, None, :] - target[None, :, :]) ** 2).sum(axis=2)
            nearest = distances.argmin(axis=1)
            indices[start:start + len(batch)] = nearest
            squared_errors[start:start + len(batch)] = distances[np.arange(len(batch)), nearest]

        # 2. Reject distant pairs; point indices alone are not correspondences.
        keep = squared_errors < 0.5 ** 2
        if keep.sum() < 3:
            raise ValueError("Too few matches within 0.5 m. ICP needs overlapping scans.")
        a, b = moved[keep], target[indices[keep]]
        centroid_a, centroid_b = a.mean(axis=0), b.mean(axis=0)

        # 3. Solve the least-squares rigid alignment of the matched points.
        H = (a - centroid_a).T @ (b - centroid_b)
        U, singular_values, Vt = np.linalg.svd(H)
        if singular_values[1] < 1e-10:
            raise ValueError("Matches are collinear or coincident; rotation is ambiguous.")
        if np.linalg.det(Vt.T @ U.T) < 0:
            Vt[-1] *= -1  # Avoid a reflection: a rotation has determinant +1.
        R = Vt.T @ U.T
        t = centroid_b - R @ centroid_a

        # 4. Compose the incremental correction with the existing estimate.
        delta = np.eye(4)
        delta[:3, :3], delta[:3, 3] = R, t
        T = delta @ T

        error = squared_errors[keep].mean()
        if abs(previous_error - error) < 1e-9:
            break
        previous_error = error
    return T


if __name__ == "__main__":
    import json
    from pathlib import Path
    from runner import run_sequence

    root = Path(__file__).resolve().parent
    data = json.loads((root / "sequence.json").read_text())
    clouds = [np.asarray(scan["points"], dtype=float) for scan in data["scans"]]
    results = run_sequence(clouds, registration)
    output = root / "transformations.json"
    output.write_text(json.dumps({"dataset": data["id"], "results": results}, indent=2))
    print(f"Saved {len(results)} poses to {output}. Import this file in the workshop.")
