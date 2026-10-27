"""Shared browser/local workshop runner. The first scan defines the map frame."""
import numpy as np


def validate_transform(value):
    T = np.asarray(value, dtype=float)
    if T.shape != (4, 4):
        raise ValueError(f"registration must return a (4, 4) matrix; received {T.shape}.")
    if not np.isfinite(T).all():
        raise ValueError("The transformation contains NaN or infinity.")
    if not np.allclose(T[3], [0, 0, 0, 1], atol=1e-6, rtol=0):
        raise ValueError("The last row must be [0, 0, 0, 1].")
    R = T[:3, :3]
    if not np.allclose(R.T @ R, np.eye(3), atol=1e-4, rtol=0):
        raise ValueError("The rotation must be orthonormal; scaling/shearing is not a rigid pose.")
    if not np.isclose(np.linalg.det(R), 1, atol=1e-4, rtol=0):
        raise ValueError("The rotation determinant must be +1 (no reflection).")
    return T


def nearest_rmse(source, target):
    """One-way nearest-neighbour RMSE, in metres; not ground-truth pose error."""
    squared = []
    for start in range(0, len(source), 128):
        distances = ((source[start:start + 128, None] - target[None]) ** 2).sum(axis=2)
        squared.extend(distances.min(axis=1))
    return float(np.sqrt(np.mean(squared)))


def run_sequence(clouds, register, on_frame=None):
    pose = np.eye(4)
    results = [{"index": 0, "relative": pose.tolist(), "pose": pose.tolist(), "rmse": None}]
    if on_frame:
        on_frame(results[0])
    for index in range(1, len(clouds)):
        # Copies prevent student code from changing subsequent input scans.
        source = np.array(clouds[index], dtype=float, copy=True)
        target = np.array(clouds[index - 1], dtype=float, copy=True)
        try:
            relative = validate_transform(register(source.copy(), target.copy()))
        except Exception as error:
            raise ValueError(f"Scan {index + 1:02d} → {index:02d}: {error}") from error
        pose = pose @ relative
        aligned = source @ relative[:3, :3].T + relative[:3, 3]
        result = {"index": index, "relative": relative.tolist(), "pose": pose.tolist(),
                  "rmse": nearest_rmse(aligned, target)}
        results.append(result)
        if on_frame:
            on_frame(result)
    return results
