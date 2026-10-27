"""ICP workshop. Complete registration(), then run this file or paste it into the page."""
import numpy as np


def registration(source, target):
    """Align the current scan (source) to the previous scan (target).

    Inputs: independent NumPy arrays of shape (N, 3) and (M, 3), in metres.
    Return: a rigid 4 x 4 matrix T_previous_current, such that
        aligned_source = source @ T[:3, :3].T + T[:3, 3]
    Point indices are NOT correspondences. Start with an identity estimate.
    """
    T = np.eye(4)

    # TODO: repeat until convergence (e.g. at most 40 iterations):
    # 1. Transform source points with the current estimate T.
    # 2. Find each transformed point's nearest neighbour in target.
    # 3. Reject distant matches (try a 0.5 m threshold).
    # 4. Centre the matched points and estimate a rigid rotation with SVD.
    #    Correct reflections: the rotation determinant must be +1.
    # 5. Estimate translation; form an incremental 4 x 4 matrix delta.
    # 6. Compose T = delta @ T and check the change in alignment error.

    return T  # Identity is a runnable baseline, not a completed solution.


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
