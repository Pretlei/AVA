"""
transform.py
─────────────────────────────────────────────────────────────────────────────
Pixel → real-world (mm) transform for use in your robot pipeline.

Requires calibration.json produced by calibrate.py.

Typical usage:
    from transform import pixel_to_world, pixel_to_world_batch

    x_mm, y_mm = pixel_to_world(u, v)          # single point
    coords      = pixel_to_world_batch(pixels)  # numpy array (N,2)
"""

import json
import numpy as np

_CALIBRATION_PATH = "calibration.json"

# ─────────────────────────────────────────────────────────────────────────────
# Load H once at import time – zero overhead in the hot loop
# ─────────────────────────────────────────────────────────────────────────────

def _load_H(path: str = _CALIBRATION_PATH) -> np.ndarray:
    try:
        with open(path) as f:
            data = json.load(f)
        H = np.array(data["H"], dtype=np.float64)
        assert H.shape == (3, 3), "Unexpected matrix shape in calibration file."
        return H
    except FileNotFoundError:
        raise FileNotFoundError(
            f"calibration.json not found at '{path}'.\n"
            "Run  python calibrate.py  first."
        )

H: np.ndarray = _load_H()


# ─────────────────────────────────────────────────────────────────────────────
# Public API
# ─────────────────────────────────────────────────────────────────────────────

def pixel_to_world(u: float, v: float) -> tuple[float, float]:
    """
    Map one image pixel to real-world table coordinates.

    Parameters
    ──────────
    u : pixel column  (horizontal, left = 0)
    v : pixel row     (vertical,   top  = 0)

    Returns
    ───────
    (x_mm, y_mm)  –  position on the table surface in millimetres,
                     in the coordinate frame you defined during calibration.
    """
    p = np.array([u, v, 1.0], dtype=np.float64)
    w = H @ p
    return float(w[0] / w[2]), float(w[1] / w[2])


def pixel_to_world_batch(pixels: np.ndarray) -> np.ndarray:
    """
    Map N pixels to real-world coordinates in one vectorised call.

    Parameters
    ──────────
    pixels : array-like, shape (N, 2)  –  columns are [u, v]

    Returns
    ───────
    np.ndarray, shape (N, 2)  –  columns are [x_mm, y_mm]

    Example
    ───────
    detections = np.array([[120, 340], [500, 210], [88, 400]])
    coords     = pixel_to_world_batch(detections)
    # coords[0] → (x_mm, y_mm) for the first detection
    """
    px   = np.asarray(pixels, dtype=np.float64)           # (N, 2)
    ones = np.ones((len(px), 1), dtype=np.float64)
    hom  = np.hstack([px, ones])                          # (N, 3)
    w    = (H @ hom.T).T                                  # (N, 3)
    return w[:, :2] / w[:, 2:3]                           # (N, 2)


def reload_calibration(path: str = _CALIBRATION_PATH) -> None:
    """
    Hot-reload H from disk without restarting your process.
    Useful if you recalibrate while the robot pipeline is running.
    """
    global H
    H = _load_H(path)
    print(f"  Calibration reloaded from {path}")