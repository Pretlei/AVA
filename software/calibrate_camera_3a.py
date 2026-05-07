"""
calibrate.py

Steps:
    1. Edit WORLD_POINTS below to match your physical marker positions (mm).
    2. Place the markers on the table.
    3. Run the script – a camera window opens.
    4. Click each marker in the same order as WORLD_POINTS.
    5. Inspect the accuracy report and overlay.
    6. calibration.json is written automatically.
"""

import cv2
import numpy as np
import json
import os


# ─────────────────────────────────────────────────────────────────────────────
# ❶  EDIT THIS  –  your real-world marker positions in mm
#    Origin = wherever you choose (robot base, workspace corner, etc.)
#    X = right,  Y = away from you  (or whatever frame you use for the arm)
#    Use ≥ 6 points spread across ALL FOUR corners of the workspace.
# ─────────────────────────────────────────────────────────────────────────────
WORLD_POINTS: list[tuple[float, float]] = [
    (192.0, -163.0),   # 1 Top left
    (190.0,   9.0),   # 2 Top middle
    (192.0,   163.0),   # 3 Top right
    (285.0, -167.0),   # 4  Middle left
    (293.0, 8.0),   # 5 Centre
    (302.0, 158.0),   # 6   Middle right
    (379.0, -177.0),   # 7 Bottom left
    (388.0, 3.0),   # 8 bottom middle
    (398.0, 149.0),   # 9 bottom right
]

CAMERA_INDEX   = 0                  # change if your webcam isn't index 0
OUTPUT_PATH    = "calibration.json"
RANSAC_THRESH  = 3.0                # pixels – RANSAC inlier threshold


# ─────────────────────────────────────────────────────────────────────────────
# Internals  (no need to edit below)
# ─────────────────────────────────────────────────────────────────────────────

def _collect_clicks(num_points: int) -> list[tuple[float, float]]:
    """Open the camera and collect mouse clicks until num_points are reached."""
    clicks: list[tuple[float, float]] = []
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)   # add
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,  720)   # add
    cap.set(cv2.CAP_PROP_FPS,            60)   # add
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera index {CAMERA_INDEX}")

    win = "CALIBRATION – click each marker in order  |  Q = abort"

    def on_mouse(event, x, y, flags, _):
        if event == cv2.EVENT_LBUTTONDOWN:
            clicks.append((float(x), float(y)))
            print(f"  [{len(clicks):>2}/{num_points}]  pixel ({x:>4}, {y:>4})")

    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        display = frame.copy()
        for i, (u, v) in enumerate(clicks):
            cv2.circle(display, (int(u), int(v)), 6, (0, 255, 0), -1)
            cv2.circle(display, (int(u), int(v)), 8, (0, 200, 0), 2)
            cv2.putText(display, str(i + 1), (int(u) + 10, int(v) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

        status = (f"Clicked: {len(clicks)}/{num_points}"
                  + ("  – all done!" if len(clicks) >= num_points else ""))
        cv2.putText(display, status, (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.imshow(win, display)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            print("  Aborted.")
            break
        if len(clicks) >= num_points:
            cv2.waitKey(600)
            break

    cap.release()
    cv2.destroyAllWindows()
    return clicks


def _build_homography(
    pixel_pts: list[tuple[float, float]],
    world_pts: list[tuple[float, float]],
) -> tuple[np.ndarray, np.ndarray]:
    """Compute H via RANSAC. Returns (H, inlier_mask)."""
    src = np.array(pixel_pts, dtype=np.float64)
    dst = np.array(world_pts, dtype=np.float64)
    H, mask = cv2.findHomography(src, dst, cv2.RANSAC, RANSAC_THRESH)
    if H is None:
        raise RuntimeError("findHomography() failed – check your point correspondences.")
    n_in = int(mask.sum()) if mask is not None else len(pixel_pts)
    print(f"\n  Homography computed  ({n_in}/{len(pixel_pts)} inliers)")
    print(f"  H =\n{H}\n")
    return H, mask


def _apply_H(u: float, v: float, H: np.ndarray) -> tuple[float, float]:
    """Single-point homogeneous projection."""
    p = np.array([u, v, 1.0], dtype=np.float64)
    w = H @ p
    return float(w[0] / w[2]), float(w[1] / w[2])


def _verify(
    pixel_pts: list[tuple[float, float]],
    world_pts: list[tuple[float, float]],
    H: np.ndarray,
) -> float:
    """Leave-one-out cross-validation. Prints a table, returns RMSE (mm)."""
    src = np.array(pixel_pts, dtype=np.float64)
    dst = np.array(world_pts, dtype=np.float64)
    n   = len(src)
    errors = []

    print(f"\n{'─'*62}")
    print("  Leave-one-out validation")
    print(f"{'─'*62}")
    print(f"  {'Pt':>3}   {'predicted (mm)':^24}   {'actual (mm)':^24}   {'err':>7}")
    print(f"  {'──':>3}   {'─'*24}   {'─'*24}   {'───':>7}")

    for i in range(n):
        mask_loo = np.arange(n) != i
        H_loo, _ = cv2.findHomography(src[mask_loo], dst[mask_loo],
                                       cv2.RANSAC, RANSAC_THRESH)
        if H_loo is None:
            print(f"  [{i+1:>2}]  ⚠  skipped (homography failed without this point)")
            continue
        pred = _apply_H(src[i, 0], src[i, 1], H_loo)
        true = dst[i]
        err  = float(np.linalg.norm(np.array(pred) - true))
        errors.append(err)
        print(f"  [{i+1:>2}]   ({pred[0]:>9.2f}, {pred[1]:>9.2f})   "
              f"({true[0]:>9.2f}, {true[1]:>9.2f})   {err:>6.2f} mm")

    errors = np.array(errors)
    rmse   = float(np.sqrt((errors**2).mean()))
    print(f"\n  Mean  : {errors.mean():.2f} mm")
    print(f"  Max   : {errors.max():.2f} mm")
    print(f"  RMSE  : {rmse:.2f} mm")
    print(f"{'─'*62}\n")
    return rmse


def _show_overlay(
    pixel_pts: list[tuple[float, float]],
    world_pts: list[tuple[float, float]],
    H: np.ndarray,
) -> None:
    """Grab one frame and draw clicked dots vs back-projected dots."""
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)   # add
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,  720)   # add
    cap.set(cv2.CAP_PROP_FPS,            60)   # add
    ret, frame = cap.read()
    cap.release()
    if not ret:
        print("  [overlay] Could not grab frame.")
        return

    H_inv = np.linalg.inv(H)
    for (u, v), (xw, yw) in zip(pixel_pts, world_pts):
        cv2.circle(frame, (int(u), int(v)), 7, (0, 255, 0), -1)          # green = clicked
        bp = _apply_H(xw, yw, H_inv)
        cv2.circle(frame, (int(bp[0]), int(bp[1])), 5, (0, 0, 255), -1)  # red = back-project
        cv2.line(frame, (int(u), int(v)), (int(bp[0]), int(bp[1])),
                 (0, 140, 255), 1)
        cv2.putText(frame, f"({xw:.0f},{yw:.0f})",
                    (int(u) + 10, int(v) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 230, 230), 1)

    cv2.putText(frame, "GREEN=clicked  RED=back-projected  |  any key to close",
                (10, frame.shape[0] - 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (200, 200, 200), 1)
    cv2.imshow("Verification overlay", frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def _save(H: np.ndarray,
          pixel_pts: list[tuple[float, float]],
          world_pts: list[tuple[float, float]]) -> None:
    data = {
        "H"            : H.tolist(),
        "pixel_points" : pixel_pts,
        "world_points" : world_pts,
    }
    with open(OUTPUT_PATH, "w") as f:
        json.dump(data, f, indent=2)
    print(f"  ✓  Saved → {os.path.abspath(OUTPUT_PATH)}")


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main() -> None:
    n = len(WORLD_POINTS)

    print("\n" + "═"*62)
    print("  HOMOGRAPHY CALIBRATION")
    print("═"*62)
    print(f"\n  Click {n} markers in this order:\n")
    for i, (x, y) in enumerate(WORLD_POINTS):
        print(f"    [{i+1}]  ({x:>7.1f} mm,  {y:>7.1f} mm)")

    input("\n  Place markers on the table, then press ENTER to open camera…")

    # 1. Collect pixel clicks
    pixel_pts = _collect_clicks(n)
    if len(pixel_pts) < 4:
        raise RuntimeError("Need at least 4 clicks – rerun and try again.")
    world_pts = WORLD_POINTS[: len(pixel_pts)]

    # 2. Compute H
    H, _ = _build_homography(pixel_pts, world_pts)

    # 3. Verify
    rmse = _verify(pixel_pts, world_pts, H)
    if rmse > 5.0:
        print("  ⚠  RMSE > 5 mm – consider re-collecting (move markers to corners).")
    else:
        print("  ✓  Accuracy good – ready to save.")

    # 4. Overlay
    _show_overlay(pixel_pts, world_pts, H)

    # 5. Save
    _save(H, pixel_pts, world_pts)
    print("\n  Calibration complete. Run your robot pipeline using transform.py.\n")


if __name__ == "__main__":
    main()