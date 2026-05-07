"""
arm_calibration.py
──────────────────────────────────────────────────────────────────────────────
Builds an error correction map for the robot arm using the camera instead
of a ruler. For each grid point:
  1. Arm moves to the target position
  2. Camera feed opens — click on the actual fingertip position
  3. pixel_to_world converts the click to mm automatically
  4. Correction (delta) is computed and saved

Usage
─────
  python arm_calibration.py          # collect corrections
  python arm_calibration.py --test   # verify corrections are working

Requirements
────────────
  pip install scipy
"""

import json
import sys
import time

import cv2
import numpy as np
from scipy.interpolate import RBFInterpolator

from serial_comms_comp_1 import RobotArm
from ik_solver_2 import IKController
from transform_coordinates_3 import pixel_to_world
from object_detection_4 import open_camera, set_exposure, EXPOSURE_START

# ─────────────────────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────────────────────

PORT   = "COM6"
PICK_Z = -90
SLEEP_MOVE      = 3.0   # seconds to wait after each arm move
CORRECTIONS_PATH = "arm_corrections.json"

# Grid of (x, y) world coordinates to test — adjust to your workspace (mm).
TEST_GRID = [
    # existing — keep these
    (200, -150), (200,   0), (200, 150),
    (260, -150), (260,   0), (260, 150),
    (320, -100), (320,   0), (320, 100),

    # add — denser near lateral extremes where error is highest
    (230, -150), (230, -75), (230,   0), (230,  75), (230, 150),
    (290, -150), (290, -75), (290,   0), (290,  75), (290, 150),
    (320, -150), (320,  150),
]


# ─────────────────────────────────────────────────────────────────────────────
# Camera click — open feed, wait for one click, return world coords
# ─────────────────────────────────────────────────────────────────────────────

def get_click_world(cap, prompt: str) -> tuple | None:
    """
    Show live camera feed with prompt text.
    Left-click to confirm fingertip position → returns (x_mm, y_mm).
    Press S to skip this point, Q to abort the whole session.
    """
    clicked = []

    def on_mouse(event, x, y, flags, _):
        if event == cv2.EVENT_LBUTTONDOWN:
            clicked.append((x, y))

    win = "Arm Calibration"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 1280, 720)
    cv2.setMouseCallback(win, on_mouse)

    print(f"  Camera open — {prompt}")
    print("  Left-click on fingertip  |  S = skip  |  Q = quit")

    result = None
    while True:
        ret, frame = cap.read()
        if not ret or frame is None or frame.size == 0:
            continue

        display = frame.copy()

        # Draw crosshair at last click
        if clicked:
            cx, cy = clicked[-1]
            cv2.circle(display, (cx, cy), 8, (0, 255, 0), -1)
            cv2.line(display, (cx - 20, cy), (cx + 20, cy), (0, 255, 0), 1)
            cv2.line(display, (cx, cy - 20), (cx, cy + 20), (0, 255, 0), 1)
            x_mm, y_mm = pixel_to_world(cx, cy)
            cv2.putText(display, f"({x_mm:.1f}, {y_mm:.1f}) mm — press ENTER to confirm",
                        (cx + 12, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)

        cv2.putText(display, prompt,
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 220, 255), 2)
        cv2.putText(display, "Click fingertip  |  ENTER=confirm  |  S=skip  |  Q=quit",
                    (10, display.shape[0] - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (180, 180, 180), 1)

        cv2.imshow(win, display)
        key = cv2.waitKey(1) & 0xFF

        if key in (ord("q"), 27):
            cv2.destroyWindow(win)
            return "quit"
        elif key == ord("s"):
            cv2.destroyWindow(win)
            return None
        elif key in (13, 10) and clicked:   # Enter — confirm last click
            cx, cy = clicked[-1]
            x_mm, y_mm = pixel_to_world(cx, cy)
            cv2.destroyWindow(win)
            return (x_mm, y_mm)


# ─────────────────────────────────────────────────────────────────────────────
# Correction map — save / load / interpolate
# ─────────────────────────────────────────────────────────────────────────────

def save_corrections(corrections: dict) -> None:
    data = [{"target": list(k), "delta": list(v)} for k, v in corrections.items()]
    with open(CORRECTIONS_PATH, "w") as f:
        json.dump(data, f, indent=2)
    print(f"\n  ✓ Saved {len(data)} corrections → {CORRECTIONS_PATH}")


def load_corrections() -> dict:
    try:
        with open(CORRECTIONS_PATH) as f:
            content = f.read().strip()
            if not content:
                return {}
            data = json.loads(content)
        return {tuple(d["target"]): tuple(d["delta"]) for d in data}
    except FileNotFoundError:
        return {}


def build_interpolator(corrections: dict):
    points = np.array(list(corrections.keys()),   dtype=np.float64)
    deltas = np.array(list(corrections.values()), dtype=np.float64)
    return RBFInterpolator(points, deltas, kernel="thin_plate_spline")


def apply_correction(interp, x: float, y: float) -> tuple:
    dx, dy = interp([[x, y]])[0]
    return x + dx, y + dy


# ─────────────────────────────────────────────────────────────────────────────
# Collection pass
# ─────────────────────────────────────────────────────────────────────────────

def collect_corrections() -> None:
    print("\n══ Arm Error Correction Calibration ═════════════════════════")
    print(f"  {len(TEST_GRID)} test points  |  Z = {PICK_Z} mm  |  port {PORT}")
    print("\n  For each point:")
    print("    1. Arm moves to target")
    print("    2. Camera opens — click on the actual fingertip position")
    print("    3. Press ENTER to confirm, S to skip, Q to quit\n")
    input("  Press ENTER to connect and begin...")

    # Load existing corrections so partial sessions can be resumed
    try:
        corrections = load_corrections()
        print(f"  Loaded {len(corrections)} existing corrections.")
    except FileNotFoundError:
        corrections = {}

    cap, _ = open_camera()
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,  720)
    cap.set(cv2.CAP_PROP_FPS,            60)
    set_exposure(cap, EXPOSURE_START)
    time.sleep(1.0)   # warm up

    with RobotArm(PORT) as arm:
        controller = IKController(arm)
        arm.rest()
        time.sleep(SLEEP_MOVE)

        for i, (tx, ty) in enumerate(TEST_GRID):
            print(f"\n{'─'*54}")
            print(f"  Point {i+1}/{len(TEST_GRID)}  —  target ({tx}, {ty}) mm")
            print(f"{'─'*54}")

            try:
                controller.move_to(tx, ty, z=PICK_Z)
            except ValueError as e:
                print(f"  [SKIP] IK failed: {e}")
                continue

            time.sleep(SLEEP_MOVE)

            result = get_click_world(
                cap,
                prompt=f"Point {i+1}/{len(TEST_GRID)}: target ({tx}, {ty}) mm — click fingertip"
            )

            if result == "quit":
                print("\n  Session aborted by user.")
                break
            elif result is None:
                print("  Skipped.")
                continue
            else:
                actual_x, actual_y = result
                dx = tx - actual_x
                dy = ty - actual_y
                corrections[(tx, ty)] = (dx, dy)
                print(f"  Actual  : ({actual_x:.1f}, {actual_y:.1f}) mm")
                print(f"  Delta   : ({dx:+.1f}, {dy:+.1f}) mm")

        print("\n  Returning to rest...")
        arm.rest()
        time.sleep(SLEEP_MOVE)

    cap.release()
    cv2.destroyAllWindows()

    if corrections:
        save_corrections(corrections)
        print(f"\n══ Done — {len(corrections)} corrections saved. ══════════════════")
        print(f"  Run  python arm_calibration.py --test  to verify accuracy.\n")
    else:
        print("\n  No corrections collected — nothing saved.")


# ─────────────────────────────────────────────────────────────────────────────
# Test pass
# ─────────────────────────────────────────────────────────────────────────────

def test_corrections() -> None:
    print("\n══ Testing Interpolated Corrections ═════════════════════════")
    corrections  = load_corrections()
    interp       = build_interpolator(corrections)
    print(f"  Loaded {len(corrections)} correction points.")
    print("  Arm moves to each point WITH corrections — click the fingertip")
    print("  to measure residual error. Should be near 0.\n")
    input("  Press ENTER to connect and begin...")

    cap, _ = open_camera()
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,  720)
    cap.set(cv2.CAP_PROP_FPS,            60)
    set_exposure(cap, EXPOSURE_START)
    time.sleep(1.0)

    errors = []

    with RobotArm(PORT) as arm:
        controller = IKController(arm)
        arm.rest()
        time.sleep(SLEEP_MOVE)

        for i, (tx, ty) in enumerate(TEST_GRID):
            print(f"\n{'─'*54}")
            print(f"  Point {i+1}/{len(TEST_GRID)}  —  target ({tx}, {ty}) mm")

            cx, cy = apply_correction(interp, tx, ty)
            print(f"  Corrected target: ({cx:.1f}, {cy:.1f}) mm")

            try:
                controller.move_to(cx, cy, z=PICK_Z)
            except ValueError as e:
                print(f"  [SKIP] IK failed: {e}")
                continue

            time.sleep(SLEEP_MOVE)

            result = get_click_world(
                cap,
                prompt=f"Point {i+1}/{len(TEST_GRID)}: target ({tx}, {ty}) mm — click fingertip"
            )

            if result == "quit":
                print("\n  Test aborted by user.")
                break
            elif result is None:
                print("  Skipped.")
                continue
            else:
                actual_x, actual_y = result
                err = np.hypot(tx - actual_x, ty - actual_y)
                errors.append(err)
                print(f"  Actual  : ({actual_x:.1f}, {actual_y:.1f}) mm")
                print(f"  Residual: {err:.1f} mm")

        print("\n  Returning to rest...")
        arm.rest()
        time.sleep(SLEEP_MOVE)

    cap.release()
    cv2.destroyAllWindows()

    if errors:
        errors = np.array(errors)
        print(f"\n══ Test Results ══════════════════════════════════════════")
        print(f"  Points tested : {len(errors)}")
        print(f"  Mean error    : {errors.mean():.1f} mm")
        print(f"  Max error     : {errors.max():.1f} mm")
        print(f"  RMSE          : {np.sqrt((errors**2).mean()):.1f} mm")
        if errors.mean() < 5:
            print("  ✓ Accuracy good — ready to use in pipeline.")
        else:
            print("  ⚠ Mean error > 5 mm — add more TEST_GRID points in")
            print("    high-error regions and re-run collection.")
    print()


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    if "--test" in sys.argv:
        test_corrections()
    else:
        collect_corrections()