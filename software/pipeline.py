"""
pipeline.py
──────────────────────────────────────────────────────────────────────────────
Autonomous pick-and-place pipeline for the How To Mechatronics robot arm.

Requires arm_corrections.json — run arm_calibration.py first.

Tuning constants
────────────────
  PORT              – COM port of your Arduino
  SLEEP_MOVE        – seconds to wait after move_to
  SLEEP_PRESET      – seconds to wait after move_multi_preset
  SLEEP_GRIP        – seconds to wait after gripper open/close
  PICK_Z            – Z height (mm) when picking
  PLACE_Z           – Z height (mm) when placing (raised by plate thickness)
  DETECTION_FRAMES  – frames to sample during detection
  DETECTION_MIN_HITS– min frames an object must appear in to be trusted
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
from object_detection_4 import (
    open_camera,
    set_exposure,
    capture_stable_detections,
    EXPOSURE_START,
)

# ─────────────────────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────────────────────

PORT = "COM6"

SLEEP_MOVE   = 3.0
SLEEP_PRESET = 2.0
SLEEP_GRIP   = 1.0

PICK_Z  = -90
PLACE_Z = -85

DETECTION_FRAMES   = 20
DETECTION_MIN_HITS = 10

CORRECTIONS_PATH = "arm_corrections.json"

PAIRS = [
    ("cube_pink",  "plate_purple", "Pink Cube → Purple Plate"),
    ("cube_green", "plate_orange", "Green Cube → Orange Plate"),
]


# ─────────────────────────────────────────────────────────────────────────────
# Correction map
# ─────────────────────────────────────────────────────────────────────────────

def load_interpolator():
    try:
        with open(CORRECTIONS_PATH) as f:
            data = json.load(f)
    except FileNotFoundError:
        print(f"[WARN] {CORRECTIONS_PATH} not found — running without corrections.")
        return None

    points = np.array([d["target"] for d in data], dtype=np.float64)
    deltas = np.array([d["delta"]  for d in data], dtype=np.float64)
    interp = RBFInterpolator(points, deltas, kernel="thin_plate_spline")
    print(f"[INFO] Loaded correction map ({len(data)} points).")
    return interp


def apply_correction(interp, x: float, y: float) -> tuple:
    if interp is None:
        return x, y
    dx, dy = interp([[x, y]])[0]
    return x + dx, y + dy


# ─────────────────────────────────────────────────────────────────────────────
# Stage 1: Vision
# ─────────────────────────────────────────────────────────────────────────────

def detect_all_objects() -> dict:
    print("\n══ Stage 1: Object Detection ════════════════════════════════")
    cap, backend = open_camera()
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,  720)
    cap.set(cv2.CAP_PROP_FPS,            60)
    set_exposure(cap, EXPOSURE_START)

    print("[Vision] Warming up camera for 1 s...")
    time.sleep(1.0)

    pixel_detections = capture_stable_detections(
        cap,
        num_frames=DETECTION_FRAMES,
        min_detections=DETECTION_MIN_HITS,
    )
    cap.release()
    cv2.destroyAllWindows()
    return pixel_detections


# ─────────────────────────────────────────────────────────────────────────────
# Stage 2: Transform
# ─────────────────────────────────────────────────────────────────────────────

def to_world_coords(pixel_detections: dict) -> dict:
    print("\n══ Stage 2: Coordinate Transform ════════════════════════════")
    world = {}
    for name, (cx, cy) in pixel_detections.items():
        x_mm, y_mm = pixel_to_world(cx, cy)
        world[name] = (x_mm, y_mm)
        print(f"  {name:15s}  pixel ({cx:4d}, {cy:4d})  →  "
              f"world ({x_mm:7.1f}, {y_mm:7.1f}) mm")
    return world


# ─────────────────────────────────────────────────────────────────────────────
# Stage 3: Motion
# ─────────────────────────────────────────────────────────────────────────────

def pick_and_place(controller: IKController,
                   arm: RobotArm,
                   pick_pos: tuple,
                   place_pos: tuple,
                   label: str,
                   interp) -> None:
    px, py = pick_pos
    dx, dy = place_pos

    px_adj, py_adj = apply_correction(interp, px, py)
    dx_adj, dy_adj = apply_correction(interp, dx, dy)

    print(f"\n── {label} ────────────────────────────────────────────────")
    print(f"   Pick  : ({px:.1f}, {py:.1f}) → corrected ({px_adj:.1f}, {py_adj:.1f}, {PICK_Z}) mm")
    print(f"   Place : ({dx:.1f}, {dy:.1f}) → corrected ({dx_adj:.1f}, {dy_adj:.1f}, {PLACE_Z}) mm")

    print("→ [1/9] Moving to rest...")
    arm.rest()
    time.sleep(SLEEP_MOVE)

    print("→ [2/9] Opening gripper...")
    arm.open_gripper()
    time.sleep(SLEEP_GRIP)

    print(f"→ [3/9] Moving to cube at ({px_adj:.1f}, {py_adj:.1f}, {PICK_Z}) mm...")
    controller.move_to(px_adj, py_adj, z=PICK_Z)
    time.sleep(SLEEP_MOVE)

    print("→ [4/9] Closing gripper...")
    arm.close_gripper()
    time.sleep(SLEEP_GRIP)

    print("→ [5/9] Lifting arm...")
    arm.move_multi_preset()
    time.sleep(SLEEP_PRESET)

    print(f"→ [6/9] Moving to plate at ({dx_adj:.1f}, {dy_adj:.1f}, {PLACE_Z}) mm...")
    controller.move_to(dx_adj, dy_adj, z=PLACE_Z)
    time.sleep(SLEEP_MOVE)

    print("→ [7/9] Opening gripper...")
    arm.open_gripper()
    time.sleep(SLEEP_GRIP)

    print("→ [8/9] Retracting arm...")
    arm.move_multi_preset()
    time.sleep(SLEEP_PRESET)

    print("→ [9/9] Returning to rest...")
    arm.rest()
    time.sleep(SLEEP_MOVE)

    print(f"   ✓ {label} complete.")


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    interp = load_interpolator()

    pixel_detections = detect_all_objects()
    if not pixel_detections:
        print("\n[ERROR] No objects detected. Check lighting and HSV thresholds.")
        sys.exit(1)

    world_coords = to_world_coords(pixel_detections)

    print("\n══ Pair Resolution ══════════════════════════════════════════")
    tasks = []
    for cube_key, plate_key, label in PAIRS:
        if cube_key not in world_coords:
            print(f"  [SKIP] {label}  — {cube_key} not detected.")
            continue
        if plate_key not in world_coords:
            print(f"  [SKIP] {label}  — {plate_key} not detected.")
            continue
        tasks.append((world_coords[cube_key], world_coords[plate_key], label))
        print(f"  [OK]   {label}")

    if not tasks:
        print("\n[ERROR] No complete pairs detected. Exiting.")
        sys.exit(1)

    print(f"\n══ Stage 3: Robot Motion  (port {PORT}) ══════════════════════")
    with RobotArm(PORT) as arm:
        controller = IKController(arm)
        for pick_pos, place_pos, label in tasks:
            try:
                pick_and_place(controller, arm, pick_pos, place_pos, label, interp)
            except ValueError as e:
                print(f"\n[ERROR] IK failed for '{label}': {e}")
                print("  Returning to rest and continuing...")
                arm.rest()
                time.sleep(SLEEP_MOVE)

    print("\n══ Pipeline complete ════════════════════════════════════════")


if __name__ == "__main__":
    main()