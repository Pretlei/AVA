"""
colour_picker.py
================
Point your webcam at an object, click on it, and get its HSV values
along with recommended h_min/h_max/s_min etc. ranges to paste into
dual_object_tracker.py.

# No idea what this code does, made using Claude

Controls
--------
  Click       – sample the pixel under your cursor
  C           – clear all samples
  E / Shift+E – increase / decrease exposure (stops)
  Q / ESC     – quit and print final recommended ranges
"""

import cv2
import numpy as np
import sys
import time


# Accumulates (H, S, V) tuples from every click
samples = []

# Current manual exposure stop value (DSHOW range is typically -11 to 0)
exposure_val = -4


def on_mouse(event, x, y, flags, param):
    if event != cv2.EVENT_LBUTTONDOWN:
        return
    frame, hsv = param
    h, s, v = hsv[y, x]
    samples.append((int(h), int(s), int(v)))
    print(f"  Clicked ({x:4d}, {y:4d})  ->  H={h:3d}  S={s:3d}  V={v:3d}")
    cv2.circle(frame, (x, y), 6, (0, 255, 255), 2)
    cv2.circle(frame, (x, y), 2, (0, 255, 255), -1)


def recommend(samples, hue_margin=10, sv_margin=40):
    if not samples:
        return None
    hs = [s[0] for s in samples]
    ss = [s[1] for s in samples]
    vs = [s[2] for s in samples]

    h_min = max(0,   min(hs) - hue_margin)
    h_max = min(179, max(hs) + hue_margin)
    s_min = max(0,   min(ss) - sv_margin)
    s_max = min(255, max(ss) + sv_margin)
    v_min = max(0,   min(vs) - sv_margin)
    v_max = min(255, max(vs) + sv_margin)

    wrap = (min(hs) < 15 or max(hs) > 165)

    return dict(h_min=h_min, h_max=h_max,
                s_min=s_min, s_max=s_max,
                v_min=v_min, v_max=v_max,
                wrap_hue=wrap)


def set_exposure(cap, val):
    """Disable auto-exposure and apply a manual stop value."""
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # 0.25 = manual mode on DSHOW
    cap.set(cv2.CAP_PROP_EXPOSURE, val)
    actual = cap.get(cv2.CAP_PROP_EXPOSURE)
    print(f"[INFO] Exposure set to {val} (driver reports {actual})")


def open_camera(preferred_index=None):
    indices = ([preferred_index] if preferred_index is not None else []) + [0, 1, 2, 3, 4]
    backends = [cv2.CAP_DSHOW, cv2.CAP_ANY]
    for idx in indices:
        for backend in backends:
            cap = cv2.VideoCapture(idx, backend)
            if not cap.isOpened():
                cap.release()
                continue
            time.sleep(0.5)
            warmed = False
            for _ in range(30):
                ret, frame = cap.read()
                if ret and frame is not None and frame.size > 0:
                    warmed = True
                    break
            if warmed:
                print(f"[INFO] Opened camera index {idx} (backend {backend})")
                return cap
            cap.release()
    print("[ERROR] No working camera found.")
    sys.exit(1)


def main(preferred_cam=None):
    global exposure_val

    cap = open_camera(preferred_cam)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,  720)
    cap.set(cv2.CAP_PROP_FPS,            30)

    # Disable auto-exposure and set initial manual exposure
    set_exposure(cap, exposure_val)

    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    actual_w   = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    actual_h   = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    print(f"[INFO] Camera: {int(actual_w)}x{int(actual_h)} @ {actual_fps:.0f} fps")

    win = "Colour Picker  -  click your object | C=clear | E=exposure | Q=quit"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 1280, 720)

    print("\nColour Picker")
    print("-" * 57)
    print("Click directly on your object (5-10 spots across its")
    print("surface). Press Q when done to see the HSV ranges.")
    print("E = raise exposure (+1 stop)  Shift+E = lower (-1 stop)\n")

    while True:
        ret, frame = cap.read()
        if not ret or frame is None or frame.size == 0:
            continue

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        display_frame = frame.copy()
        cv2.setMouseCallback(win, on_mouse, (display_frame, hsv))

        # HUD
        cv2.putText(display_frame,
                    f"Samples: {len(samples)}  |  Exposure: {exposure_val}  |  C=clear  E=exposure  Q=quit",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        if samples:
            rec = recommend(samples)
            summary = (f"H {rec['h_min']}-{rec['h_max']}  "
                       f"S {rec['s_min']}-{rec['s_max']}  "
                       f"V {rec['v_min']}-{rec['v_max']}"
                       + ("  [RED wrap]" if rec["wrap_hue"] else ""))
            cv2.putText(display_frame, summary,
                        (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)

            lower = np.array([rec["h_min"], rec["s_min"], rec["v_min"]], dtype=np.uint8)
            upper = np.array([rec["h_max"], rec["s_max"], rec["v_max"]], dtype=np.uint8)
            mask  = cv2.inRange(hsv, lower, upper)
            if rec["wrap_hue"]:
                lower2 = np.array([170, rec["s_min"], rec["v_min"]], dtype=np.uint8)
                upper2 = np.array([179, rec["s_max"], rec["v_max"]], dtype=np.uint8)
                mask   = cv2.bitwise_or(mask, cv2.inRange(hsv, lower2, upper2))

            ih, iw = display_frame.shape[:2]
            inset_w, inset_h = 240, 180
            inset = cv2.resize(mask, (inset_w, inset_h))
            inset_bgr = cv2.cvtColor(inset, cv2.COLOR_GRAY2BGR)
            display_frame[5: 5 + inset_h, iw - inset_w - 5: iw - 5] = inset_bgr
            cv2.rectangle(display_frame,
                          (iw - inset_w - 5, 5),
                          (iw - 5, 5 + inset_h),
                          (0, 255, 255), 1)
            cv2.putText(display_frame, "mask preview",
                        (iw - inset_w - 2, inset_h + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

        cv2.imshow(win, display_frame)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        elif key == ord("c"):
            samples.clear()
            print("[INFO] Samples cleared.")
        elif key == ord("e"):
            # Raise exposure by 1 stop (brighter), cap at 0
            exposure_val = min(0, exposure_val + 1)
            set_exposure(cap, exposure_val)
        elif key == ord("E"):
            # Lower exposure by 1 stop (darker), floor at -11
            exposure_val = max(-11, exposure_val - 1)
            set_exposure(cap, exposure_val)

    cap.release()
    cv2.destroyAllWindows()

    print("\n" + "-" * 57)
    if not samples:
        print("No samples collected.")
        return

    rec = recommend(samples)
    print(f"Collected {len(samples)} sample(s).\n")
    print("Paste these values into dual_object_tracker.py:\n")
    print(f'    "h_min": {rec["h_min"]}, "h_max": {rec["h_max"]},')
    print(f'    "s_min": {rec["s_min"]}, "s_max": {rec["s_max"]},')
    print(f'    "v_min": {rec["v_min"]}, "v_max": {rec["v_max"]},')
    print(f'    "wrap_hue": {rec["wrap_hue"]},')
    if rec["wrap_hue"]:
        print("    # Red object detected — wrap_hue should be True.")
    print()


if __name__ == "__main__":
    cam = int(sys.argv[1]) if len(sys.argv) > 1 else None
    main(cam)