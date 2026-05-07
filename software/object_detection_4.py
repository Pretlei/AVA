"""
object_detection_3.py
Detects 4 coloured objects (2 cubes, 2 plates) from a webcam feed.

Controls (standalone): Q/ESC=quit  S=snapshot  E=exp+  Shift+E=exp-
Exposure keys only work under DSHOW; MSMF uses auto-exposure.

Callable API (for pipeline.py):
  detect_once(frame)               -> dict[str, (cx, cy)]
  capture_stable_detections(cap)   -> dict[str, (cx, cy)]
"""

import cv2
import numpy as np
import sys
import time

# ── Object configs — update HSV ranges with colour_picker.py ─────────────────
OBJECTS = {
    "cube_pink": {
        "h_min": 140, "h_max": 170, "s_min":  41, "s_max": 139,
        "v_min": 177, "v_max": 255, "wrap_hue": False,
        "h_wrap_upper": (170, 179), "colour_bgr": (255, 80, 50),
        "label": "Pink",   "open_k": 5, "close_k": 10, "min_area": 500,
    },
    "cube_green": {
        "h_min":  65, "h_max":  87, "s_min":  73, "s_max": 161,
        "v_min": 168, "v_max": 255, "wrap_hue": False,
        "h_wrap_upper": (170, 179), "colour_bgr": (50, 220, 50),
        "label": "Green",  "open_k": 5, "close_k": 10, "min_area": 500,
    },
    "plate_purple": {
        "h_min": 114, "h_max": 136, "s_min": 123, "s_max": 217,
        "v_min": 161, "v_max": 255, "wrap_hue": False,
        "h_wrap_upper": (170, 179), "colour_bgr": (0, 220, 255),
        "label": "Purple", "open_k": 5, "close_k": 10, "min_area": 500,
    },
    "plate_orange": {
        "h_min":   0, "h_max":  15, "s_min": 102, "s_max": 195,
        "v_min": 208, "v_max": 255, "wrap_hue": True,
        "h_wrap_upper": (170, 179), "colour_bgr": (50, 50, 255),
        "label": "Orange", "open_k": 5, "close_k": 10, "min_area": 500,
    },
}

EXPOSURE_START = -4   # only applies under DSHOW


# ── Camera ────────────────────────────────────────────────────────────────────
def _read(cap):
    """Safe cap.read() — returns (False, None) on cv2.error."""
    try:
        return cap.read()
    except cv2.error:
        return False, None


def open_camera(preferred_index=None):
    """
    Returns (cap, backend). Tries MSMF first (better throughput on Windows),
    falls back to CAP_ANY. Does NOT set FOURCC — causes MSMF stream errors.
    """
    indices  = ([preferred_index] if preferred_index is not None else []) + [0, 1, 2, 3, 4]
    backends = [cv2.CAP_DSHOW, cv2.CAP_ANY]
    for idx in indices:
        for backend in backends:
            cap = cv2.VideoCapture(idx, backend)
            if not cap.isOpened():
                cap.release()
                continue
            time.sleep(0.5)
            for _ in range(30):
                ret, frame = _read(cap)
                if ret and frame is not None and frame.size > 0:
                    print(f"[INFO] Opened camera index {idx} (backend {backend})")
                    return cap, backend
            cap.release()
    print("[ERROR] No working camera found.")
    sys.exit(1)


def set_exposure(cap, val):
    """Manual exposure — DSHOW only. Silently ignored under MSMF."""
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    cap.set(cv2.CAP_PROP_EXPOSURE, val)
    print(f"[INFO] Exposure set to {val} (driver reports {cap.get(cv2.CAP_PROP_EXPOSURE)})")


# ── Detection helpers ─────────────────────────────────────────────────────────
def build_mask(hsv, cfg):
    lo  = np.array([cfg["h_min"], cfg["s_min"], cfg["v_min"]], dtype=np.uint8)
    hi  = np.array([cfg["h_max"], cfg["s_max"], cfg["v_max"]], dtype=np.uint8)
    mask = cv2.inRange(hsv, lo, hi)
    if cfg["wrap_hue"]:
        h_lo, h_hi = cfg["h_wrap_upper"]
        lo2  = np.array([h_lo, cfg["s_min"], cfg["v_min"]], dtype=np.uint8)
        hi2  = np.array([h_hi, cfg["s_max"], cfg["v_max"]], dtype=np.uint8)
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo2, hi2))
    return mask


def clean_mask(mask, open_k, close_k):
    k1 = cv2.getStructuringElement(cv2.MORPH_RECT, (open_k,  open_k))
    k2 = cv2.getStructuringElement(cv2.MORPH_RECT, (close_k, close_k))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k2)
    return mask


def detect_object(mask, min_area):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, None, None, 0.0
    best = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(best)
    if area < min_area:
        return None, None, None, area
    M = cv2.moments(best)
    if M["m00"] == 0:
        return None, None, None, area
    cx  = int(M["m10"] / M["m00"])
    cy  = int(M["m01"] / M["m00"])
    box = np.intp(cv2.boxPoints(cv2.minAreaRect(best)))
    return (cx, cy), box, best, area


def draw_overlay(frame, centroid, box, contour, area, cfg):
    colour = cfg["colour_bgr"]
    label  = cfg["label"]
    if centroid is not None:
        cx, cy = centroid
        cv2.drawContours(frame, [box], 0, colour, 2)
        cv2.drawContours(frame, [contour], -1, colour, 1)
        cv2.line(frame,   (cx - 15, cy), (cx + 15, cy), colour, 1)
        cv2.line(frame,   (cx, cy - 15), (cx, cy + 15), colour, 1)
        cv2.circle(frame, (cx, cy), 5, colour, -1)
        top_left     = tuple(box[box[:, 1].argmin()])
        text         = f"{label}  cx={cx} cy={cy}"
        (tw, th), _  = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.52, 1)
        cv2.rectangle(frame,
                      (top_left[0], top_left[1] - th - 8),
                      (top_left[0] + tw + 6, top_left[1]),
                      (0, 0, 0), -1)
        cv2.putText(frame, text, (top_left[0] + 3, top_left[1] - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, colour, 1, cv2.LINE_AA)
    else:
        row = list(OBJECTS.keys()).index(next(k for k, v in OBJECTS.items() if v is cfg))
        cv2.putText(frame, f"{label}: not detected", (10, 30 + row * 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, colour, 1, cv2.LINE_AA)


# ── Public API ────────────────────────────────────────────────────────────────
def detect_once(frame) -> dict:
    """Returns {object_name: (cx, cy)} for all objects found in frame."""
    hsv     = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    results = {}
    for name, cfg in OBJECTS.items():
        mask     = clean_mask(build_mask(hsv, cfg), cfg["open_k"], cfg["close_k"])
        centroid, _, _, _ = detect_object(mask, cfg["min_area"])
        if centroid is not None:
            results[name] = centroid
    return results


def capture_stable_detections(cap, num_frames=90, min_detections=30) -> dict:
    """
    Samples num_frames frames and returns averaged centroids for objects
    detected in at least min_detections frames.
    """
    accumulated = {name: [] for name in OBJECTS}
    print(f"[Detection] Sampling {num_frames} frames...")
    for _ in range(num_frames):
        ret, frame = _read(cap)
        if not ret or frame is None or frame.size == 0:
            continue
        for name, centroid in detect_once(frame).items():
            accumulated[name].append(centroid)

    results = {}
    for name, centroids in accumulated.items():
        count = len(centroids)
        if count >= min_detections:
            cx = int(np.mean([c[0] for c in centroids]))
            cy = int(np.mean([c[1] for c in centroids]))
            results[name] = (cx, cy)
            print(f"[Detection] {name:15s} → pixel ({cx:4d}, {cy:4d})  [{count}/{num_frames}]")
        else:
            print(f"[Detection] {name:15s} → NOT reliably detected       [{count}/{num_frames}]")
    return results


# ── Standalone main ───────────────────────────────────────────────────────────
def main(preferred_cam=None):
    exposure_val = EXPOSURE_START
    cap, backend = open_camera(preferred_cam)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,  720)
    cap.set(cv2.CAP_PROP_FPS,            60)

    use_exposure = True
    if use_exposure:
        set_exposure(cap, exposure_val)
    else:
        print("[INFO] MSMF backend — manual exposure not available")

    print(f"[INFO] Camera: "
          f"{int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
          f"{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))} @ "
          f"{cap.get(cv2.CAP_PROP_FPS):.0f} fps")

    cv2.namedWindow("Tracker", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Tracker", 1280, 720)

    fps_time    = time.time()
    frame_count = 0
    fps         = 0.0
    hint        = "Q=quit  S=snap" + ("  E=exp+  Shift+E=exp-" if use_exposure else "")
    print(f"Running.  {hint}")

    while True:
        ret, frame = _read(cap)
        if not ret or frame is None or frame.size == 0:
            print("[WARN] Frame grab failed – retrying...")
            continue

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        for obj_name, cfg in OBJECTS.items():
            mask                         = clean_mask(build_mask(hsv, cfg), cfg["open_k"], cfg["close_k"])
            centroid, box, contour, area = detect_object(mask, cfg["min_area"])
            draw_overlay(frame, centroid, box, contour, area, cfg)

        frame_count += 1
        if frame_count % 30 == 0:
            fps      = 30.0 / (time.time() - fps_time)
            fps_time = time.time()

        exp_str = f"  Exp: {exposure_val}" if use_exposure else ""
        cv2.putText(frame, f"FPS: {fps:.1f}{exp_str}",
                    (10, frame.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        cv2.putText(frame, hint,
                    (frame.shape[1] - 310, frame.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)

        cv2.imshow("Tracker", frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        elif key == ord("s"):
            fname = f"snapshot_{int(time.time())}.png"
            cv2.imwrite(fname, frame)
            print(f"[INFO] Saved {fname}")
        elif use_exposure and key == ord("e"):
            exposure_val = min(0, exposure_val + 1)
            set_exposure(cap, exposure_val)
        elif use_exposure and key == ord("E"):
            exposure_val = max(-11, exposure_val - 1)
            set_exposure(cap, exposure_val)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    cam = int(sys.argv[1]) if len(sys.argv) > 1 else None
    main(cam)