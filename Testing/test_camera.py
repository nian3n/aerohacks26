"""
test_camera.py
--------------
Tests both cameras independently and validates drone detection.

Camera setup:
  Camera 0 (front view) — sees the cage front-on  → gives drone X (left/right) and Z (height)
  Camera 1 (side view)  — 90 degrees to the right → gives drone Y (depth) and Z (height)

Both cameras point at a tarp-covered 1m x 1m x 1m cube.
Background subtraction works well against a uniform tarp.

Run this BEFORE flying to confirm:
  1. Both cameras open correctly
  2. The drone is being detected (green box appears)
  3. The centroid coordinates look reasonable
  4. No phantom detections when the drone is still
"""

import cv2
import numpy as np
import time


# ─── CAMERA SETUP ─────────────────────────────────────────────────────────────

CAM_FRONT_ID = 0    # front camera  → X, Z
CAM_SIDE_ID  = 1    # side camera   → Y, Z

# These are the pixel coords of the CENTER of the cage in each camera frame.
# Measure these once by looking at the camera feed with the drone at the target
# hover position, then paste the values here.
CAGE_CENTER_FRONT = (320, 240)   # (x, z) in front cam pixels
CAGE_CENTER_SIDE  = (320, 240)   # (y, z) in side cam pixels


# ─── DETECTION HELPERS ────────────────────────────────────────────────────────

def make_detector():
    """
    Returns a background subtractor tuned for tarp + drone detection.
    history=500 gives a stable background model before the drone takes off.
    varThreshold=40 is slightly sensitive — good for a tarp that doesn't move.
    """
    return cv2.createBackgroundSubtractorMOG2(
        history=500,
        varThreshold=40,
        detectShadows=False     # shadows = False speeds up processing
    )


def detect_centroid(frame, back_sub, min_area=250, debug_draw=True):
    """
    Detects the drone in a single frame using background subtraction.

    Returns:
        (cx, cy) centroid in pixels if detected, else None
        annotated frame for display
    """
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    fg_mask = back_sub.apply(blurred)

    # Threshold to remove weak detections (shadows, noise)
    _, thresh = cv2.threshold(fg_mask, 180, 255, cv2.THRESH_BINARY)

    # Morphological open: removes small noise blobs
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    clean = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

    # Optional: dilate slightly to connect broken drone contours
    clean = cv2.dilate(clean, kernel, iterations=1)

    contours, _ = cv2.findContours(clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    large = [c for c in contours if cv2.contourArea(c) > min_area]

    out = frame.copy()
    centroid = None

    if large:
        # Pick the largest contour — the drone
        biggest = max(large, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(biggest)
        cx, cy = x + w // 2, y + h // 2
        centroid = (cx, cy)

        if debug_draw:
            cv2.rectangle(out, (x, y), (x+w, y+h), (0, 0, 200), 2)
            cv2.circle(out, (cx, cy), 5, (0, 255, 0), -1)
            cv2.putText(out, f"({cx}, {cy})", (cx + 8, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    return centroid, out


# ─── INDIVIDUAL CAMERA TESTS ──────────────────────────────────────────────────

def test_single_camera(cam_id, label, warmup_frames=60):
    """
    Opens one camera, warms up the background model, then shows live detection.
    Press Q to quit, S to print the current centroid as a suggested cage center.
    """
    print(f"\n[{label}] Opening camera {cam_id}...")
    cap = cv2.VideoCapture(cam_id)

    if not cap.isOpened():
        print(f"[{label}] ERROR: Could not open camera {cam_id}")
        return False

    back_sub = make_detector()
    print(f"[{label}] Warming up background model ({warmup_frames} frames)...")

    for i in range(warmup_frames):
        ret, frame = cap.read()
        if ret:
            back_sub.apply(cv2.GaussianBlur(frame, (5, 5), 0))
        if i % 20 == 0:
            print(f"  {i}/{warmup_frames}")

    print(f"[{label}] Background ready. Showing live feed — press Q to quit, S to sample centroid.")
    detection_count = 0
    total_frames    = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            print(f"[{label}] Camera read failed.")
            break

        centroid, annotated = detect_centroid(frame, back_sub)
        total_frames += 1
        if centroid:
            detection_count += 1

        # Overlay stats
        rate = detection_count / total_frames * 100 if total_frames else 0
        status = f"Detected: {centroid}  |  Detection rate: {rate:.0f}%"
        cv2.putText(annotated, status, (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        cv2.putText(annotated, f"Camera: {label}", (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        cv2.imshow(f"Camera {cam_id} — {label}", annotated)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s') and centroid:
            print(f"\n[{label}] Suggested cage center: {centroid}")
            print(f"  Paste into main.py as CAGE_CENTER_{label.upper().replace(' ', '_')}")

    cap.release()
    cv2.destroyAllWindows()

    print(f"\n[{label}] Results: detected in {detection_count}/{total_frames} frames ({rate:.0f}%)")
    if rate < 50:
        print(f"  WARNING: Detection rate below 50% — check lighting or min_area threshold")
    else:
        print(f"  OK: Detection looks reliable")
    return True


# ─── DUAL CAMERA TEST ─────────────────────────────────────────────────────────

def test_dual_cameras(warmup_frames=60):
    """
    Runs both cameras simultaneously and shows the 3D position estimate.
    Camera 0 gives (x, z_front), Camera 1 gives (y, z_side).
    z is averaged between both cameras for a better height estimate.
    """
    print("\n[DUAL] Opening both cameras...")
    cap0 = cv2.VideoCapture(CAM_FRONT_ID)
    cap1 = cv2.VideoCapture(CAM_SIDE_ID)

    if not cap0.isOpened():
        print("ERROR: Could not open front camera (id=0)")
        return
    if not cap1.isOpened():
        print("ERROR: Could not open side camera (id=1)")
        return

    sub0 = make_detector()
    sub1 = make_detector()

    print(f"[DUAL] Warming up ({warmup_frames} frames)...")
    for _ in range(warmup_frames):
        ret0, f0 = cap0.read()
        ret1, f1 = cap1.read()
        if ret0: sub0.apply(cv2.GaussianBlur(f0, (5, 5), 0))
        if ret1: sub1.apply(cv2.GaussianBlur(f1, (5, 5), 0))

    print("[DUAL] Live — press Q to quit")

    while True:
        ret0, frame0 = cap0.read()
        ret1, frame1 = cap1.read()

        if not ret0 or not ret1:
            print("[DUAL] Camera read failed")
            break

        c0, ann0 = detect_centroid(frame0, sub0)   # front: (drone_x, drone_z)
        c1, ann1 = detect_centroid(frame1, sub1)   # side:  (drone_y, drone_z)

        # Compute 3D position estimate
        pos_x = c0[0] if c0 else None
        pos_y = c1[0] if c1 else None
        pos_z = None
        if c0 and c1:
            pos_z = (c0[1] + c1[1]) // 2   # average height from both cameras
        elif c0:
            pos_z = c0[1]
        elif c1:
            pos_z = c1[1]

        # Draw cage center crosshair on each view
        cx0, cz0 = CAGE_CENTER_FRONT
        cx1, cz1 = CAGE_CENTER_SIDE
        cv2.drawMarker(ann0, (cx0, cz0), (255, 0, 255), cv2.MARKER_CROSS, 20, 2)
        cv2.drawMarker(ann1, (cx1, cz1), (255, 0, 255), cv2.MARKER_CROSS, 20, 2)

        # Overlay 3D position
        pos_str = f"3D pos — X:{pos_x}  Y:{pos_y}  Z:{pos_z}"
        cv2.putText(ann0, pos_str, (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
        cv2.putText(ann1, pos_str, (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

        cv2.imshow("Front Camera (X, Z)", ann0)
        cv2.imshow("Side Camera  (Y, Z)", ann1)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap0.release()
    cap1.release()
    cv2.destroyAllWindows()


# ─── ENTRY POINT ──────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("=" * 50)
    print("CAMERA TEST")
    print("=" * 50)
    print("1. Test front camera only (cam 0)")
    print("2. Test side camera only  (cam 1)")
    print("3. Test both cameras together (full 3D)")
    choice = input("Choose [1/2/3]: ").strip()

    if choice == "1":
        test_single_camera(CAM_FRONT_ID, "Front")
    elif choice == "2":
        test_single_camera(CAM_SIDE_ID, "Side")
    elif choice == "3":
        test_dual_cameras()
    else:
        print("Invalid choice")
