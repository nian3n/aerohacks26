"""
main.py
-------
Vision-based autonomous hover.

Architecture:
  - Camera thread: reads both cameras, publishes 3D position to shared state
  - Main loop:     mode 2 the whole time, uses get_pitch/get_roll for attitude
                   and camera position to nudge set_pitch/set_roll targets

Camera layout (two cameras 90 degrees apart, pointing at tarp-covered 1m cube):
  Camera 0 (front view) → drone X (left/right) and Z (height)
  Camera 1 (side view)  → drone Y (depth)      and Z (height)

Mode flow:
  MODE 0 → MODE 1 (takeoff ramp) → MODE 2 (full hover, stays here for 60s)

Usage:
  python main.py
  Ctrl+C → emergency stop
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "Libraries"))

import drone_rc as rc
import time
import threading
import cv2
import numpy as np


# ═══════════════════════════════════════════════════════════════════════════════
# CONFIG — tune these values before flying
# ═══════════════════════════════════════════════════════════════════════════════

# PID attitude gains — from your Z-N auto-tune (apply_zn_gains printed these)
ATTITUDE_KP = 0.02
ATTITUDE_KI = 0.00000
ATTITUDE_KD = 3.0

# Base thrust: increase by 5 until drone lifts, then back off 5
BASE_THRUST  = 50

# Position PID gains — controls how aggressively drone corrects to cage center
# Start with just Kp_pos, keep Ki/Kd at 0 until Kp is stable
POS_KP = 0.03       # how hard to lean toward center (degrees per pixel of error)
POS_KI = 0.0        # leave at 0 until position is stable
POS_KD = 0.05       # damping — helps prevent overshooting center

# Max lean angle the position loop is allowed to command (degrees)
# Keep small — large leans = drone drifts fast = hard to recover
MAX_LEAN_DEG = 4.0

# Target position in pixels — measure from your camera feed (run test_camera.py → press S)
TARGET_X = 320      # front camera: horizontal center
TARGET_Y = 320      # side camera:  horizontal center
TARGET_Z = 240      # both cameras: vertical target (higher pixel = lower drone for most mounts)

# Takeoff config
TAKEOFF_THRUST_START = 80
TAKEOFF_THRUST_END   = BASE_THRUST
TAKEOFF_RAMP_TIME    = 2.0      # seconds to ramp from 0 to base thrust

# Loop rates
POSITION_LOOP_HZ  = 10          # how often Python sends position corrections
INTEGRAL_RESET_HZ = 20          # how often firmware integral gets reset (fights drift)

# Background subtractor warmup
CAM_WARMUP_FRAMES = 120         # frames before detection is trusted


# ═══════════════════════════════════════════════════════════════════════════════
# SHARED STATE (written by camera thread, read by main loop)
# ═══════════════════════════════════════════════════════════════════════════════

drone_pos  = {"x": None, "y": None, "z": None}
pos_lock   = threading.Lock()
cam_running = True


# ═══════════════════════════════════════════════════════════════════════════════
# CAMERA THREAD
# ═══════════════════════════════════════════════════════════════════════════════

def make_detector():
    return cv2.createBackgroundSubtractorMOG2(
        history=500,
        varThreshold=40,
        detectShadows=False
    )


def get_centroid(frame, back_sub, min_area=250):
    """
    Returns (cx, cy) of the largest detected moving object, or None.
    """
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    fg      = back_sub.apply(blurred)
    _, thresh = cv2.threshold(fg, 180, 255, cv2.THRESH_BINARY)
    kernel  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    clean   = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    clean   = cv2.dilate(clean, kernel, iterations=1)
    contours, _ = cv2.findContours(clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    large = [c for c in contours if cv2.contourArea(c) > min_area]
    if not large:
        return None
    biggest = max(large, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(biggest)
    return (x + w // 2, y + h // 2)


def camera_thread():
    """
    Reads both cameras and updates drone_pos continuously.
    Runs as a daemon thread — killed automatically when main exits.
    """
    global cam_running

    cap0 = cv2.VideoCapture(0)   # front camera
    cap1 = cv2.VideoCapture(1)   # side camera
    sub0 = make_detector()
    sub1 = make_detector()

    if not cap0.isOpened() or not cap1.isOpened():
        print("[camera] ERROR: Could not open one or both cameras")
        cam_running = False
        return

    # Warmup background model before drone takes off
    print(f"[camera] Warming up background model ({CAM_WARMUP_FRAMES} frames)...")
    for i in range(CAM_WARMUP_FRAMES):
        r0, f0 = cap0.read()
        r1, f1 = cap1.read()
        if r0: sub0.apply(cv2.GaussianBlur(f0, (5, 5), 0))
        if r1: sub1.apply(cv2.GaussianBlur(f1, (5, 5), 0))
    print("[camera] Background ready")

    while cam_running:
        r0, f0 = cap0.read()
        r1, f1 = cap1.read()

        c0 = get_centroid(f0, sub0) if r0 else None   # front: (x, z)
        c1 = get_centroid(f1, sub1) if r1 else None   # side:  (y, z)

        with pos_lock:
            drone_pos["x"] = c0[0] if c0 else drone_pos["x"]   # left/right
            drone_pos["y"] = c1[0] if c1 else drone_pos["y"]   # depth
            # height: average from both cameras when available
            if c0 and c1:
                drone_pos["z"] = (c0[1] + c1[1]) // 2
            elif c0:
                drone_pos["z"] = c0[1]
            elif c1:
                drone_pos["z"] = c1[1]

        time.sleep(1 / 30)     # 30fps max — camera doesn't need to be faster

    cap0.release()
    cap1.release()


# ═══════════════════════════════════════════════════════════════════════════════
# POSITION PID
# ═══════════════════════════════════════════════════════════════════════════════

_prev_err_x  = 0.0
_prev_err_y  = 0.0
_integral_x  = 0.0
_integral_y  = 0.0
_pos_dt      = 1.0 / POSITION_LOOP_HZ


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


def compute_position_correction():
    """
    Reads drone_pos, computes pitch/roll setpoint corrections to return to center.
    Returns (lean_pitch, lean_roll, thrust_correction) or None if no detection.
    """
    global _prev_err_x, _prev_err_y, _integral_x, _integral_y

    with pos_lock:
        x = drone_pos["x"]
        y = drone_pos["y"]
        z = drone_pos["z"]

    if x is None or y is None:
        return None     # no detection — hold current setpoints

    # Pixel errors (positive = drone is to the left/back/low of target)
    err_x = TARGET_X - x
    err_y = TARGET_Y - y
    err_z = TARGET_Z - z

    # X → controls roll  (lean left/right to move toward center X)
    _integral_x += err_x * _pos_dt
    d_x = (err_x - _prev_err_x) / _pos_dt
    out_x = POS_KP * err_x + POS_KI * _integral_x + POS_KD * d_x
    _prev_err_x = err_x

    # Y → controls pitch (lean forward/back to move toward center Y)
    _integral_y += err_y * _pos_dt
    d_y = (err_y - _prev_err_y) / _pos_dt
    out_y = POS_KP * err_y + POS_KI * _integral_y + POS_KD * d_y
    _prev_err_y = err_y

    # Z → thrust correction (simple proportional only)
    thrust_corr = int(clamp(err_z * 0.1, -20, 20))

    lean_pitch = clamp(out_y, -MAX_LEAN_DEG, MAX_LEAN_DEG)
    lean_roll  = clamp(out_x, -MAX_LEAN_DEG, MAX_LEAN_DEG)

    return lean_pitch, lean_roll, thrust_corr


# ═══════════════════════════════════════════════════════════════════════════════
# FLIGHT PHASES
# ═══════════════════════════════════════════════════════════════════════════════

def phase_takeoff():
    """
    MODE 1: Ramps thrust from 0 to BASE_THRUST over TAKEOFF_RAMP_TIME seconds.
    No PID during this phase — just get off the ground cleanly.
    """
    print("[takeoff] Starting — mode 1, ramping thrust...")
    rc.set_mode(1)

    steps = 200000
    for i in range(steps + 1):
        thrust = int(TAKEOFF_THRUST_START + (BASE_THRUST - TAKEOFF_THRUST_START) * i / steps)
        rc.manual_thrusts(thrust, thrust, thrust, thrust)
        rc.set_yaw(25)
        time.sleep(TAKEOFF_RAMP_TIME / steps)

    print(f"[takeoff] Reached base thrust {BASE_THRUST} — handing off to mode 2")


def phase_hover(duration=60):
    """
    MODE 2: Firmware handles attitude, Python handles position via set_pitch/set_roll.
    Uses get_pitch/get_roll to monitor attitude and camera for position correction.
    Runs for `duration` seconds (default 60 for judging).
    """
    print("[hover] Switching to mode 2 — firmware PID active")

    # Apply attitude gains and enter mode 2
    rc.set_p_gain(ATTITUDE_KP)
    rc.set_i_gain(ATTITUDE_KI)
    rc.set_d_gain(ATTITUDE_KD)
    rc.set_pitch(0)
    rc.set_roll(0)
    rc.reset_integral()
    rc.set_mode(2)
    rc.manual_thrusts(BASE_THRUST, BASE_THRUST, BASE_THRUST, BASE_THRUST)

    current_thrust   = BASE_THRUST
    loop_dt          = _pos_dt
    last_reset_time  = time.time()
    reset_interval   = 1.0 / INTEGRAL_RESET_HZ

    print(f"[hover] Hovering for {duration}s — Ctrl+C to emergency stop")

    t_start = time.time()
    while time.time() - t_start < duration:
        loop_start = time.time()

        # Read attitude from firmware (ground truth for stability)
        pitch = rc.get_pitch()
        roll  = rc.get_roll()

        # Compute position correction from cameras
        correction = compute_position_correction()

        if correction:
            lean_pitch, lean_roll, thrust_corr = correction
            current_thrust = clamp(BASE_THRUST + thrust_corr, 80, 180)
            rc.set_pitch(lean_pitch)
            rc.set_roll(lean_roll)
            rc.manual_thrusts(current_thrust, current_thrust,
                              current_thrust, current_thrust)

        # Periodically reset integral to fight gyro drift
        if time.time() - last_reset_time > reset_interval:
            rc.reset_integral()
            last_reset_time = time.time()

        # Log
        elapsed = time.time() - t_start
        with pos_lock:
            px, py, pz = drone_pos["x"], drone_pos["y"], drone_pos["z"]
        print(f"  t={elapsed:5.1f}s | attitude pitch={pitch:+.2f} roll={roll:+.2f} "
              f"| pos X={px} Y={py} Z={pz} "
              f"| thrust={current_thrust}")

        # Pace the loop
        elapsed_loop = time.time() - loop_start
        time.sleep(max(0, loop_dt - elapsed_loop))


# ═══════════════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    global cam_running

    print("=" * 60)
    print("VISION-BASED AUTONOMOUS HOVER")
    print("=" * 60)

    # Start camera thread first so background model is ready by takeoff
    # cam_thread = threading.Thread(target=camera_thread, daemon=True)
    # cam_thread.start()

    # Wait for camera warmup to finish
    warmup_wait = (CAM_WARMUP_FRAMES / 30) + 1.0
    print(f"[main] Waiting {warmup_wait:.0f}s for camera warmup...")
    time.sleep(warmup_wait)
    cam_running = True

    if not cam_running:
        print("[main] Camera failed — aborting")
        return

    input("[main] Cameras ready. Place drone on launch pad, then press Enter to take off...")

    try:
        rc.recalibrate()
        # Phase 1: Takeoff (mode 1)
        phase_takeoff()
        print(rc.get_mode())
        # Brief pause to stabilize before handing off
        time.sleep(0.5)

        # Phase 2: Hover (mode 2, full 60s)
        #phase_hover(duration=60)

        print("[main] Hover complete — landing")

    except KeyboardInterrupt:
        print("\n[main] Emergency stop triggered by user")

    finally:
        cam_running = False
        rc.emergency_stop()
        print("[main] Motors off — done")


if __name__ == "__main__":
    main()
