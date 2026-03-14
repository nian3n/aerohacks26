"""
test_pid_stability.py
---------------------
Tests PID stability WITHOUT flying — validates gains, drift, and integral behaviour.
Safe to run on the ground with props off.

Tests:
  1. gain_check    — verifies current gains are in safe ranges
  2. drift_test    — monitors angle drift over 30s to measure gyro quality
  3. integral_test — checks integral windup behaviour
  4. response_test — puts drone in mode 2 and monitors how fast it corrects (PROPS ON)
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "Libraries"))

import drone_rc as rc
import time
import drone_rc as rc

rc.get_conn()
# ─── SAFE GAIN RANGES (from firmware comments + Z-N experience) ───────────────
SAFE_P = (0.001, 0.5)
SAFE_I = (0.0,   0.00003)   # firmware hard limit
SAFE_D = (0.0,   10.0)


# ─── TEST 1: GAIN CHECK ───────────────────────────────────────────────────────

def gain_check(Kp, Ki, Kd):
    """
    Validates that proposed gains are within safe ranges before applying them.
    Call this before apply_zn_gains() to avoid unexpected behaviour.
    """
    print("\n[gain_check] Validating gains...")
    ok = True

    def check(name, val, lo, hi):
        nonlocal ok
        if lo <= val <= hi:
            print(f"  {name} = {val:.6f}  ✓")
        else:
            print(f"  {name} = {val:.6f}  ✗  (safe range: {lo} – {hi})")
            ok = False

    check("Kp", Kp, *SAFE_P)
    check("Ki", Ki, *SAFE_I)
    check("Kd", Kd, *SAFE_D)

    if ok:
        print("[gain_check] All gains OK — safe to apply")
    else:
        print("[gain_check] WARNING: One or more gains out of range — do not fly")
    return ok


# ─── TEST 2: DRIFT TEST ───────────────────────────────────────────────────────

def drift_test(duration=30):
    """
    Monitors pitch and roll drift with drone in mode 0 (motors off).
    Measures how much the angle drifts over `duration` seconds.
    Good result: < 5 degrees of drift over 30 seconds.
    Bad result:  > 10 degrees means gyro needs reset_integral() more often.

    Keep drone PERFECTLY STILL on a flat surface during this test.
    """
    print(f"\n[drift_test] Monitoring angle drift for {duration}s — keep drone still on a flat surface")
    rc.set_mode(0)
    time.sleep(0.5)

    pitch_start = rc.get_pitch()
    roll_start  = rc.get_roll()
    print(f"  Initial — pitch: {pitch_start:+.3f}  roll: {roll_start:+.3f}")

    samples_pitch = []
    samples_roll  = []
    t_start = time.time()

    while time.time() - t_start < duration:
        p = rc.get_pitch()
        r = rc.get_roll()
        samples_pitch.append(p)
        samples_roll.append(r)
        elapsed = time.time() - t_start
        print(f"  t={elapsed:5.1f}s  pitch={p:+.3f}  roll={r:+.3f}", end="\r")
        time.sleep(0.5)

    pitch_end   = rc.get_pitch()
    roll_end    = rc.get_roll()
    pitch_drift = abs(pitch_end - pitch_start)
    roll_drift  = abs(roll_end  - roll_start)

    print(f"\n\n[drift_test] Results after {duration}s:")
    print(f"  Pitch drift: {pitch_drift:.3f} degrees")
    print(f"  Roll drift:  {roll_drift:.3f} degrees")

    if pitch_drift < 5 and roll_drift < 5:
        print("  ✓ Drift acceptable for a 60s hover")
    elif pitch_drift < 10 and roll_drift < 10:
        print("  ⚠ Moderate drift — call reset_integral() every ~20s during flight")
    else:
        print("  ✗ High drift — consider shortening hover duration or adding reset_integral() calls")

    return pitch_drift, roll_drift


# ─── TEST 3: INTEGRAL TEST ────────────────────────────────────────────────────

def integral_test(duration=15):
    """
    Monitors I-term windup in mode 2 with drone on the ground.
    Checks that the integral doesn't explode over time.

    Safe result: I values stay near 0 when drone is flat and level.
    """
    print(f"\n[integral_test] Checking integral windup for {duration}s in mode 2 (motors off baseline)")
    rc.set_mode(2)
    rc.set_pitch(0)
    rc.set_roll(0)
    rc.manual_thrusts(0, 0, 0, 0)   # mode 2 but no thrust — safe on the ground
    rc.reset_integral()

    t_start = time.time()
    while time.time() - t_start < duration:
        i_vals = rc.get_i_values()
        elapsed = time.time() - t_start
        print(f"  t={elapsed:4.1f}s  I_pitch={i_vals[0]:+.4f}  I_roll={i_vals[1]:+.4f}", end="\r")
        time.sleep(0.5)

    i_final = rc.get_i_values()
    print(f"\n\n[integral_test] Final I values: pitch={i_final[0]:+.4f}  roll={i_final[1]:+.4f}")

    if abs(i_final[0]) < 500 and abs(i_final[1]) < 500:
        print("  ✓ Integral stable")
    else:
        print("  ✗ Integral winding up — increase reset frequency or reduce Ki")

    rc.set_mode(0)


# ─── TEST 4: RESPONSE TEST (PROPS ON) ─────────────────────────────────────────

def response_test(base_thrust=120, duration=10):
    """
    !! PROPS ON — run in a safe open area !!
    Puts drone in mode 2 at low thrust and measures how quickly pitch/roll
    return to 0 after a manual tilt disturbance.

    Watch the terminal output — good response = returns to < 1 degree within 1-2s
    """
    print(f"\n[response_test] !! PROPS ON !!")
    print(f"  Base thrust: {base_thrust} | Duration: {duration}s")
    print(f"  Tilt the drone by hand and watch it correct — Ctrl+C to stop")
    input("  Press Enter when ready...")

    rc.set_mode(2)
    rc.set_pitch(0)
    rc.set_roll(0)
    rc.reset_integral()
    rc.manual_thrusts(base_thrust, base_thrust, base_thrust, base_thrust)

    try:
        t_start = time.time()
        while time.time() - t_start < duration:
            p = rc.get_pitch()
            r = rc.get_roll()
            elapsed = time.time() - t_start
            stable = "✓" if abs(p) < 2 and abs(r) < 2 else "~"
            print(f"  {stable} t={elapsed:4.1f}s  pitch={p:+.2f}  roll={r:+.2f}")
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n  Stopped by user")
    finally:
        rc.emergency_stop()
        print("[response_test] Done — motors off")


# ─── ENTRY POINT ──────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("=" * 50)
    print("PID STABILITY TEST")
    print("=" * 50)
    print("1. gain_check    (no drone needed)")
    print("2. drift_test    (motors off, drone flat)")
    print("3. integral_test (motors off, drone flat)")
    print("4. response_test (PROPS ON — tilt test)")
    print("5. Run all safe tests (1+2+3)")
    choice = input("Choose [1-5]: ").strip()

    if choice == "1":
        Kp = float(input("  Enter Kp: "))
        Ki = float(input("  Enter Ki: "))
        Kd = float(input("  Enter Kd: "))
        gain_check(Kp, Ki, Kd)
    elif choice == "2":
        drift_test()
    elif choice == "3":
        integral_test()
    elif choice == "4":
        thrust = int(input("  Enter base_thrust (default 120): ") or "120")
        response_test(base_thrust=thrust)
    elif choice == "5":
        Kp = float(input("  Enter Kp: "))
        Ki = float(input("  Enter Ki: "))
        Kd = float(input("  Enter Kd: "))
        if gain_check(Kp, Ki, Kd):
            drift_test()
            integral_test()
    else:
        print("Invalid choice")
