"""
pid_stability.py
----------------
PID tuning and stable hover logic.
Imports from the official drone_rc.py — do NOT modify drone_rc.py.

Usage:
    from pid_stability import find_Ku, apply_zn_gains, fly

    find_Ku()                            # step 1: find oscillation point
    apply_zn_gains(Ku=0.XX, Tu=X.XX)    # step 2: apply Z-N gains
    fly(base_thrust=120, duration=60)    # step 3: hover
"""

import time
import drone_rc as rc


<<<<<<< HEAD
# ─── Z-N AUTO TUNE ────────────────────────────────────────────────────────────

def find_Ku(base_thrust=120, p_start=0.005, p_step=0.005, p_max=0.5):
    """
    Slowly increases P gain until sustained oscillation is detected.
    Keep the drone tethered or close to the ground while running this.
    Prints Ku and Tu — paste them into apply_zn_gains().
    """
    rc.set_mode(2)
    rc.set_i_gain(0)
    rc.set_d_gain(0)
    rc.set_pitch(0)
    rc.set_roll(0)
    rc.manual_thrusts(base_thrust, base_thrust, base_thrust, base_thrust)

    P = p_start
    while P <= p_max:
        rc.set_p_gain(P)
        rc.reset_integral()
        print(f"Testing P = {P:.4f} — observing for 4s...")

        crossings = []
        last_sign = None
        t_start = time.time()

        while time.time() - t_start < 4.0:
            error = rc.get_pitch()
            sign = 1 if error > 0 else -1
            if last_sign is not None and sign != last_sign:
                crossings.append(time.time())
            last_sign = sign
            time.sleep(0.05)

        if len(crossings) >= 4:
            periods = [crossings[i+2] - crossings[i] for i in range(len(crossings) - 2)]
            Tu = sum(periods) / len(periods)
            print(f"\n✓ Oscillation detected!")
            print(f"  Ku = {P:.4f}")
            print(f"  Tu = {Tu:.3f} s")
            print(f"\nNext step → apply_zn_gains(Ku={P:.4f}, Tu={Tu:.3f})")
            rc.emergency_stop()
            return P, Tu

        P = round(P + p_step, 4)
        time.sleep(0.3)

    print("No oscillation found — try increasing p_max")
    rc.emergency_stop()
    return None, None


def apply_zn_gains(Ku, Tu):
    """
    Computes and applies PID gains using the Ziegler-Nichols method.
    Ki is clamped to the firmware's maximum safe value of 0.00003.
    Returns (Kp, Ki, Kd) for reference.
    """
    Kp = 0.6   * Ku
    Ki = min(1.2 * Ku / Tu, 0.00003)   # firmware hard limit
    Kd = 0.075 * Ku * Tu

    print(f"Applying Z-N gains:")
    print(f"  Kp = {Kp:.5f}")
    print(f"  Ki = {Ki:.7f}")
    print(f"  Kd = {Kd:.5f}")

    rc.set_p_gain(Kp)
    rc.set_i_gain(Ki)
    rc.set_d_gain(Kd)
    rc.reset_integral()

    return Kp, Ki, Kd


# ─── HOVER LOOP ───────────────────────────────────────────────────────────────

def fly(base_thrust=120, duration=60):
    """
    Hovers using mode 2 (firmware PID handles attitude).
    Call apply_zn_gains() before this.

    base_thrust: 0-250, increase by 5 until drone lifts off
    duration:    seconds to hover (default 60 for judging)
    """
    rc.set_mode(2)
    rc.set_pitch(0)
    rc.set_roll(0)
    rc.manual_thrusts(base_thrust, base_thrust, base_thrust, base_thrust)
    rc.reset_integral()

    print(f"Hovering for {duration}s at base_thrust={base_thrust} — Ctrl+C to emergency stop")
    try:
        t_start = time.time()
        while time.time() - t_start < duration:
            pitch = rc.get_pitch()
            roll  = rc.get_roll()
            elapsed = time.time() - t_start
            print(f"  t={elapsed:5.1f}s  pitch={pitch:+.2f}  roll={roll:+.2f}")
            time.sleep(0.5)     # low poll rate — don't starve the gyro
    except KeyboardInterrupt:
        print("\nManual stop triggered.")
    finally:
        rc.emergency_stop()
        print("Motors off.")

