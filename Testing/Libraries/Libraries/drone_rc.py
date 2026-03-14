"""
The following functions can be used to communicate with the drone

general advice:
do not have constant high-bandwidth communications with the drone,
because processing time doing wifi stuff is processing time not spent updating the gyroscope,
which will lead to increased drift
"""



import socket
import socket
import time

# ---- connection ----
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(("192.168.4.1", 8080))

def msg(tx):
    s.sendall((tx + "\n").encode("ASCII"))
    rx = ""
    while not rx.endswith("\n"):
        rx += s.recv(1).decode("ASCII")
    return rx[:-1]

def emergency_stop():   msg("mode0")
def set_mode(m):        msg("mode" + str(m))
def manual_thrusts(A,B,C,D): msg("manT\n" + str(A) + "," + str(B) + "," + str(C) + "," + str(D) + "\n")
def get_pitch():        return float(msg("angX")) / 16
def get_roll():         return float(msg("angY")) / 16
def set_pitch(r):       msg("gx" + str(r))
def set_roll(r):        msg("gy" + str(r))
def set_p_gain(p):      msg("gainP" + str(p))
def set_i_gain(i):      msg("gainI" + str(i))
def set_d_gain(d):      msg("gainD" + str(d))
def reset_integral():   msg("irst")

#Using Z-N algorith to determine optimum PID gain
def find_Ku(base_thrust=120, p_start=0.005, p_step=0.005, p_max=0.5):
    """Slowly increases P until oscillation is detected. Watch the drone."""
    set_mode(2)
    set_i_gain(0)
    set_d_gain(0)
    set_pitch(0)
    set_roll(0)
    manual_thrusts(base_thrust, base_thrust, base_thrust, base_thrust)

    P = p_start
    while P <= p_max:
        set_p_gain(P)
        reset_integral()
        print(f"P = {P:.4f} — watching for oscillation for 4s...")

        crossings = []
        last_sign = None
        start = time.time()
        while time.time() - start < 4.0:
            error = get_pitch()
            sign = 1 if error > 0 else -1
            if last_sign is not None and sign != last_sign:
                crossings.append(time.time())
            last_sign = sign
            time.sleep(0.05)

        if len(crossings) >= 4:
            periods = [crossings[i+2] - crossings[i] for i in range(len(crossings)-2)]
            Tu = sum(periods) / len(periods)
            print(f"\n✓ Oscillation detected!")
            print(f"  Ku = {P:.4f}")
            print(f"  Tu = {Tu:.3f} s")
            print(f"\nNow call: apply_zn_gains(Ku={P:.4f}, Tu={Tu:.3f})")
            emergency_stop()
            return P, Tu

        P = round(P + p_step, 4)
        time.sleep(0.3)

    print("No oscillation found — increase p_max")
    emergency_stop()


def apply_zn_gains(Ku, Tu):
    """Paste Ku and Tu from find_Ku() here."""
    Kp = 0.6   * Ku
    Ki = min(1.2 * Ku / Tu, 0.00003)  # clamped to firmware limit
    Kd = 0.075 * Ku * Tu

    print(f"Applying gains — Kp={Kp:.5f}  Ki={Ki:.7f}  Kd={Kd:.5f}")
    set_p_gain(Kp)
    set_i_gain(Ki)
    set_d_gain(Kd)
    reset_integral()
    return Kp, Ki, Kd


def fly(base_thrust=120, duration=10):
    """Hovers for `duration` seconds using mode 2 (firmware PID)."""
    set_mode(2)
    set_pitch(0)
    set_roll(0)
    manual_thrusts(base_thrust, base_thrust, base_thrust, base_thrust)
    reset_integral()

    print(f"Flying for {duration}s — Ctrl+C to emergency stop")
    try:
        start = time.time()
        while time.time() - start < duration:
            p = get_pitch()
            r = get_roll()
            print(f"  pitch={p:+.2f}  roll={r:+.2f}")
            time.sleep(0.5)  # low poll rate to not starve gyro
    except KeyboardInterrupt:
        print("Manual stop!")
    finally:
        emergency_stop()
        print("Stopped.")

