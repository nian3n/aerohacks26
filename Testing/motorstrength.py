import drone_rc as rc
import time
import threading

def emergency_stop():
    input("press enter to trigger emergency stop...\n")
    rc.e()

def main():
    stop_thread = threading.Thread(target=listen_for_stop, daemon=True)
    stop_thread.start()

    motor_power = 0
    while motor_power < 100:
        motor_power = motor_power + 5
        rc.increment_thrusts(motor_power, motor_power, motor_power, motor_power)
        time.sleep(1)

    emergency_stop()

main()
