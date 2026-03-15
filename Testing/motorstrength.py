import drone_rc as rc
import time

def emergency_stop():
    rc.e()

def main():
    try:
        motor_power = 0
        while motor_power < 100:
            motor_power += 5
            rc.manual_thrusts(motor_power, motor_power, motor_power, motor_power)
            time.sleep(1)
            print(rc.get_pitch())
    except KeyboardInterrupt:
        print("Emergency Stop Initiated")
    finally:
        emergency_stop()

main()