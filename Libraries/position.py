import numpy as np
import drone_rc as rc

#getting sensor data 
def sensor_input():
    pitch = rc.get_pitch()
    roll = rc.get_roll()
    pitch_rate = rc.get_gyro_pitch()
    roll_rate = rc.get_gyro_roll()
    return pitch, roll, pitch_rate, roll_rate

def calibrate_drone(pitch roll, pitch_rate, roll_rate):
    if pitch_rate != 0:
        rc.set_pitch(-pitch);
    if roll_rate != 0:
        rc.set_roll(-roll)
