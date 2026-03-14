import numpy as np
import drone_rc as rc


#error is our setpoint - mesured angle
pitch = get_pitch()
roll = get_roll()
gyro_pitch = 0
gyro_roll = 0
setpoint_pitch = 0
setpoint_roll = 0

