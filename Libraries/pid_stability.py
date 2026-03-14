import numpy as np
import drone_rc as rc


#error is our setpoint - mesured angle
pitch = get_pitch()
roll = get_roll()

setpoint_pitch = 0
setpoint_roll = 0

error_pitch = setpoint_pitch - pitch
error_roll = setpoint_roll - roll

integral = setpoint_
