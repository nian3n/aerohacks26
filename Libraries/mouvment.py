import drone_rc as rc
import time

def move_forward(baseline):
    rc.manualthrusts(baseline,baseline,baseline + 10, baseline + 10)
    time.sleep(0.15)
    rc.manualthrusts(baseline,baseline,baseline,baseline)
    rc.set_pitch(0)

def move_backwards(baseline)
    rc.manualthrusts(baseline + 10,baseline + 10,baseline, baseline)
    time.sleep(0.15)
    rc.manualthrusts(baseline, baseline, baseline,baseline)
    rc.set_pitch(0)

def move_right(baseline)
    rc.manualthrusts(baseline + 10 ,baseline,baseline + 10, baseline)
    time.sleep(0.15)
    rc.manualthrusts(baseline,baseline,baseline,baseline)
    rc.set_roll(0)

def move_left(baseline)
    rc.manualthrusts(baseline,baseline + 10,baseline, baseline + 10)
    time.sleep(0.15)
    rc.manualthrusts(baseline,baseline,baseline,baseline)
    rc.set_roll(0)

def move_up(baseline):
    rc.manualthrusts(baseline + 10,baseline + 10,baseline + 10, baseline + 10)
    time.sleep(0.15)
    rc.manualthrusts(baseline,baseline,baseline,baseline)
def move_down(baseline):
    rc.manualthrusts(baseline - 10,baseline - 10,baseline - 10, baseline - 10)
    time.sleep(0.15)
    rc.manualthrusts(baseline,baseline,baseline,baseline)
    
