import drone_rc as rc

rc.manual_thrusts(10, 0, 0, 0)
print("Spun motor A")
time.sleep(1)
rc.manual_thrusts(0, 10, 0, 0)
print("Spun motor B")
time.sleep(1)
rc.manual_thrusts(0,0,10,0)
print("Spun motor C")
time.sleep(1)
rc.manual_thrusts(0,0,0,10)
print("spun motor D")
time.sleep(1)
