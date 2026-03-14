import drone_rc as rc

rc.manual_thrusts(10, 0, 0, 0)
print("Spun motor A")
rc.manual_thrusts(0, 10, 0, 0)
print("Spun motor B")
rc.manual_thrusts(0,0,10,0)
print("Spun motor C")
rc.manual_thrusts(0,0,0,10)
print("spun motor D")
