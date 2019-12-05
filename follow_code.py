import pixy_pan

drive_upper_limit = .3
drive_lower_limit = -.3
turn_kP = 25  #original constants... 25, 10, 50
turn_kI = 10
turn_kD = 50

drive_kP = 5 #1, 0, .5
drive_kI = 0
drive_kD = .5

print (pixy_pan.Pan_Offset)