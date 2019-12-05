# This has been adapted from the Charmed Labs Pixy code.

from __future__ import print_function
import sys
import time
from rcpy.encoder import encoder2, encoder3
from rcpy.motor import motor2, motor3
import pixy 
from ctypes import *
from pixy import *

# Pixy2 Python SWIG get blocks example #

print("Pixy2 Python SWIG Example -- Get Blocks")

pixy.init ()
pixy.change_prog ("video");

pixy.set_lamp (1, 0);

pixy.change_prog ("color_connected_components");

class Blocks (Structure):
  _fields_ = [ ("m_signature", c_uint),
    ("m_x", c_uint),
    ("m_y", c_uint),
    ("m_width", c_uint),
    ("m_height", c_uint),
    ("m_angle", c_uint),
    ("m_index", c_uint),
    ("m_age", c_uint) ]

blocks = BlockArray(100)
frame = 0

DesiredDistance = 160

kP = .006
kI = .0018
kD = -.00008

upperLimit = .5
lowerLimit = -.5

prev_error = 0
sum_error = 0
PID = 0

try:
    while 1:
        count = pixy.ccc_get_blocks (100, blocks)
    
        if count > 0:
            print('frame %3d:' % (frame))
            frame = frame + 1
            for index in range (0, count):
                print('[BLOCK: SIG=%d X=%3d Y=%3d WIDTH=%3d HEIGHT=%3d]' % (blocks[index].m_signature, blocks[index].m_x, blocks[index].m_y, blocks[index].m_width, blocks[index].m_height))
                if (blocks[index].m_signature == 1):
                    encValue1 = encoder2.get()
                    encValue2 = encoder3.get()
                    
                    turn_error = DesiredDistance - blocks[index].m_x
                    size = ((blocks[index].m_width * blocks[index].m_height)/256)
                    speed = max(min(upperLimit, size), lowerLimit)
                    error = turn_error
                    
                    print (error)
                    PID += (error * kP) + (prev_error * kD)+(sum_error * kI)
                    PID = max(min(upperLimit, PID), lowerLimit)
                    motor2.set(-PID)
                    motor3.set(-PID)
                    
                    print("e1 {} e2 {}".format(encValue1, encValue2))
                    time.sleep(.001)
                    prev_error = error
                    sum_error += error
        else:
            motor2.set(0)
            motor3.set(0)



except:
    raise
finally:
    pixy.change_prog ("video");

    pixy.set_lamp (0, 0);
