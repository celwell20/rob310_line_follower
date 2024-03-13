import utime
from line_sensor import LineSensor
from encoder import Encoder
from motor import Motor
from mbot_defs import *

class BangBang:
    def __init__(self, line_sensor):
        self.line_sensor = line_sensor
        
        
    def update(self):
        byte = self.line_sensor.read() >> 1    # clear first bit as it isn't useful
        right_sensors = (byte >> 4) & (0b111)
        left_sensors = byte & (0b111)
        control = 0
        L_setpoint = 0.25
        R_setpoint = 0.25
        
        if (byte & (0b1111111)) == 0:
            L_setpoint = 0
            R_setpoint = 0
            
        elif right_sensors > 0:         # turn the robot to the left if the right sensors are triggered
            L_setpoint += -0.6
            R_setpoint += 0.6
            
        elif left_sensors > 0:          # turn the robot to the right if the left sensors are triggered
            L_setpoint += 0.6
            R_setpoint += -0.6
            
        return L_setpoint, R_setpoint