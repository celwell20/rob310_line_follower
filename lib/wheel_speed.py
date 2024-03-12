##############################################################
#  wheel_speed.py
#
#  task: create a class that calculates wheel speed in (revolutions / second) from an input of (encoders ticks / second)
#  
#  init args: P - proportional gain value
#             I - intergral gain value
#             D - derivative gain value
#             setpoint - wheel speed setpoint for the controller
# 
#
#############################################################

import utime
import machine
from bangbang import BangBang
from pid import PID
from line_sensor import LineSensor
from encoder import Encoder
from motor import Motor
from speed_smoothing_filter import SmoothingFilter
from mbot_defs import *

class WheelSpeedCalculator:
    
    def __init(self, init_ticks, init_time):
        self.CONV = 1/(20.0 * 78.0)
        
        self.previous_ticks = init_ticks
        self.previous_time = init_time
        
    def calculateSpeed(present_ticks, present_time):
        
        enc_delta = present_ticks - self.previous_ticks # encoder delta between previous loop and now
        dt = utime.ticks_diff(present_time, self.previous_time) / 1000 # divide by 1000 to convert from milliseconds to seconds
        
        self.previous_ticks = present_ticks # book-keeping
        self.previous_time = present_time
        
        if dt == 0: # set measured wheel speed to zero at start to prevent dividing by zero
            omega = 0
            
        else: # calculate present wheel speed velocities
            omega = self.CONV * enc_delta / dt  # units are rev / s --> divide by 1000 to go from ms to s
            
        return omega
            
            
        