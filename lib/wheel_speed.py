##############################################################
#  wheel_speed.py
#
#  task: create a class that calculates wheel speed in (revolutions / second) from an input of (encoders ticks / second)
#  
#  init args: init_ticks - the encoder tick value read at initialization of the encoders, before the control loop starts
#             init_time - the time value at initialization of encoder, motor, controller objects, etc.
#
#############################################################

import utime

class WheelSpeedCalculator:
    
    def __init__(self, init_ticks, init_time):
        self.CONV = 1/(20.0 * 78.0) # conversion from ticks to revolutions
        
        self.previous_ticks = init_ticks # initialize previous time and ticks to initial time and tick values
        self.previous_time = init_time   # set before the control loop starts
        
    def calculateSpeed(self, present_ticks, present_time):
        
        enc_delta = present_ticks - self.previous_ticks # encoder delta between previous loop and now
        dt = utime.ticks_diff(present_time, self.previous_time) / 1000 # divide by 1000 to convert from milliseconds to seconds
        
        self.previous_ticks = present_ticks # book-keeping
        self.previous_time = present_time
        
        if dt == 0: # set measured wheel speed to zero at start to prevent dividing by zero
            omega = 0
            
        else: # calculate present wheel speed velocities
            omega = self.CONV * enc_delta / dt  # units are rev / s --> divide by 1000 to go from ms to s
            
        return omega
            
            
        