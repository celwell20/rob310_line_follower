import utime
from bangbang import BangBang
from pid import PID
from line_sensor import LineSensor
from encoder import Encoder
from motor import Motor
from speed_smoothing_filter import SmoothingFilter
from mbot_defs import *

# Values for determining desired control state
STOP_CTRL_INPUT = 0
LEFT_CTRL_INPUT = -1
RIGHT_CTRL_INPUT = 1

# Wheel speed PID parameters
K_P = 7
K_I = 0.1
K_D = 0

LEFT_MOTOR_POLARITY = -1
RIGHT_MOTOR_POLARITY = -1

ALPHA = 0.2 # smoothing parameter

CONV = 1/(20.0 * 78.0)   # GEAR RATIO / ENCODER CPR CONVERZAION FACTOR; converts from encoder counts to motor output revs
dt = 0.1

if __name__ == "__main__":

    left_motor = Motor(mot0_pwm_pin, mot0_dir_pin)  # motor object
    left_motor.set(LEFT_MOTOR_POLARITY*0)           # set motor object duty to zero
    left_enc = Encoder(enc0_A_pin, enc0_B_pin)      # encoder
    l_enc_delta = 0                                 # reset encoder value
    
    right_motor = Motor(mot1_pwm_pin, mot1_dir_pin)
    right_motor.set(RIGHT_MOTOR_POLARITY*0)
    right_enc = Encoder(enc1_A_pin, enc1_B_pin)
    r_enc_delta = 0
    
    l_enc_count_prev = left_enc.encoderCount()     #  preallocating encoder counts
    r_enc_count_prev = right_enc.encoderCount()
    l_enc_count_pres = left_enc.encoderCount()
    r_enc_count_pres = right_enc.encoderCount()
    
    line_sensor = LineSensor(4,5)                  # line sensor object
    bb_controller = BangBang(line_sensor)          # line follower controller
    
    L_filter = SmoothingFilter(ALPHA)              # filter for smoothing wheel speeds
    R_filter = SmoothingFilter(ALPHA)
    
    L_setpoint = 0                                 # wheel speed setpoints
    R_setpoint = 0
    
    L_filter.update(L_setpoint)   # initialization update for wheel speed filter
    R_filter.update(R_setpoint)   # 
    
    L_pid = PID(K_P, K_I, K_D, L_setpoint, left_motor, LEFT_MOTOR_POLARITY)     # wheel speed pid objects
    R_pid = PID(K_P, K_I, K_D, R_setpoint, right_motor, RIGHT_MOTOR_POLARITY)
    
    loop_start_time = utime.time()
    while True:
        
        l_enc_count_pres = left_enc.encoderCount()   # present encoder count values
        r_enc_count_pres = right_enc.encoderCount()
        
        l_enc_delta = l_enc_count_pres - l_enc_count_prev # encoder deltas between previous loop and now
        r_enc_delta = l_enc_count_pres - r_enc_count_prev
        
        l_enc_count_prev = l_enc_count_pres   # book keeping for next iteration of control loop
        r_enc_count_prev = r_enc_count_pres
        
        present_time = utime.time()
        # calculate present wheel speed velocities
        omega_L = CONV * l_enc_delta / (present_time - loop_start_time)  # units are rev / s
        omega_R = CONV * r_enc_delta / (present_time - loop_start_time)  # units are rev / s
        
        loop_start_time = present_time # book keeping for next control loop iteration
        
        bb_output = bb_controller.update()      # tells us whether to turn left or right
        
        if bb_output == STOP_CTRL_INPUT:   # stop the mbot if it doesn't see any line
            L_setpoint = 0
            R_setpoint = 0
        
        elif bb_output == LEFT_CTRL_INPUT: # turn mbot to the left
            L_setpoint = -0.5
            R_setpoint = 1
            
        elif bb_output == RIGHT_CTRL_INPUT: # turn mbot to the right
            L_setpoint = 1
            R_setpoint = -0.5
            
        L_setpoint_filter = 
        R_setpoint_filter = 
#         (1 - self.alpha) * self.filtered_speed + self.alpha * current_speed

        L_pid.set_speed(L_setpoint)   # apply the setpoint to the 
        R_pid.set_speed(R_setpoint)

        L_motor_duty = L_pid.update(omega_L, dt) # update the setpoint using the PID controller
        R_motor_duty = R_pid.update(omega_R, dt)
        
        left_motor.set(LEFT_MOTOR_POLARITY*L_motor_duty) # apply duty cycle to the motors
        right_motor.set(RIGHT_MOTOR_POLARITY*R_motor_duty)
        
        
        
    
    