#from machine import Pin, PWM
import utime
from encoder import Encoder
from motor import Motor
from speed_smoothing_filter import SmoothingFilter
from mbot_defs import *
import asyncio

# Wheel speed PID parameters
K_P = 0.5
K_I = 0.4
K_D = 0.0001

LEFT_MOTOR_POLARITY = 1
RIGHT_MOTOR_POLARITY = -1

ALPHA = 0.2 # smoothing parameter

CONV = 1/(20.0 * 78.0)   # GEAR RATIO / ENCODER CPR CONVERZAION FACTOR; converts from encoder counts to motor output revs
dt = 0.1


class PID:
    def __init__(self, P=0.0, I=0.0, D=0.0, setpoint=0.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0
    
    def update(self, error, dt):

        #error = self.setpoint - curr_val # Error units are rev / s
        #print(f'error: {error}')
        P = error*self.Kp
        
        self.integral += error * dt * self.Ki    # Gain units are [ % duty cycle / (rad / s) ]
        
        D = ((error-self.prev_error)/dt)*self.Kd
        
        self.prev_error = error
        
        #return P
        return (P + self.integral) # PID output is a duty cycle
    
    def set_speed(self, setpoint):
        self.setpoint = setpoint
        self.prev_error = 0
        
    def setP(self, P):
        self.Kp = P
        
    def setI(self, I):
        self.Ki = I
        
    def setD(self, D):
        self.Ki = D

# async def calc_encoder_delta(encoder):
#     while 

# class wheel_speed_control:
#     def __init__(self, encoder, motor, pid, setpoint):
#         self.encoder = encoder
#         self.motor = motor
#         self.pid = pid
#         self.setpoint = setpoint
#         
#     async def update(self):
#         encoder_t0 = encoder.encoderCount
#         asynco.sleep_ms(200)
#         encoder_t1 = encoder.encoderCount
#         omega_meas = (encoder_t1 - encoder_t0) / 

if __name__ == "__main__":
    
    left_motor = Motor(mot0_pwm_pin, mot0_dir_pin)  # motor object
    left_motor.set(LEFT_MOTOR_POLARITY*0)           # set motor object duty to zero
    left_enc = Encoder(enc0_A_pin, enc0_B_pin)      # encoder
    l_enc_delta = 0                                 # reset encoder value
    
    right_motor = Motor(mot1_pwm_pin, mot1_dir_pin)
    right_motor.set(RIGHT_MOTOR_POLARITY*0)
    right_enc = Encoder(enc1_A_pin, enc1_B_pin)
    r_enc_delta = 0
    
    l_enc_count_prev = left_enc.encoderCount     #  preallocating encoder counts
    r_enc_count_prev = -1*right_enc.encoderCount # flip encoder count to account for flipped motor
    l_enc_count_pres = left_enc.encoderCount
    r_enc_count_pres = -1*right_enc.encoderCount
    
    L_filter = SmoothingFilter(ALPHA)              # filter for smoothing wheel speeds
    R_filter = SmoothingFilter(ALPHA)
    
    L_setpoint = 0.25                                 # wheel speed setpoints
    R_setpoint = 0.25                                 # [rev / s]
    
    L_filter.update(L_setpoint)   # initialization update for wheel speed filter
    R_filter.update(R_setpoint)   # 
    
    L_pid = PID(K_P, K_I, K_D, L_setpoint)     # wheel speed pid objects
    R_pid = PID(K_P, K_I, K_D, R_setpoint)
    
    loop_start_time = utime.ticks_ms()    # get the time in milliseconds
    
    try:
        
        while True:
            utime.sleep_ms(100)
            l_enc_count_pres = left_enc.encoderCount   # present encoder count values
            r_enc_count_pres = -1*right_enc.encoderCount
            #print(f'left encoder: {l_enc_count_pres}, right encoder {r_enc_count_pres}')
            
            #print(f'prev: {l_enc_count_prev}; pres: {l_enc_count_pres}; delta: {l_enc_delta}')
            l_enc_delta = l_enc_count_pres - l_enc_count_prev # encoder deltas between previous loop and now
            r_enc_delta = r_enc_count_pres - r_enc_count_prev
            #print(f'left delta: {l_enc_delta}, right delta {r_enc_delta}')
            l_enc_count_prev = l_enc_count_pres   # book keeping for next iteration of control loop
            r_enc_count_prev = r_enc_count_pres
            
            present_time = utime.ticks_ms()
            
            #print(f'left revs: {CONV*l_enc_delta}, right revs {CONV*r_enc_delta}')
            #print(f"time delta: {utime.ticks_diff(present_time, loop_start_time)}")
            if utime.ticks_diff(present_time, loop_start_time) == 0: # set measured wheel speed to zero at start to prevent dividing by zero
                omega_L = 0
                omega_R = 0
            else: # calculate present wheel speed velocities
                omega_L = CONV * l_enc_delta / (utime.ticks_diff(present_time, loop_start_time) / 1000)  # units are rev / s --> divide by 1000 to go from ms to s
                omega_R = CONV * r_enc_delta / (utime.ticks_diff(present_time, loop_start_time) / 1000)  # units are rev / s
            
            print(f'omega_L: {round(omega_L,2)}, omega_R {round(omega_R,2)}')
            
            loop_start_time = present_time # book keeping for next control loop iteration
            
            #print(f'left setpoint: {L_setpoint}, right setpoint {R_setpoint}')
            
            #L_setpoint_filtered = L_filter.update(L_setpoint)
            #R_setpoint_filtered = R_filter.update(R_setpoint)

            #print(f'left filter: {L_setpoint_filtered}, right filter: {R_setpoint_filtered}')

            L_pid.set_speed(L_setpoint)   # apply the setpoint to the pid controller
            R_pid.set_speed(R_setpoint)

            L_motor_duty = L_pid.update(omega_L, dt) # update the duty cycle output using the PID controller
            R_motor_duty = R_pid.update(omega_R, dt)
            
            #print(f'left duty {L_motor_duty}, right duty {R_motor_duty}')
            
            #L_motor_duty = 0.5
            #R_motor_duty = 0.5
            
            # saturation limit
            if L_motor_duty > 0:
                L_motor_duty = min(0.99, L_motor_duty)
            else:
                L_motor_duty = max(-0.99, L_motor_duty)
            
            if R_motor_duty > 0:
                R_motor_duty = min(0.99, R_motor_duty)
            else:
                R_motor_duty = max(-0.99, R_motor_duty)
            
            #print(f'left duty {L_motor_duty}, right duty {R_motor_duty}')
            
            left_motor.set(LEFT_MOTOR_POLARITY*L_motor_duty) # apply duty cycle to the motors
            right_motor.set(RIGHT_MOTOR_POLARITY*R_motor_duty)
            
    except KeyboardInterrupt:
        left_motor.set(0) # turn off motors
        right_motor.set(0)
        print("Loop interrupted by Ctrl+C")            