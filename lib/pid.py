#from machine import Pin, PWM
import utime
from encoder import Encoder
from motor import Motor
from mbot_defs import *
import asyncio

class PID:
    def __init__(self, P=0.0, I=0.0, D=0.0, setpoint=0.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0
    
    def update(self, curr_val, dt):
        error = self.setpoint - curr_val # Error units are rev / s
        P = error*self.Kp
        self.integral += error * dt * self.Ki    # Gain units are [ % duty cycle / (rad / s) ]
        D = ((error-self.prev_error)/dt)*self.Kd
        self.prev_error = error
        return (P + self.integral + D) # PID output is a duty cycle
    
    def set_speed(self, setpoint):
        self.setpoint = setpoint
        self.prev_error = 0
        
    def setP(self, P):
        self.Kp = P
        
    def setI(self, I):
        self.Ki = I
        
    def setD(self, D):
        self.Ki = D

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
    #declare vars, DO NOT CHANGE
    conv = 1/(20.0 * 78.0)
    duty0 = 0.1
    duty1 = 0.1
    dt = 0.1
    
    #depends on motor/encoder set up, change as needed
    mot0pol = -1
    mot1pol = 1
    
    setpoint = -0.8
    #init PID
    Kp0 = 7
    Ki0 = 0.1
    Kd0 = 0
    pid0 = PID(Kp0,Ki0,Kd0,setpoint)
    
    Kp1 = 7
    Ki1 = 0.1
    Kd1 = 0
    pid1 = PID(Kp1,Ki1,Kd1,-setpoint) #negative setpoint since axis flipped
    
    #init motors and encoders
    mot0 = Motor(mot0_pwm_pin, mot0_dir_pin)
    mot0.set(mot0pol*duty0)
    enc0 = Encoder(enc0_A_pin, enc0_B_pin)
    
    mot1 = Motor(mot1_pwm_pin, mot1_dir_pin)
    mot1.set(mot1pol*duty1)
    enc1 = Encoder(enc1_A_pin, enc1_B_pin)
    
    utime.sleep_ms(100)
    
    while True:
        #measure velocity
        enc0_initial = enc0.encoderCount
        utime.sleep_ms(200)
        enc0_final = enc0.encoderCount
        vel0 = conv*(enc0_final - enc0_initial)/0.4
        #update and set duty
        duty0 += pid0.update(vel0,dt)*dt
        if duty0<-1:
            duty0 = -1
        elif duty0>1:
            duty0 = 1
        mot0.set(mot0pol*duty0)
        utime.sleep_ms(100)
        
        #measure velocity
        enc1_initial = enc1.encoderCount
        utime.sleep_ms(200)
        enc1_final = enc1.encoderCount
        vel1 = conv*(enc1_final - enc1_initial)/0.4
        #update and set duty
        duty1 += pid1.update(vel1,dt)*dt
        if duty1<-1:
            duty1 = -1
        elif duty1>1:
            duty1 = 1
        mot1.set(mot1pol*duty1)
        utime.sleep_ms(100)
        
        print("Motor 0 Velocity = ",vel0,"  Motor 1 Velocity = ",-vel1);
        
