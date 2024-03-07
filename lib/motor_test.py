from machine import Pin, PWM
import utime
from encoder import Encoder
from motor import Motor
from mbot_defs import *

analog26 = machine.ADC(26)
analog27 = machine.ADC(27)
analog28 = machine.ADC(28)

mot0 = Motor(mot1_pwm_pin, mot1_dir_pin)
enc0 = Encoder(enc1_A_pin, enc1_B_pin)
CONV = 1/(20.0 * 78.0)

try:
    l_enc_count_prev = 0
    mot0.set(-0.25)
    loop_start_time = utime.ticks_ms()    # get the time in milliseconds
    while True:
        
        utime.sleep_ms(50)
        l_enc_count_pres = enc0.encoderCount   # present encoder count values
            #print(f'left encoder: {l_enc_count_pres}, right encoder {r_enc_count_pres}')
            
            #print(f'prev: {l_enc_count_prev}; pres: {l_enc_count_pres}; delta: {l_enc_delta}')
        l_enc_delta = l_enc_count_pres - l_enc_count_prev # encoder deltas between previous loop and now
        #print(f'left delta: {l_enc_delta}') # right delta {r_enc_delta}')
        l_enc_count_prev = l_enc_count_pres   # book keeping for next iteration of control loop
            
        present_time = utime.ticks_ms()
        dt = utime.ticks_diff(present_time, loop_start_time)
            #print(f'left revs: {CONV*l_enc_delta}, right revs {CONV*r_enc_delta}')
            #print(f"time delta: {utime.ticks_diff(present_time, loop_start_time)}")
        if dt == 0: # set measured wheel speed to zero at start to prevent dividing by zero
            omega_L = 0
        else: # calculate present wheel speed velocities
            omega_L = CONV * l_enc_delta / (dt / 1000)  # units are rev / s --> divide by 1000 to go from ms to s
        loop_start_time = present_time # book keeping for next control loop iteration
                
        print(f'omega_L: {omega_L}') # omega_R {round(omega_R,2)}')
        
    
    
    
except KeyboardInterrupt:
    mot0.set(0) # turn off motors
    print("Loop interrupted by Ctrl+C")   
# for i in range(0, 100, 10):
#     mot0.set(i/100.0)
#     utime.sleep_ms(100)
#     enc_initial = enc0.encoderCount
#     utime.sleep_ms(400)
#     enc_final = enc0.encoderCount
#     print(conv*(enc_final - enc_initial)/0.4)
#     
#     r26 = analog26.read_u16() 
#     r27 = analog27.read_u16() 
#     r28 = analog28.read_u16()     
#     print(f"r26: {r26}, r27: {r27}, r28: {r28}")
# 
# mot0.set(0.8)
# utime.sleep_ms(100)
# mot0.set(0.6)
# utime.sleep_ms(100)
# mot0.set(0.4)
# utime.sleep_ms(100)
# mot0.set(0.2)
# utime.sleep_ms(100)
# mot0.set(0.0)
# utime.sleep_ms(100)
# 
# 
# 
# for i in range(0, 100, 10):
#     mot0.set(-i/100.0)
#     utime.sleep_ms(100)
#     enc_initial = enc0.encoderCount
#     utime.sleep_ms(400)
#     enc_final = enc0.encoderCount
#     print(conv*(enc_final - enc_initial)/0.4)
#     
#    
#     
# mot0.set(-0.8)
# utime.sleep_ms(100)
# mot0.set(-0.6)
# utime.sleep_ms(100)
# mot0.set(-0.4)
# utime.sleep_ms(100)
# mot0.set(-0.2)
# utime.sleep_ms(100)
# mot0.set(0.0)
# utime.sleep_ms(100)