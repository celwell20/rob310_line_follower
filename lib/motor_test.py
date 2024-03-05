from machine import Pin, PWM
import utime
from encoder import Encoder
from motor import Motor
from mbot_defs import *

analog26 = machine.ADC(26)
analog27 = machine.ADC(27)
analog28 = machine.ADC(28)

mot0 = Motor(mot0_pwm_pin, mot0_dir_pin)
enc0 = Encoder(enc0_A_pin, enc0_B_pin)
conv = 1/(20.0 * 78.0)
for i in range(0, 100, 10):
    mot0.set(i/100.0)
    utime.sleep_ms(100)
    enc_initial = enc0.encoderCount
    utime.sleep_ms(400)
    enc_final = enc0.encoderCount
    print(conv*(enc_final - enc_initial)/0.4)
    
    r26 = analog26.read_u16() 
    r27 = analog27.read_u16() 
    r28 = analog28.read_u16()     
    print(f"r26: {r26}, r27: {r27}, r28: {r28}")

mot0.set(0.8)
utime.sleep_ms(100)
mot0.set(0.6)
utime.sleep_ms(100)
mot0.set(0.4)
utime.sleep_ms(100)
mot0.set(0.2)
utime.sleep_ms(100)
mot0.set(0.0)
utime.sleep_ms(100)



for i in range(0, 100, 10):
    mot0.set(-i/100.0)
    utime.sleep_ms(100)
    enc_initial = enc0.encoderCount
    utime.sleep_ms(400)
    enc_final = enc0.encoderCount
    print(conv*(enc_final - enc_initial)/0.4)
    
   
    
mot0.set(-0.8)
utime.sleep_ms(100)
mot0.set(-0.6)
utime.sleep_ms(100)
mot0.set(-0.4)
utime.sleep_ms(100)
mot0.set(-0.2)
utime.sleep_ms(100)
mot0.set(0.0)
utime.sleep_ms(100)