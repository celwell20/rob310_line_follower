import utime
from pid import PID
from line_sensor import LineSensor
from encoder import Encoder
from motor import Motor
from mbot_defs import *

if __name__ == "__main__":
    line_sns = LineSensor(4, 5)
    
    mot0pol = 1
    mot1pol = -1
    
    mot0 = Motor(mot0_pwm_pin, mot0_dir_pin)
    enc0 = Encoder(enc0_A_pin, enc0_B_pin)
    mot0.set(mot0pol*0)
    
    mot1 = Motor(mot1_pwm_pin, mot1_dir_pin)
    enc1 = Encoder(enc1_A_pin, enc1_B_pin)
    mot1.set(mot1pol*0)
    
    while(True):
        byte = line_sns.read() >> 1
        right = (byte >> 4) & (0b111)
        left = byte & (0b111)
        print(byte)
        print(left)
        print(right)
        if (byte & (0b1111111)) == 0:
            #stop
            print("stop")
            mot0.set(mot0pol*0)
            mot1.set(mot1pol*0)
        elif right > 0:
            #turn left
            print("turning left")
            mot0.set(mot0pol*0.0)
            mot1.set(mot1pol*0.5)
        elif left > 0:
            #turn right
            print("turning right")
            mot0.set(mot0pol*0.5)
            mot1.set(mot1pol*0.0)
        utime.sleep_ms(10)
    