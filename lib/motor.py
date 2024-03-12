#################################################################
# motor.py
# 
# task: create a motor class by filling in the functions
#       Motor.set() should take a duty cycle from -1 to +1
#       and set the control signals to the motor driver
#
# init args: pwm_pin - pin# connected to PWM input of motor driver
#            dir_pin - pin# connected to DIR input of motor driver
#
##################################################################

from machine import Pin, PWM
import utime

class Motor:
    def __init__(self, pwm_pin, dir_pin):
        self.dir = Pin(dir_pin, Pin.OUT) # motor direction control pin
        self.pwm = PWM(Pin(pwm_pin)) # motor pwm input pin
        self.pwm.freq(10000) # set the pwm frequency
        self.pwm.duty_u16(0) # set the pwm duty cycle
        
    def set(self, duty):
        if((duty >= 0.0) and (duty <= 1.0)):
            self.dir.on()
            self.pwm.duty_u16(int(duty * 65535))
        elif((duty < 0.0) and (duty >= -1.0)):
            self.dir.off()
            self.pwm.duty_u16(int(-duty * 65535))
        else:
            print("ERROR: duty out of range")
            self.pwm.duty_u16(int(0))

if __name__ == "__main__":
    mot0 = Motor(2, 14)
    mot0.set(0.5)
    utime.sleep_ms(1000)
    mot0.set(-0.5)
    utime.sleep_ms(1000)
    mot0.set(0.0)
    print("yes")