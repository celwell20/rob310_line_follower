from machine import Pin, PWM
import utime

class Motor:
    def __init__(self, pwm_pin, dir_pin):
        self.dir = Pin(dir_pin, Pin.OUT)
        self.pwm = PWM(Pin(pwm_pin))
        self.pwm.freq(10000)
        self.pwm.duty_u16(0)
        
    def set(self, duty):
        if((duty >= 0.0) and (duty <= 1.0)):
            self.dir.on()
            self.pwm.duty_u16(int(duty * 65535))
        elif((duty < 0.0) and (duty >= -1.0)):
            self.dir.off()
            self.pwm.duty_u16(int(-duty * 65535))
        else:
            print("ERROR: duty out of range")

if __name__ == "__main__":
    mot0 = Motor(2, 14)
    mot0.set(0.5)
    utime.sleep_ms(1000)
    mot0.set(-0.5)
    utime.sleep_ms(1000)
    mot0.set(0.0)
    print("yes")