from machine import Pin
import utime

class Encoder:
    def __init__(self, pinA, pinB):
        self.encoderCount = 0
        self.A = Pin(pinA, Pin.IN, Pin.PULL_UP)
        self.B = Pin(pinB, Pin.IN, Pin.PULL_UP)
        self.A.irq(self.A_ISR, Pin.IRQ_RISING or Pin.IRQ_FALLING)
        self.B.irq(self.B_ISR, Pin.IRQ_RISING or Pin.IRQ_FALLING)

    def A_ISR(self, pin):
        if self.A.value():
            if self.B.value():
                self.inc()
            else:
                self.dec()
        else:
            if self.B.value():
                self.dec()
            else:
                self.inc()
    
    def B_ISR(self, pin):
        if self.B.value():
            if self.A.value():
                self.dec()
            else:
                self.inc()
        else:
            if self.A.value():
                self.inc()
            else:
                self.dec()
    
    def inc(self):
        self.encoderCount += 1
        
    def dec(self):
        self.encoderCount -= 1

            
if __name__ == "__main__":
    enc0 = Encoder(6, 7)
    enc1 = Encoder(8, 9)
    enc2 = Encoder(10, 11)

    while True:
        print(f"ENC: {enc0.encoderCount} | {enc1.encoderCount} | {enc2.encoderCount}\r")
        utime.sleep_ms(100)
    