##############################################################
#  encoder.py
#
#  task: create an encoder class by filling in the functions
#  
#  args: pinA and pinB where the encoder is connected
#
#  note: to set up an IRQ that runs on both a rising
#  and falling edge on a pin do the following:
#   
#    pin.IRQ(ISR, Pin.IRQ_FALLING | Pin.IRQ_RISING)
#
#  where pin is the pin name, and ISR is the handler function 
#
#############################################################

from machine import Pin
import utime

class Encoder:
    def __init__(self, pinA, pinB):
        self.encoderCount = 0   # attribute to tally encoder count
        #self.isr_count = 0
        self.A = Pin(pinA, Pin.IN, Pin.PULL_UP) # pin associated with encoder channel A
        self.B = Pin(pinB, Pin.IN, Pin.PULL_UP) # pin associated with encoder channel B
        self.A.irq(self.A_ISR, Pin.IRQ_RISING | Pin.IRQ_FALLING) # interrupt associated with channel A
        self.B.irq(self.B_ISR, Pin.IRQ_RISING | Pin.IRQ_FALLING) # interrupt associated with channel B

    def A_ISR(self, pin):
        #self.isr_count += 1
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
        #self.isr_count += 1
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
        print(f"ENC: {enc0.encoderCount} | ENC: {enc1.encoderCount}") #{enc1.isr_count}\r")
        utime.sleep_ms(100)
    