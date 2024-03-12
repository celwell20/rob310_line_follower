#################################################################
# line_sensor.py
# 
# task: create a line sensor class by filling in the functions
#       __init__() and LineSensor.read(). LineSensor.read() should
#       return one byte of data that contains the readings from the 7 reflectance
#       sensors and the control bit value. While D0 in the byte is always the control
#       bit value, we do not use this value when interpreting the data; we only use it
#       to turn the reflectance sensor's LEDs on/off.
#
# init args: sda_pin - pin# connected to the MBot control board's I2C SDA pin
#            scl_pin - pin# connected to the MBot control board's I2C SCL pin
#
##################################################################

from machine import I2C, Pin
import time

INPUT_PORT = 0x0
OUTPUT_PORT = 0x1
POLARITY_PORT = 0x2
CONF_PORT = 0x3
DEV_ADDR = 0x18

class LineSensor:
    def __init__(self, sda_pin, scl_pin):
        self.i2c = I2C(0, scl=Pin(scl_pin), sda=Pin(sda_pin))
        # D1-D7 are inputs, D0 is output
        self.i2c.writeto(DEV_ADDR, bytearray([CONF_PORT, 0xFE]))
        # set polarity of inputs
        self.i2c.writeto(DEV_ADDR, bytearray([POLARITY_PORT, 0xFE]))
        # Set D0 off (LEDs off)
        self.i2c.writeto(DEV_ADDR, bytearray([OUTPUT_PORT, 0x00]))
        
    def read(self):
        self.i2c.writeto(DEV_ADDR, bytearray([OUTPUT_PORT, 0x01]))
        time.sleep_ms(20)
        self.i2c.writeto(DEV_ADDR, bytearray([INPUT_PORT]))
        data = self.i2c.readfrom(DEV_ADDR, 1)
        byte = data[0]
        self.i2c.writeto(DEV_ADDR, bytearray([OUTPUT_PORT, 0x00]))
        return byte
        

if __name__ == "__main__":
        line_sns = LineSensor(4, 5)
        while(1):
            byte = line_sns.read()
            for i in range(1,8):
                bit = (byte >> (8-i)) & 0x01
                if bit == 0:
                    print('O', end='')
                else:
                    print('X', end='')
            print('\r', end='')
            time.sleep_ms(10)