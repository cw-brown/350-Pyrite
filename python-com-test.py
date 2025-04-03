import smbus
import struct
import time

I2C_ADDR = 0x08  # Arduino's I2C address
bus = smbus.SMBus(1)  # Use I2C bus 1

def send_floats(floats):
    data = struct.pack('ff', *floats)  # Pack two floats into bytes
    byte_list = list(data)  # Convert bytes to list of integers
    bus.write_i2c_block_data(I2C_ADDR, 0, byte_list)  # Send data

while True:
    send_floats([3.14, 2.71])  # Example data
    print("sent")
    time.sleep(1)
