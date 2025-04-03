import smbus
import time
import struct  # For packing floats into binary format

# I2C bus number (on Raspberry Pi, typically 1)
bus = smbus.SMBus(1)

# Arduino slave address
slave_address = 0x08

def write_vector(vector):
    # Convert the floats into a byte array (4 bytes each)
    data = struct.pack('ff', vector[0], vector[1])
    
    # Send the packed float data (8 bytes total)
    bus.write_i2c_block_data(slave_address, 0, list(data))

# Example usage
try:
    # Send a 2-element vector (floats) to the Arduino
    vector_to_send = [12.3, 45.67]  # Example floats
    write_vector(vector_to_send)
    print(f"Sent vector: {vector_to_send}")

except Exception as e:
    print(f"Error: {e}")
