""" import struct

# Simulating a list of byte values
byte_data = [b'\x01',b'\x02',b'\x03',b'\x04']  # This is a bytes object

size_half_float = 2

def half_float_to_float(hex_values):

    Convert a half-precision float (16-bit) represented by 4 hexadecimal values
    into a single-precision float (32-bit).
    
    Args:
        hex_values: A list of 4 hexadecimal values that represent a half-precision float.
    
    Returns:
        A 32-bit float converted from the half-precision values.

    
    # Step 1: Convert the list of hex values to a bytes object
    byte_data = bytes([int(val, 16) for val in hex_values])
    
    # Step 2: Unpack the first 2 bytes as a half-precision float (16-bit) using struct
    half_float = struct.unpack('<H', byte_data[:2])[0]  # Little-endian unpack
    
    # Step 3: Convert the half-precision float to a full float (32-bit)
    full_float = np.float32(half_float)
    
    return full_float

# Convert list to bytes and unpack
half_float=half_float_to_float(byte_data)
print("Half Float:", half_float)

 """

import struct
import numpy as np  # You can also use np.float32() if needed

def normal_float_from_hex(hex_values):
    """
    Convert a normal float (32-bit) represented by 4 hexadecimal values
    into a single-precision float (32-bit).
    
    Args:
        hex_values: A list of 4 hexadecimal values that represent a normal float.
    
    Returns:
        A 32-bit float converted from the hexadecimal values.
    """
    
    # Step 1: Convert the list of hex values (as integers) into a bytes object

    hex_values.reverse()
    byte_data = b''.join(hex_values)  # No need to reverse for big-endian data
    
    # Step 2: Unpack the 4 bytes as a 32-bit float using struct
    normal_float = struct.unpack('<f', byte_data[:4])[0]  # Little-endian unpack for float
    
    return normal_float

# Example usage for 1.0
hex_values = [b'\x56', b'\x43', b'\x00', b'\x00']  # This represents the float 1.0 in hexadecimal (IEEE 754 format)

# Convert the hex values into a normal float
converted_float = normal_float_from_hex(hex_values)
print("Converted Float:", converted_float)
 