byte_data = [b'0xf8']  # A list containing a byte 'A'

# Extract the integer value from the byte (0th byte) and format it as hex
print(format(byte_data[0][0], '02x'))  # '02x' means two characters, padded with zero if needed
