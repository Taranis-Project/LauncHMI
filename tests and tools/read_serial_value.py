import serial
import time

# Specify the correct COM port and baud rate
COM_PORT = 'COM32'  # Replace with your Arduino's COM port
BAUD_RATE = 9600   # Match the baud rate in your Arduino sketch

try:
    # Open the serial port
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # Wait for the connection to initialize

    print("Connected to Arduino!")

    # Continuously read from the serial port
    while True:
        if ser.in_waiting > 0:  # Check if data is available
            line = ser.read()  # Read and decode the line
            print(f"Received: {line}")
except serial.SerialException as e:
    print(f"Error: {e}")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()  # Ensure the serial port is closed
        print("Serial connection closed.")