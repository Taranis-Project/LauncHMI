import asyncio
import serial_asyncio
import serial
import json
from FileWriter import FileWriter
import datetime
import os
import time

class Serial_link(serial.Serial):
    def __init__(self, number, baudrate):
        port = 'COM' + str(number)
        baudrate = int(baudrate)
        super().__init__(port=port, baudrate=baudrate, timeout=1)

        now = datetime.datetime.now()
        current_date_hour = now.strftime("%Y_%m_%d_%H_%M_%S")
        filename = os.path.join("raw", f"{port}_{baudrate}_{current_date_hour}.txt")
        print(filename)
        self.file_writer = FileWriter(filename)

    def connect_port(self):
        if not self.is_open:
            self.open()

    def disconnect_port(self):
        if self.is_open:
            self.close()

    def send_data(self, data_string):
        """Sends data to the Arduino board asynchronously.

        Args:
            data_string: The data to be sent as a string.
        """
        data_bytes = data_string.encode()
        try:
            self.write(data_bytes)
        except serial.SerialException as e:
            print(f"Error sending data: {e}")

    def get_port_data(self,hex_string_bytes_end):
        """
        Reads data from the serial port synchronously and returns the data as a string.
        """
        hex_values = hex_string_bytes_end.split('0x')[1:]  # Split and ignore the first empty part

        # Convert each part to an integer and then to a byte
        bytes_end = bytes([int(h, 16) for h in hex_values])
        #print(bytes_end)

        while True:
            if self.in_waiting:  # Check if there is data to read
                #data_bytes = self.readline()  # Read data synchronously
                received_parts = []  # Initialize as a string
                while True:
                    chunk = self.read()
                    #print(chunk)
                    if not chunk:
                        break
                    received_parts.append(chunk)  # Store raw bytes directly

                    if bytes_end == b''.join(received_parts)[-len(bytes_end):]:
                        break
                try:
                    # Decode the bytes to a string and strip newline characters
                    self.file_writer.write_to_file(str(received_parts))
                    return received_parts  # Return the processed string
                except UnicodeDecodeError:
                    print(f"Error decoding data: {received_parts}")
                    return None  # Return None or handle the error appropriately