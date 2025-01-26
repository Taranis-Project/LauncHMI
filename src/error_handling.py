from FileWriter import FileWriter
import datetime
import FreeSimpleGUI as sg
import socket
import os

class ErrorHandling:
    def __init__(self):
        now = datetime.datetime.now()
        current_date_hour = now.strftime("%Y_%m_%d_%H_%M_%S")
        filename = os.path.join("log", f"log_{current_date_hour}.txt")
        print(filename)
        self.file_writer = FileWriter(filename)
        pass

    def on_error_stop():
        print("Error occured, stopping the program")
        while True:
            pass
        exit()

    def show_error_message(self, string):
        sg.Popup(string)
        return None

    def log_error(self, string):
        self.file_writer.write_to_file(string)
        return None

    def AttributeError(self, error):
        now = datetime.datetime.now()
        current_date_hour = now.strftime("%Y-%m-%d_%H:%M:%S") 
        string = f"{current_date_hour} Attribute_error/dataprocessing:byte_data=string_data.encode('utf-8', errors='ignore')_bytes: {error}"
        print(string)
        self.log_error(string)
        #self.show_error_message(string)
        #self.on_error_stop(string)
        return None

""" test=ErrorHandling()
test.AttributeError("testerror") """