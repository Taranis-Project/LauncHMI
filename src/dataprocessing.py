import json
from dataclasses import dataclass, make_dataclass, field
from typing import Any, List, Dict, Union, get_origin, get_args
from influxdb_interface import Influxdb_var_interface
from influxdb_client import Point
import struct
import numpy as np
from error_handling import ErrorHandling
import os

#Import influxdb

influxdb_interface=Influxdb_var_interface()

# read the conf from JSON file
with open('config.json', 'r') as json_conf_file:
    config = json.load(json_conf_file)

""" class dataprocess:
    def __init__(self):

@dataclass
class Data: """

Errorhandler=ErrorHandling()

#décode le string en byte
#string data / config / list of value

class dataprocessing():

    def __init__(self):
        self.threed_enable=0

        #If the 3d is enable get the name of the tag of the influxdb value in the config file
        if(config['HMI'][0]['Display_number'][0]=="1" and config['HMI'][0]['Parameters'][0]['Widget'][0]['3D_display']=="1"):
            self.x=config['HMI'][0]['Parameters'][0]['Widget'][0]['Parameters'][0]['x']
            self.y=config['HMI'][0]['Parameters'][0]['Widget'][0]['Parameters'][0]['y']
            self.z=config['HMI'][0]['Parameters'][0]['Widget'][0]['Parameters'][0]['z']
            self.rx=config['HMI'][0]['Parameters'][0]['Widget'][0]['Parameters'][0]['rx']
            self.ry=config['HMI'][0]['Parameters'][0]['Widget'][0]['Parameters'][0]['ry']
            self.rz=config['HMI'][0]['Parameters'][0]['Widget'][0]['Parameters'][0]['rz']
            self.rw=config['HMI'][0]['Parameters'][0]['Widget'][0]['Parameters'][0]['rw']
            self.threed_enable=1
        pass

    def decoding_data(self,bytes_data,config):

        #croping function used to trunk the data for each value
        def croping_data(data_list, size):
            # Ensure size is not larger than the length of the data_list
            if size > len(data_list):
                print("Size cannot be larger than the length of the data.")
            # Crop the data from the specified size onward
            data_list = data_list[size:]
            return data_list

        #hex to normal float (because the value is stored on a string where each byte is in hex form)
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
        
        #hex to half float
        def half_float_from_hex(hex_values):
            """
            Convert a half-precision float (16-bit) represented by 2 hexadecimal values
            into a single-precision float (32-bit).

            Args:
                hex_values: A list of 2 hexadecimal values (as strings or bytes) that represent a half-precision float.

            Returns:
                A 32-bit float converted from the hexadecimal values.
            """
            # Si hex_values sont des chaînes hexadécimales, les convertir en bytes
            if isinstance(hex_values[0], str):
                hex_values = [bytes.fromhex(value[2:]) for value in hex_values]

            # Vérification que la liste contient exactement 2 octets (pour un half float)
            if len(hex_values) != 2:
                raise ValueError("Il faut exactement 2 octets pour un half float 16 bits.")

            # Inverser les octets pour l'endianness si nécessaire (big-endian vers little-endian)
            hex_values.reverse()

            # Concaténer les octets
            byte_data = b''.join(hex_values)

            # Unpacker les 2 octets en un half-float 16 bits
            half_float_bits = struct.unpack('<H', byte_data)[0]  # Little-endian unpack pour un half float

            # Extraire les parties du half-float (sign, exposant, mantisse)
            sign = (half_float_bits >> 15) & 0x1
            exponent = (half_float_bits >> 10) & 0x1F
            mantissa = half_float_bits & 0x3FF

            # Calcul du nombre réel en format IEEE 754 demi-précision
            if exponent == 0:
                # Cas des sous-normaux
                float_value = mantissa * (2 ** -24)
            elif exponent == 31:
                # Cas de l'infini ou NaN
                if mantissa == 0:
                    float_value = float('inf') if sign == 0 else float('-inf')
                else:
                    float_value = float('nan')
            else:
                # Cas des nombres normaux
                float_value = (1 + mantissa * (2 ** -10)) * (2 ** (exponent - 15))

            # Appliquer le signe
            if sign == 1:
                float_value = -float_value

            return float_value

        #goging back to decode function

        try:
            byte_data=bytes_data
        except:
            Errorhandler.AttributeError(bytes_data)
            return None

        #harcoded the number of bytes for each value

        size_bool = 1
        size_byte = 1
        size_char= 1
        size_int8 = 1
        size_int16 = 2
        size_int32 = 4
        size_long_int = 4
        size_half_float = 2
        size_float = 4
        size_double = 8

        #Using the JSON config file to get the size of the array
        for x in range(0,len(config['Serial'][0]['data_config'])):  #get the number of different inputs from the JSON file
            #Read the config file for each data parameters
            type=config['Serial'][0]['data_config'][x][0]
            tagname=config['Serial'][0]['data_config'][x][1]
            tagvalue=config['Serial'][0]['data_config'][x][2]
            field=config['Serial'][0]['data_config'][x][3]

            #Big match case for every possibility
            match type:
                case "bool":
                    #print("bool case:")

                    try:
                        # Example usage
                        #print(byte_data[0])
                        booleans = booleans = [bool(int.from_bytes(byte_data[0]) >> i & 1) for i in range(7, -1, -1)]
                        #print(float(booleans[0]))

                        #print("running")

                    except:
                        Errorhandler.log_error("error converting bool")

                    try:
                        #storing 8 bool each time
                        #print(tagname+"0")
                        influxdb_interface.send_to_influxdb(tagname+"0",tagvalue,field,float(booleans[0]))
                        influxdb_interface.send_to_influxdb(tagname+"1",tagvalue,field,float(booleans[1]))
                        influxdb_interface.send_to_influxdb(tagname+"2",tagvalue,field,float(booleans[2]))
                        influxdb_interface.send_to_influxdb(tagname+"3",tagvalue,field,float(booleans[3]))
                        influxdb_interface.send_to_influxdb(tagname+"4",tagvalue,field,float(booleans[4]))
                        influxdb_interface.send_to_influxdb(tagname+"5",tagvalue,field,float(booleans[5]))
                        influxdb_interface.send_to_influxdb(tagname+"6",tagvalue,field,float(booleans[6]))
                        influxdb_interface.send_to_influxdb(tagname+"7",tagvalue,field,float(booleans[7]))
                    except:
                        Errorhandler.log_error("error sending "+str(x)+" bool data to influxdb")

                    byte_data=croping_data(byte_data,size_bool)

                case "uint8":
                    #print("uint8 case:")

                    try:
                        data = int.from_bytes(byte_data[0], byteorder='big')                    
                    except:
                        Errorhandler.log_error("error converting uint8")

                    try:
                        influxdb_interface.send_to_influxdb(tagname,tagvalue,field,data)
                    except:
                        Errorhandler.log_error("error sending "+str(x)+" uint8 data to influxdb")

                    byte_data=croping_data(byte_data,size_int8)

                case "int8":
                    #print("int8 case:")
                    try:
                        data = int.from_bytes(byte_data[0], byteorder='big', signed=True)
                        #print(data)
                    except:
                        Errorhandler.log_error("error converting short int")

                    try:
                        influxdb_interface.send_to_influxdb(tagname,tagvalue,field,data)
                    except:
                        Errorhandler.log_error("error sending "+str(x)+" int8 data to influxdb")

                    byte_data=croping_data(byte_data,size_int8)

                case "uint16":
                    try:
                        data = int.from_bytes(b''.join(byte_data[:size_int16]), byteorder='big')
                    except:
                        Errorhandler.log_error("error converting long int")

                    try: 
                        influxdb_interface.send_to_influxdb(tagname,tagvalue,field,data)
                    except:
                        Errorhandler.log_error("error sending "+str(x)+" uint16 data to influxdb")

                    byte_data=croping_data(byte_data,size_int16)

                case "int16":
                    try:
                        data = int.from_bytes(b''.join(byte_data[:size_int16]), byteorder='big', signed=True)
                    except:
                        Errorhandler.log_error("error converting long int")

                    try: 
                        influxdb_interface.send_to_influxdb(tagname,tagvalue,field,data)
                    except:
                        Errorhandler.log_error("error sending "+str(x)+" int16 data to influxdb")

                    byte_data=croping_data(byte_data,size_int16)

                case "uint32":
                    try:
                        data = int.from_bytes(b''.join(byte_data[:size_int32]), byteorder='big')
                    except:
                        Errorhandler.log_error("error converting long int")

                    try: 
                        influxdb_interface.send_to_influxdb(tagname,tagvalue,field,data)
                    except:
                        Errorhandler.log_error("error sending  "+str(x)+" uint32 data to influxdb")

                    byte_data=croping_data(byte_data,size_int32)

                case "int32":
                    try:
                        data = int.from_bytes(b''.join(byte_data[:size_int32]), byteorder='big', signed=True)
                    except:
                        Errorhandler.log_error("error converting long int")

                    try: 
                        influxdb_interface.send_to_influxdb(tagname,tagvalue,field,data)
                    except:
                        Errorhandler.log_error("error sending "+str(x)+" int32 data to influxdb")
                    byte_data=croping_data(byte_data,size_int32)

                case "float":
                    #print(byte_data[:size_float])
                    try:
                        data=normal_float_from_hex(byte_data[:size_float])
                    except:
                        Errorhandler.log_error("error converting float")

                    try: 
                        influxdb_interface.send_to_influxdb(tagname,tagvalue,field,data)
                    except:
                        Errorhandler.log_error("error sending "+str(x)+" float data to influxdb")

                    #print("float:")
                    #print(data)
                    byte_data=croping_data(byte_data,size_float)
                    #os.system("pause")

                case "half_float":
                    #print(byte_data[:size_half_float])
                    try:
                        data=half_float_from_hex(byte_data[:size_half_float])
                    except:
                        Errorhandler.log_error("error converting half float")

                    try:
                        influxdb_interface.send_to_influxdb(tagname,tagvalue,field,data)
                    except:
                        Errorhandler.log_error("error sending "+str(x)+" halft_float data to influxdb")

                    byte_data=croping_data(byte_data,size_half_float)
                    #os.system("pause")

                case "byte":

                    try:
                    #print(byte_data)
                        data_str=format(byte_data[0][0], '02x')
                        #print(format(byte_data[0][0], '02x'))                    
                        #print("byte:")
                        #print(data_str)
                    except:
                        Errorhandler.log_error("error converting byte")

                    try:
                        influxdb_interface.send_to_influxdb(tagname,tagvalue,field,data_str)
                    except:
                        Errorhandler.log_error("error sending " + str(x)+ " byte data to influxdb")
                        
                    #print(data)
                    byte_data=croping_data(byte_data,size_byte)
                    #os.system("pause")

                case "char":
                    try:
                    # Extraire l'entier à partir du byte

                    #print(byte_data)
                        data_byte = byte_data[0][0]  # 0x41 -> 65 (l'entier)
                    #print(bytes(data_byte))
                    #print("char:")
                    # Utiliser chr pour obtenir le caractère correspondant
                        data = chr(data_byte)  # Convertir 65 en 'A'
                    except:
                        Errorhandler.log_error("error converting char")
                    #print(data)

                    try:
                        influxdb_interface.send_to_influxdb(tagname,tagvalue,field,data)
                    except:
                        Errorhandler.log_error("error sending " + str(x) + " char data to influxdb")
                    
                    byte_data=croping_data(byte_data,size_char)
                    #os.system("pause")

                case _:
                    print("error match case type")

            #return the 6 value if the 3d viewer is activated 
            if(self.threed_enable==1):
                if(self.x==tagname):
                    coord_x=data
                elif (self.y==tagname):
                    coord_y=data
                elif (self.z==tagname):
                    coord_z=data
                elif (self.rx==tagname):
                    coord_rx=data
                elif (self.ry==tagname):
                    coord_ry=data
                elif (self.rz==tagname):
                    coord_rz=data
                elif (self.rw==tagname):
                    coord_rw=data
        
        return coord_x,coord_y,coord_z,coord_rx,coord_ry,coord_rz,coord_rw