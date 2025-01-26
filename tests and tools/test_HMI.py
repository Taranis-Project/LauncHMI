import json
from dataclasses import dataclass, make_dataclass, field
from typing import Any, List, Dict, Union, get_origin, get_args
from influxdb_client import Point
import struct
import numpy as np
import os

with open('config.json', 'r') as json_conf_file:
    config = json.load(json_conf_file)

if(config['HMI'][0]['Display_number'][0]=="1" and config['HMI'][0]['Parameters'][0]['Widget'][0]['3D_display']=="1"):
    x=config['HMI'][0]['Parameters'][0]['Widget'][0]['Parameters'][0]['x']
    y=config['HMI'][0]['Parameters'][0]['Widget'][0]['Parameters'][0]['y']
    z=config['HMI'][0]['Parameters'][0]['Widget'][0]['Parameters'][0]['z']
    rx=config['HMI'][0]['Parameters'][0]['Widget'][0]['Parameters'][0]['rx']
    ry=config['HMI'][0]['Parameters'][0]['Widget'][0]['Parameters'][0]['ry']
    rz=config['HMI'][0]['Parameters'][0]['Widget'][0]['Parameters'][0]['rz']
    rw=config['HMI'][0]['Parameters'][0]['Widget'][0]['Parameters'][0]['rw']
