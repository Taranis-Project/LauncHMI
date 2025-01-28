#InfluxDB link
import json
import influxdb_client, os, time
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS
from datetime import datetime, timezone


class Influxdb_var_interface(influxdb_client.Point):
  def __init__(self):
     return
  
  def send_to_influxdb(self,tagname,tagvalue,field,value):
    point = (Point("mesurement1")
    .tag(tagname, tagvalue)
    .field(field, value)
    )
    write_api.write(bucket=bucket, org="Taranis_project", record=point)