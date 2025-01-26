#InfluxDB link
import json
import influxdb_client, os, time
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS
from datetime import datetime, timezone

with open('config.json', 'r') as json_conf_file:
    config = json.load(json_conf_file)

token = config['Influxdb'][0]['token']
org = config['Influxdb'][0]['org']
url = config['Influxdb'][0]['url']
bucket = config['Influxdb'][0]['bucket']

write_client = influxdb_client.InfluxDBClient(url=url, token=token, org=org)
write_api = write_client.write_api(write_options=SYNCHRONOUS)

class Influxdb_var_interface(influxdb_client.Point):
  def __init__(self):
     return
  
  def send_to_influxdb(self,tagname,tagvalue,field,value):
    point = (Point("mesurement1")
    .tag(tagname, tagvalue)
    .field(field, value)
    )
    write_api.write(bucket=bucket, org="Taranis_project", record=point)

# Define your InfluxDB connection details
url = "http://localhost:8086"
token = config['Influxdb'][0]['token']
org = config['Influxdb'][0]['org']
bucket = config['Influxdb'][0]['bucket']

# Initialize the InfluxDB client
client = InfluxDBClient(url=url, token=token, org=org)

# Initialize the Delete API
delete_api = client.delete_api()

# Define the measurement you want to delete and the time range
measurement = "mesurement1"
start = "1970-01-01T00:00:00Z"  # Start time (beginning of time)
stop = datetime.now(timezone.utc).isoformat()

# Construct the delete predicate (measurement filter)
predicate = f'_measurement="{measurement}"'

# Perform the deletion
delete_api.delete(start, stop, bucket=bucket, org=org, predicate=predicate)

print(f"Measurement '{measurement}' deleted successfully from bucket '{bucket}'.")

type=config['Serial'][0]['data_config'][0][0]
tagname=config['Serial'][0]['data_config'][0][1]
tagvalue=config['Serial'][0]['data_config'][0][2]
field=config['Serial'][0]['data_config'][0][3]

value=1.0

influxdb_interface = Influxdb_var_interface()
influxdb_interface.send_to_influxdb(tagname,tagvalue,field,value)