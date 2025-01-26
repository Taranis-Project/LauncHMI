from serial_interface import Serial_link
from dataprocessing import dataprocessing
from influxdb_interface import Influxdb_var_interface
import json
from vedo import load
import subprocess
from Wind_command_int import BatchFileRunner
from d_viewer import Model
from d_viewer import ThreeDViewer
from re import S
import pygame as pg
import numpy as np
from numba import njit
from objLoader import read_obj
import threading
import time

""" runner = BatchFileRunner("startupsequence.bat")
runner.run_batch_file() """

""" 
def start_influxdb_powershell():
    try:
        # Run the PowerShell command to start InfluxDB service
        subprocess.run(["powershell", "Start-Service", "-Name", "influxdb"], check=True)
        print("InfluxDB service started successfully.")
    except subprocess.CalledProcessError as e:
        print(f"Failed to start InfluxDB service: {e}")
    except FileNotFoundError:
        print("PowerShell is not found. Please ensure PowerShell is installed.") """
    
with open('config.json', 'r') as json_conf_file:
    config = json.load(json_conf_file)

Serial=Serial_link(32,9600)
Serial.connect_port()

dataprocess=dataprocessing()
influxdb_interface = Influxdb_var_interface()

lenght=0

for i in range(0,len(config['Serial'][0]['data_config'])):  #get the number of different inputs from the JSON file
    type=config['Serial'][0]['data_config'][i][0]
    match type:
        case "bool":
            lenght+=1
        case "uint8":
            lenght+=1
        case "int8":
            lenght+=1
        case "uint16":
            lenght+=2
        case "int16":
            lenght+=2
        case "uint32":
            lenght+=4
        case "int32":
            lenght+=4
        case "float":
            lenght+=4
        case "double":
            lenght+=8
        case "byte":
            lenght+=1
        case "char":
            lenght+=1

# Shared variables
coord_rx, coord_ry, coord_rz = 0.0, 0.0, 0.0
lock = threading.Lock()

def main(skybox_path,model_path,texture_path):

    SCREEN_W, SCREEN_H = 800, 600
    FOV_V = np.pi/4 # 45 degrees vertical fov
    FOV_H = FOV_V*SCREEN_W/SCREEN_H
    SKY_BLUE = np.asarray([50,127,200]).astype('uint8')

    three_d_viewer = ThreeDViewer()

    pg.init()
    screen = pg.display.set_mode((SCREEN_W, SCREEN_H))
    running = True
    clock = pg.time.Clock()
    surf = pg.surface.Surface((SCREEN_W, SCREEN_H))
    frame= np.ones((SCREEN_W, SCREEN_H, 3)).astype('uint8')
    z_buffer = np.ones((SCREEN_W, SCREEN_H))
    skybox = pg.image.load(skybox_path).convert()
    skybox = pg.transform.smoothscale(skybox, (SCREEN_W*8, SCREEN_H*6))
    skybox = pg.surfarray.array3d(skybox)

    model7 = Model(model_path,texture_path)
    model7.translate(0, 00, 0)

    camera = np.asarray([13, 0.5, 2, 3.3, 0])
    #pg.mouse.set_visible(0)
    
    light_dir = np.asarray([1, 1, 1])
    light_dir = light_dir/np.linalg.norm(light_dir)
    
    h1 = int(SCREEN_W*camera[3]/FOV_H)
    v1 = int(SCREEN_H*camera[4]/FOV_V + 3*SCREEN_H)

    while running:
        #print("loop:")
        bytes_data=Serial.get_port_data(config['Serial'][0]['end_of_frame'])
        #print(bytes_data)
        
        if len(bytes_data)>=lenght:
            [coord_x,coord_y,coord_z,coord_rx,coord_ry,coord_rz,coord_rw]=dataprocess.decoding_data(bytes_data,config)

            #pg.mouse.set_pos(SCREEN_W/2, SCREEN_H/2)
            elapsed_time = clock.tick()*0.001

            """ for event in pg.event.get():
                if event.type == pg.QUIT: running = False
                if event.type == pg.KEYDOWN and event.key == pg.K_ESCAPE: running = False """
            
            frame[:,:,:] = SKY_BLUE
            frame[:,:,:]  = skybox[h1:h1+SCREEN_W,v1:v1+SCREEN_H,:]
            z_buffer[:,:] = 1e32 # start with some big value
            
            for model in Model._registry:
                three_d_viewer.project_points(model.points, camera)
                three_d_viewer.draw_model(frame, model.points, model.triangles, camera, light_dir, z_buffer, model.textured,
                        model.texture_uv, model.texture_map, model.texture)
            
            surf = pg.surfarray.make_surface(frame)
            screen.blit(surf, (0,0)); pg.display.update()
            #three_d_viewer.movement(camera, min(elapsed_time*10, 1))
            
            camera = np.asarray([13, 0.5, 2, 3.3, 0]) # reset camera
            #model7.translate(0,0,0)
            model7.rotate(coord_rx,coord_ry,coord_rz)

            pg.display.set_caption(str(round(1/(elapsed_time+1e-32), 1)) + ' ' + str(camera))

        else:
            print("serial sync error")

# Start threads
thread1 = threading.Thread(target=read_serial_data, args=(Serial, config, data_length), daemon=True)
thread2 = threading.Thread(target=process_data, daemon=True)

thread1.start()
thread2.start()

# Run the main rendering loop
skybox_path = '34173cb38f07f89ddbebc2ac9128303f-1235-oberflex_purepapercolor_skin_grey010_detail.jpg'
model_path = 'Rocket.obj'
texture_path = 'cow.png'
main_render_loop(skybox_path, model_path, texture_path)