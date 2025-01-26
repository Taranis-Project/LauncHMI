from multiprocessing import Process, Value
import time
import numpy as np
import pygame as pg
from serial_interface import Serial_link
from dataprocessing import dataprocessing
from d_viewer import Model, ThreeDViewer
from Exe_launcher import ExeLauncher

# Shared variables for coordinates
coord_rx = Value('d', 0.0)  # 'd' is for double precision float
coord_ry = Value('d', 0.0)
coord_rz = Value('d', 0.0)

# Serial Data Reading Process
def read_serial_data(config, data_length, coord_rx, coord_ry, coord_rz):
    serial_link = Serial_link(32, 9600)  # Initialize serial link within the process
    serial_link.connect_port()

    dataprocess = dataprocessing()
    while True:
        bytes_data = serial_link.get_port_data(config['Serial'][0]['end_of_frame'])
        if len(bytes_data) >= data_length:
            coords = dataprocess.decoding_data(bytes_data, config)
            with coord_rx.get_lock(), coord_ry.get_lock(), coord_rz.get_lock():
                coord_rx.value, coord_ry.value, coord_rz.value = coords[3], coords[4], coords[5]
        else:
            print("Serial sync error")
        time.sleep(0.01)

# Rendering Loop Process
def main_render_loop(skybox_path, model_path, texture_path, coord_rx, coord_ry, coord_rz):
    SCREEN_W, SCREEN_H = 800, 600
    SKY_BLUE = np.asarray([50, 127, 200]).astype('uint8')

    three_d_viewer = ThreeDViewer()

    pg.init()
    screen = pg.display.set_mode((SCREEN_W, SCREEN_H))
    clock = pg.time.Clock()
    frame = np.ones((SCREEN_W, SCREEN_H, 3)).astype('uint8')

    skybox = pg.image.load(skybox_path).convert()
    skybox = pg.transform.smoothscale(skybox, (SCREEN_W * 8, SCREEN_H * 6))
    skybox = pg.surfarray.array3d(skybox)
    z_buffer = np.ones((SCREEN_W, SCREEN_H))

    model7 = Model(model_path, texture_path)
    model7.translate(0, 0, 0)

    camera = np.asarray([13, 0.5, 2, 3.3, 0])
    light_dir = np.asarray([1, 1, 1])
    light_dir = light_dir / np.linalg.norm(light_dir)

    while True:
        elapsed_time = clock.tick() * 0.001
        frame[:, :, :] = SKY_BLUE
        frame[:, :, :] = skybox[:SCREEN_W, :SCREEN_H, :]
        z_buffer[:,:] = 1e32 # start with some big value

        with coord_rx.get_lock(), coord_ry.get_lock(), coord_rz.get_lock():
            rx, ry, rz = coord_rx.value, coord_ry.value, coord_rz.value

        model7.rotate(rx, ry, rz)
        for model in Model._registry:
            three_d_viewer.project_points(model.points, camera)
            three_d_viewer.draw_model(frame, model.points, model.triangles, camera, light_dir, z_buffer, model.textured,
                    model.texture_uv, model.texture_map, model.texture)
            
        surf = pg.surfarray.make_surface(frame)
        screen.blit(surf, (0, 0))
        pg.display.update()

# Main Entry Point
if __name__ == '__main__':
    import json
    with open('config.json', 'r') as json_conf_file:
        config = json.load(json_conf_file)

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
    
    # Paths
    skybox_path = '34173cb38f07f89ddbebc2ac9128303f-1235-oberflex_purepapercolor_skin_grey010_detail.jpg'
    model_path = 'Rocket.obj'
    texture_path = 'cow.png'

    # Start processes
    process1 = Process(target=read_serial_data, args=(config, lenght, coord_rx, coord_ry, coord_rz))
    process2 = Process(target=main_render_loop, args=(skybox_path, model_path, texture_path, coord_rx, coord_ry, coord_rz))

    process1.start()
    process2.start()

    process1.join()
    process2.join()