from multiprocessing import Process, Value
import time
import numpy as np
import pygame as pg
from serial_interface import Serial_link
from dataprocessing import dataprocessing
from d_viewer import Model, ThreeDViewer
from error_handling import ErrorHandling


# Shared variables for object coordinates
coord_rx = Value('d', 0.0)  # 'd' is for double precision float
coord_ry = Value('d', 0.0)
coord_rz = Value('d', 0.0)
coord_x = Value('d', 0.0)
coord_y = Value('d', 0.0)
coord_z = Value('d', 0.0)

# Serial Data Reading and storing process
def read_serial_data(config, data_length, coord_rx, coord_ry, coord_rz):
    ErrorHandler=ErrorHandling()
    serial_link = Serial_link(config['Serial'][0]['number'],config['Serial'][0]['baudrate'])  # Initialize serial link within the process
    serial_link.connect_port() # Initialize serial link within the process
    dataprocess = dataprocessing() #Initialize the dataprocessing object within the process

    #main processing loop
    while True:
        bytes_data = serial_link.get_port_data(config['Serial'][0]['end_of_frame']) #Get port data and trunk using the end_of_frame bytes
        if len(bytes_data) >= data_length:  #rapid chek for the number of bytes (>= for allowing partial reception)

            coords = dataprocess.decoding_data(bytes_data, config)  #decoding and storing the data in influxdb
            
            with coord_rx.get_lock(), coord_ry.get_lock(), coord_rz.get_lock(): #coords is used for the Rendering loop only
                coord_x.value, coord_y.value, coord_z.value, coord_rx.value, coord_ry.value, coord_rz.value = coords[0],coords[1],coords[2],coords[3],coords[4],coords[5] #value passed to the other process
        else:
            ErrorHandler.log_error("Serial sync error")  #if the number of bytes is too long an error is logged
        #time.sleep(0.01) #time for allowing other process

# Rendering Loop Process
def main_render_loop(skybox_path, model_path, texture_path, Hight, Width): #needing to pass the variable ?

    #définition de la taille de l'écran
    SCREEN_W, SCREEN_H = int(Width),int(Hight)
    SKY_BLUE = np.asarray([50, 127, 200]).astype('uint8')

    three_d_viewer = ThreeDViewer() #Initialize the viewer object

    pg.init() #Pygame initialisation
    screen = pg.display.set_mode((SCREEN_W, SCREEN_H))
    clock = pg.time.Clock()
    frame = np.ones((SCREEN_W, SCREEN_H, 3)).astype('uint8')

    #Skybox
    skybox = pg.image.load(skybox_path).convert()
    skybox = pg.transform.smoothscale(skybox, (SCREEN_W * 8, SCREEN_H * 6))
    skybox = pg.surfarray.array3d(skybox)
    z_buffer = np.ones((SCREEN_W, SCREEN_H))

    #Rocket model loading
    model1 = Model(model_path, texture_path)
    model1.translate(0, 0, 0)

    #camera and lightning setup
    camera = np.asarray([13, 0.5, 2, 3.3, 0])
    light_dir = np.asarray([1, 1, 1])
    light_dir = light_dir / np.linalg.norm(light_dir)

    while True:
        elapsed_time = clock.tick() * 0.001
        frame[:, :, :] = SKY_BLUE
        frame[:, :, :] = skybox[:SCREEN_W, :SCREEN_H, :]
        z_buffer[:,:] = 1e32 # start with some big value

        #
        with coord_rx.get_lock(), coord_ry.get_lock(), coord_rz.get_lock():
            x, y, z, rx, ry, rz = coord_x.value, coord_y.value, coord_z.value, coord_rx.value, coord_ry.value, coord_rz.value

        model1.rotate(rx, ry, rz)
        model1.translate(x, y, z)
        for model in Model._registry:
            three_d_viewer.project_points(model.points, camera)
            three_d_viewer.draw_model(frame, model.points, model.triangles, camera, light_dir, z_buffer, model.textured,
                    model.texture_uv, model.texture_map, model.texture)
            
        surf = pg.surfarray.make_surface(frame)
        screen.blit(surf, (0, 0))
        pg.display.update()

# Main process setup
if __name__ == '__main__':
    import json
    with open('config.json', 'r') as json_conf_file:
        config = json.load(json_conf_file)

    lenght=0

    for i in range(0,len(config['Serial'][0]['data_config'])):  #get the lenght from diferent inputs from the JSON file
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
    skybox_path = config['HMI'][0]['Parameters'][0]['Widget'][0]['Parameters'][0]['Skybox']
    model_path = config['HMI'][0]['Parameters'][0]['Widget'][0]['Parameters'][0]['Mesh']
    texture_path = config['HMI'][0]['Parameters'][0]['Widget'][0]['Parameters'][0]['Texture']
    Hight=config['HMI'][0]['Parameters'][0]['Widget'][0]['Parameters'][0]['Hight']
    Width=config['HMI'][0]['Parameters'][0]['Widget'][0]['Parameters'][0]['Width']

    # Start parallel processes
    process1 = Process(target=read_serial_data, args=(config, lenght, coord_rx, coord_ry, coord_rz))
    process2 = Process(target=main_render_loop, args=(skybox_path, model_path, texture_path, Hight, Width))

    process1.start()
    process2.start()

    process1.join()
    process2.join()