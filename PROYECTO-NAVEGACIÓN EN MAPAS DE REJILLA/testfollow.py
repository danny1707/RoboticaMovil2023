# -*- coding: utf-8 -*-
"""
Created on Wed May 17 22:18:02 2023

@author: dany_
"""

import numpy as np
import time
import math as m
import random
import sys
import matplotlib.pyplot as plt
import os
from zmqRemoteApi import RemoteAPIClient
from skimage.draw import line 
#from skimage.draw import line
import cv2
import astarmod

def plot_map():
    global tocc, occgrid
    plt.imshow(tocc+occgrid)
    plt.show()

# Tamaño del mapa de ocupación
initial_cols = 15
initial_rows = 15
grid_cols = initial_cols
grid_rows = initial_rows

# Cargar el archivo de texto
if os.path.exists('testmap.txt'):
    print('Map found. Loading...')
    occgrid = np.loadtxt('testmap.txt')
    tocc = 1*(occgrid > 0.5)
    occgrid[occgrid > 0.5] = 0
else:
    print('Creating new map')
    occgrid = 0.5*np.ones((grid_rows,grid_cols))
    tocc = np.zeros((grid_rows, grid_cols))

# -------- CREACIÓN DE RUTA MÁS CORTA  ---------

cfree = False
while not cfree:
    loc = np.random.randint(0, 15, (4,))
    vals = tocc[loc[0], loc[1]]
    vale = tocc[loc[2], loc[3]]
    if vals == 0 and vale == 0:
        cfree = True
print(loc)
print('punto de inicio', loc[0], loc[1])
print('punto de termino', loc[2], loc[3])

route = astarmod.astar(tocc, (loc[0], loc[1]), (loc[2], loc[3]), allow_diagonal_movement=True)
rr, cc = astarmod.path2cells(route)
tocc[rr, cc] = 1
# -------- FIN ----- CREACIÓN DE RUTA MÁS CORTA  ---------
plot_map()

# Obtener la posición inicial
start_pos = [float(loc[0]), float(loc[1]), 1.0]  # Convertir a float y agregar coordenada z

# Conexión con CoppeliaSim
client = RemoteAPIClient()
sim = client.getObject('sim')

motorL = sim.getObject('/PioneerP3DX/leftMotor')
motorR = sim.getObject('/PioneerP3DX/rightMotor')
robot = sim.getObject('/PioneerP3DX')

sim.startSimulation()

# Mover el robot a la posición inicial
sim.setObjectPosition(robot, -1, start_pos)

# Seguir la ruta generada
for point in route:
    target_pos = [float(point[0]), float(point[1]), 0.0]  # Convertir a float y agregar coordenada z
    # Calcular la dirección del punto objetivo
    target_angle = m.atan2(target_pos[1] - start_pos[1], target_pos[0] - start_pos[0])
    # Calcular la distancia al punto objetivo
    distance = m.sqrt((target_pos[0] - start_pos[0])**2 + (target_pos[1] - start_pos[1])**2)

    # Girar hacia el punto objetivo
    sim.setObjectOrientation(robot, -1, [0, 0, target_angle])

    # Mover hacia el punto objetivo
    sim.setJointTargetVelocity(motorL, 2.0)
    sim.setJointTargetVelocity(motorR, 2.0)

    # Esperar hasta que el robot llegue al punto objetivo
    while distance > 0.1:
        carpos = sim.getObjectPosition(robot, -1)
        distance = m.sqrt((target_pos[0] - carpos[0])**2 + (target_pos[1] - carpos[1])**2)
        time.sleep(0.1)

    # Detener el movimiento del robot
    sim.setJointTargetVelocity(motorL, 0.0)
    sim.setJointTargetVelocity(motorR, 0.0)

    # Actualizar la posición actual
    start_pos = target_pos

sim.stopSimulation()