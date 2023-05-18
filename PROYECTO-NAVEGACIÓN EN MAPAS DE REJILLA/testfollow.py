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
initial_cols = 150
initial_rows = 150
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
    loc = np.random.randint(0, 150, (4,))
    vals = tocc[loc[0], loc[1]]
    vale = tocc[loc[2], loc[3]]
    if vals == 0 and vale == 0:
        cfree = True
print(loc)
print('punto de inicio', loc[0], loc[1])
print('punto de termino', loc[2], loc[3])

route = astarmod.astar(tocc, (loc[0], loc[1]), (loc[2], loc[3]), allow_diagonal_movement=True)
rr, cc = astarmod.path2cells(route)
tocc[rr, cc] = 2
# -------- FIN ----- CREACIÓN DE RUTA MÁS CORTA  ---------

plot_map()

client = RemoteAPIClient()
sim = client.getObject('sim')

motorL=sim.getObject('/PioneerP3DX/leftMotor')
motorR=sim.getObject('/PioneerP3DX/rightMotor')
robot = sim.getObject('/PioneerP3DX')

sim.startSimulation()

carpos = sim.getObjectPosition(robot, -1)
carrot = sim.getObjectOrientation(robot, -1)

sim.stopSimulation()
# Imprimir el mapa de ocupación y la ruta en subplots
plot_map()