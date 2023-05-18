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

'''
    # Tamaño del mapa de ocupación
initial_cols = 100
initial_rows = 100
grid_cols = initial_cols
grid_rows = initial_rows

if os.path.exists('testmap.txt'):
    print('Map found. Loading...')
    occgrid = np.loadtxt('map.txt')
    tocc = 1.0*(occgrid > 0.5)
    occgrid[occgrid > 0.5] = 0
else:
    print('Creating new map')
    occgrid = 0.5*np.ones((grid_rows,grid_cols))
    tocc = np.zeros((grid_rows, grid_cols))
'''

def plot_map():
    global tocc, occgrid
    plt.imshow(tocc+occgrid)
    plt.show()

# Tamaño del mapa de ocupación
initial_cols = 100
initial_rows = 100
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
    
#map = np.loadtxt('testmap.txt')
#plot_map(map)

# -------- CREACIÓN DE CIRCULOS EN EL MAPA ---------
'''
map = np.zeros((100,100), np.uint8)
Ncircles = 25
xc = np.random.randint(0, 100, (Ncircles,))
yc = np.random.randint(0, 100, (Ncircles,))
print('{} {}'.format(xc[0], yc[0]))

for k in range(Ncircles):
    cv2.circle(map, (xc[k], yc[k]), np.random.randint(1, 10), (255,255,255), thickness=-1)
'''
# -------- FIN --- CREACIÓN DE CIRCULOS EN EL MAPA ---------

# -------- CREACIÓN DE RUTA MÁS CORTA  ---------

cfree = False
while not cfree:
    loc = np.random.randint(0, 100, (4,))
    vals = tocc[loc[0], loc[1]]
    vale = tocc[loc[2], loc[3]]
    if vals == 0 and vale == 0:
        cfree = True
print(loc)

route = astarmod.astar(tocc, (loc[0], loc[1]), (loc[2], loc[3]), allow_diagonal_movement=True)
rr, cc = astarmod.path2cells(route)
tocc[rr, cc] = 128
# -------- FIN ----- CREACIÓN DE RUTA MÁS CORTA  ---------

# Imprimir el mapa de ocupación y la ruta en subplots
plot_map()