# -*- coding: utf-8 -*-
"""
Created on Sun Mar 26 19:40:54 2023

@author: dany_
"""

import time
import math as m
import numpy as np
import matplotlib.pyplot as plt
#from scipy.interpolate import make_interp_splin
from scipy.interpolate import splprep, splev
from zmqRemoteApi import RemoteAPIClient

# Definir una función que convierta la velocidad lineal y angular del robot en las velocidades de los motores
def v2u(v, omega, r, L):
    return (v/r + L*omega/(2*r), v/r - L*omega/(2*r))

# Definir una función que controle los motores del robot para moverlo a una velocidad y una velocidad angular durante un tiempo determinado
def MovR(sim, motorL, motorR, v, omega, duration):
    end_time = sim.getSimulationTime() + duration
    while sim.getSimulationTime() < end_time:
        sim.setJointTargetVelocity(motorL, v2u(v, omega, R, L)[1])
        sim.setJointTargetVelocity(motorR, v2u(v, omega, R, L)[0])

# Inicializar la simulación y obtener los objetos necesarios
client = RemoteAPIClient()
sim = client.getObject('sim')
motorL = sim.getObject("/PioneerP3DX/leftMotor")
motorR = sim.getObject("/PioneerP3DX/rightMotor")
robot = sim.getObject("/PioneerP3DX")
sim.startSimulation()

# Definir el radio de las ruedas y la distancia entre ellas para el cálculo de la velocidad de los motores
R, L = 0.0975, 0.331


'''
# Generar 10 puntos aleatorios
num_points = 10
x = np.random.rand(num_points) * 10 - 5
y = np.random.rand(num_points) * 10 - 5

# Crear un spline a través de los puntos generados aleatoriamente
tck, u = make_interp_spline(x, y, k=3, s=0)(np.linspace(0, 1, 100), full_output=False)
'''

# Generar trayectoria aleatoria de 10 puntos en el rango [0, 1] x [0, 1]
#np.random.seed(0)
points = np.random.rand(10) * 10 - 5

# Calcular spline cúbico a través de los puntos
tck, u = splprep(points.T, k=3, s=0)

# Evaluar la trayectoria en un conjunto de puntos uniformemente espaciados
u_new = np.linspace(u.min(), u.max(), 100)
x_new, y_new = splev(u_new, tck)

# Guardar la posición inicial del robot
x, y, _ = sim.getObjectPosition(robot, -1)
coords = [(x, y)]

fig, ax = plt.subplots() #Figura del trayecto

for i in range(len(u) - 1):
    # Calcular la dirección y la distancia al siguiente punto de spline
    v = [tck[0][i+1] - tck[0][i], tck[1][i+1] - tck[1][i]]
    mag = m.sqrt(v[0]**2 + v[1]**2)
    dir = np.arctan2(v[1], v[0]) - m.pi/2
    if dir < -m.pi:
        dir += 2*m.pi
    if dir > m.pi:
        dir -= 2*m.pi
        
    # Mover el robot a lo largo del spline
    MovR(sim, motorL, motorR, 0.5, dir, mag/0.5)
    
    # Guardar las coordenadas
    x, y, _ = sim.getObjectPosition(robot, -1)
    coords.append((x, y))

# Pausar y detener la simulación
sim.pauseSimulation()
sim.stopSimulation()

# Graficar la trayectoria del robot
x, y = zip(*coords)
ax.plot(x, y, ".", ls="-", c="g")
ax.plot(x[0], y[0], "X", c="r")
ax.set_title("Trayecotira")
plt.show()