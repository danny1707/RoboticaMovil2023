# -*- coding: utf-8 -*-
"""
Created on Thu Mar 16 13:21:50 2023

@author: dany_
"""

import time
import math as m
import matplotlib.pyplot as plt
import numpy as np
import scipy.interpolate as spi

from zmqRemoteApi import RemoteAPIClient

'''
inicio de la trayectoria
'''

# Generar puntos aleatorios
#cambiarán con cada compilación
num_points = 10
xarr = []
yarr = []
while len(xarr) < num_points:
    x = np.random.rand() * 10
    y = np.random.rand() * 10
    if x not in xarr:
        xarr.append(x)
        yarr.append(y)

# Crear un arreglo de tiempo
tarr = np.linspace(0, 10, num_points)

# Crear un arreglo de tiempo para la interpolación
# Se crean 200 instantes de tiempo
tnew = np.linspace(0, 10, 200)

# Interpolar los puntos
xc = spi.splrep(tarr, xarr, s=0)
yc = spi.splrep(tarr, yarr, s=0)

'''
Fin de la trayectoria
'''

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

# Guardar la posición inicial del robot
x, y, _ = sim.getObjectPosition(robot, -1)
coords = [(x, y)]

fig, ax = plt.subplots() #Figura del trayecto

# Definir las constantes del controlador de la trayectoria
kp = 1.5
ka = 3

# Iniciar el controlador de la trayectoria
theta = 0
for i in range(len(tnew)):
    # Obtener la posición interpolada en el instante i
    xnew = spi.splev(tnew[i], xc, der=0)
    ynew = spi.splev(tnew[i], yc, der=0)
    
    # Calcular el error en la posición y la orientación del robot
    ex = xnew - x
    ey = ynew - y
    etheta = m.atan2(ey, ex) - theta
    
    # Ajustar el ángulo para que esté entre -pi y pi
    if etheta > m.pi:
        etheta -= 2*m
    elif etheta < -m.pi:
        etheta += 2*m.pi
    #ejecutar la trayectoria
    for i in range(len(tnew)-1):
        # Obtener la posición interpolada en el instante de tiempo actual
        x, y = spi.splev(tnew[i], xc), spi.splev(tnew[i], yc)
    
    # Guardar la posición actual del robot
    x_actual, y_actual, _ = sim.getObjectPosition(robot, -1)
    coords.append((x_actual, y_actual))

    # Calcular la velocidad lineal y angular necesaria para seguir la trayectoria
    v = kp * m.sqrt(ex**2 + ey**2)
    omega = ka * etheta

    # Mover el robot a lo largo de la trayectoria interpolada
    for i in range(len(tnew)):
        # Obtener la posición interpolada en el instante de tiempo actual
        x = spi.splev(tnew[i], xc, der=0)
        y = spi.splev(tnew[i], yc, der=0)
        
        # Calcular la velocidad lineal y angular necesaria para mover el robot hacia la posición interpolada
        for i in range(len(tnew)):
            # Obtener la posición interpolada en el instante i
            xnew = spi.splev(tnew[i], xc, der=0)
            ynew = spi.splev(tnew[i], yc, der=0)
    # Actualizar la posición y orientación del robot
    x, y, theta = sim.getObjectOrientation(robot, -1)
    coords.append((x, y))

    # Pausar y detener la simulación
    sim.pauseSimulation()
    sim.stopSimulation()
    
    #Dibujar la trayectoria del robot

    xvals = [coord[0] for coord in coords]
    yvals = [coord[1] for coord in coords]
    ax.plot(xvals, yvals)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Trayectoria del robot')
    plt.show()