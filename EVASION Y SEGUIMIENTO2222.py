# -*- coding: utf-8 -*-
"""
Created on Mon Mar 27 02:28:07 2023

@author: dany_
"""

'''
-x-x-x-x-x-x-x-x-x-x-
INICIO DE TRAYECTORIA
-x-x-x-x-x-x-x-x-x-x-
'''

import time
import numpy as np
import math as m
import scipy.interpolate as spi
import matplotlib.pyplot as plt
from zmqRemoteApi import RemoteAPIClient

# Generar puntos aleatorios entre -6 y +6
#cambiarán con cada compilación
num_points = 10
xarr = [0]
yarr = [0]
while len(xarr) < num_points:
    x = np.random.uniform(-6, 6)
    y = np.random.uniform(-6, 6)
    xarr.append(x)
    yarr.append(y)

#Tt = 20
# Crear un arreglo de tiempo
tarr = np.linspace(0, 10, num_points)

# Crear un arreglo de tiempo para la interpolación
# Se crean 200 instantes de tiempo
tnew = np.linspace(0, 10, 200)

# Interpolar los puntos
xc = spi.splrep(tarr, xarr, s=0.0)
yc = spi.splrep(tarr, yarr, s=0.0)

xnew = spi.splev(tnew, xc, der=0)
ynew = spi.splev(tnew, yc, der=0)
'''
ts = time.time()
td = time.time() - ts

tarr = []
xnewArr = []
ynewArr = []
'''

'''
-x-x-x-x-x-x-x-x-x-x-
FIN DE TRAYECTORIA
-x-x-x-x-x-x-x-x-x-x-
'''

'''
-x-x-x-x-x-x-x-x-x-x-
INICIO DE MOVIMIENTO
-x-x-x-x-x-x-x-x-x-x-
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


usensor = []
nSensor = 4

s1 = sim.getObject("/PioneerP3DX/ultrasonicSensor[0]")
usensor.append(s1)
s2 = sim.getObject("/PioneerP3DX/ultrasonicSensor[1]")
usensor.append(s2)
s3 = sim.getObject("/PioneerP3DX/ultrasonicSensor[2]")
usensor.append(s3)
s4 = sim.getObject("/PioneerP3DX/ultrasonicSensor[3]")
usensor.append(s4)

# Sensor initialization
for i in range(nSensor):
    state = sim.readProximitySensor(usensor[i])
time.sleep(1)
'''

usensor = []
nSensor = 4

for i in range(0, nSensor):
    s = sim.getObject(f"/PioneerP3DX/ultrasonicSensor[{i}]")
    usensor.append(s)

for i in range(nSensor):
    state, distance, _, _, _= sim.readProximitySensor(usensor[i])
time.sleep(1)
'''
'''
# Definir una función que evite obstáculos
def avoidObstacles(sim, motorL, motorR):
    # Obtener lecturas de los sensores ultrasónicos
    readings = [sim.readProximitySensor(sen) for sen in [sens0, sens1, sens2, sens3]]
    # Calcular la distancia a los obstáculos en las cuatro direcciones principales
    frontDist = min(readings[0], readings[1])
    leftDist = min(readings[2], readings[3])
    rightDist = min(readings[4], readings[5])
    backDist = readings[6]
    
    # Si hay un obstáculo en frente, girar a la izquierda o la derecha
    if frontDist < 0.5:
        if leftDist > rightDist:
            v = 0.0
            omega = -0.5
        else:
            v = 0.0
            omega = 0.5
    # Si hay un obstáculo a la izquierda, girar a la derecha
    elif leftDist < 0.5:
        v = 0.2
        omega = 0.5
    # Si hay un obstáculo a la derecha, girar a la izquierda
    elif rightDist < 0.5:
        v = 0.2
        omega = -0.5
    # Si hay un obstáculo detrás, retroceder
    elif backDist < 0.5:
        v = -0.2
        omega = 0.0
    # Si no hay obstáculos cercanos, seguir adelante
    else:
        v = 0.5
        omega = 0.0
    
    # Mover los motores según la velocidad y la dirección calculadas
    MovR(sim, motorL, motorR, v, omega, 0.1)
'''
sim.startSimulation()

# Definir el radio de las ruedas y la distancia entre ellas para el cálculo de la velocidad de los motores
R, L = 0.0975, 0.331

# Guardar la posición inicial del robot
x, y, _ = sim.getObjectPosition(robot, -1)
coords = [(x, y)]

fig, ax = plt.subplots() #Figura del trayecto
move, rotate = .7, .7 #Avance y rotación

'''
while td < Tt:
    
    td = time.time() - ts
    # tarr.append(td)

    sensor = {}
    for i in range(4):
        state = sim.readProximitySensor(usensor[i])
        sensor[i] = state

    if sensor[1] == True and sensor[2] == True:
        uRight, uLeft = v2u(0.05, -np.pi/9, R, L) # omega = angulo / tiempo
        err = sim.setJointTargetVelocity(motorL, uLeft)
        err = sim.setJointTargetVelocity(motorR, uRight)
        # time.sleep(0.35)
        time.sleep(0.5)
        td = time.time() - ts
    if sensor[0]:
        # con -pi/8 funciona para cilindros
        uRight, uLeft = v2u(0.05, -np.pi/10, R, L) # omega = angulo / tiempo
        err = sim.setJointTargetVelocity(motorL, uLeft)
        err = sim.setJointTargetVelocity(motorR, uRight)
        # time.sleep(0.3)
        time.sleep(0.5)
        td = time.time() - ts
    elif sensor[1]:
        uRight, uLeft = v2u(0.05, -np.pi/9, R, L) # omega = angulo / tiempo
        err = sim.setJointTargetVelocity(motorL, uLeft)
        err = sim.setJointTargetVelocity(motorR, uRight)
        # time.sleep(0.35)
        time.sleep(0.5)
        td = time.time() - ts
    elif sensor[2]:
        uRight, uLeft = v2u(0.05, np.pi/9, R, L) # omega = angulo / tiempo
        err = sim.setJointTargetVelocity(motorL, uLeft)
        err = sim.setJointTargetVelocity(motorR, uRight)
        # time.sleep(0.35)
        time.sleep(0.5)
        td = time.time() - ts
    elif sensor[3]:
        uRight, uLeft = v2u(0.05, np.pi/10, R, L) # omega = angulo / tiempo
        err = sim.setJointTargetVelocity(motorL, uLeft)
        err = sim.setJointTargetVelocity(motorR, uRight)
        # time.sleep(0.3)
        time.sleep(0.5)
        td = time.time() - ts
    else:
        td = time.time() - ts
        '''
# seguir la trayectoria con el robot
for i in range(len(xnew)):
    # obtener la posición actual del robot
    x, y, _ = sim.getObjectPosition(robot, -1)
    coords.append((x, y))

    # obtener la dirección del robot
    _, _, theta = sim.getObjectOrientation(robot, -1)

    # obtener el punto actual del spline
    x_target, y_target = xnew[i], ynew[i]

    # calcular la distancia al punto actual
    distance_to_target = np.sqrt((x_target - x)**2 + (y_target - y)**2)

    # si el robot está lo suficientemente cerca del punto actual, avanzar al siguiente
    if distance_to_target < 0.0001:
        if i < len(xnew)-1:
            x_target, y_target = xnew[i+1], ynew[i+1]
        else:
            x_target, y_target = xnew[-1], ynew[-1]

    # calcular el ángulo de dirección hacia el siguiente punto
    target_angle = np.arctan2(y_target - y, x_target - x)

    # calcular la velocidad lineal y angular del robot
    delta_angle = target_angle - theta
    if delta_angle > np.pi:
        delta_angle -= 2 * np.pi
    elif delta_angle < -np.pi:
        delta_angle += 2 * np.pi

    # ajustar la velocidad lineal y angular del robot
    v = move
    omega = rotate * delta_angle

    # mover el robot durante un breve período de tiempo
    MovR(sim, motorL, motorR, v, omega, 0.1)
    
    x, y, _ = sim.getObjectPosition(robot, -1) #Guardado de coordenadas
    coords.append((x, y))

    
# Pausar y detener la simulación
sim.pauseSimulation()
sim.stopSimulation()

'''
-x-x-x-x-x-x-x-x-x-x-
FIN DE MOVIMIENTO
-x-x-x-x-x-x-x-x-x-x-
'''

'''
-x-x-x-x-x-x-x-x-x-x-x-
GRAFICADO DE FUNCIONES
-x-x-x-x-x-x-x-x-x-x-x-
'''

# Graficar la trayectoria del robot
x, y = zip(*coords)
ax.plot(x, y, ".", ls="-", c="g")
ax.plot(x[0], y[0], "X", c="r")
plt.plot(xnew, ynew)
plt.plot(xarr, yarr, '.')
ax.set_title("TRAYECTORIA")
plt.show()

# Graficar los puntos y la trayectoria
plt.figure(1)
xnew = spi.splev(tnew, xc, der=0)
ynew = spi.splev(tnew, yc, der=0)
plt.plot(xnew, ynew)
plt.plot(xarr, yarr, '.')
plt.title('Path')
plt.show()

# Graficar las posiciones y velocidades en función del tiempo
plt.figure(2)
xdot = spi.splev(tnew, xc, der=1)
ydot = spi.splev(tnew, yc, der=1)
plt.plot(tnew, xnew, 'b', label='x')
plt.plot(tnew, ynew, 'r', label='y')
plt.plot(tnew, xdot, 'c', label='xdot')
plt.plot(tnew, ydot, 'm', label='ydot')
plt.plot(tarr, xarr, '.')
plt.plot(tarr, yarr, '.')
plt.legend()
plt.title('Position and velocity over time')
plt.show()

'''
-x-x-x-x-x-x-x-x-x-x-x-x-x-
FIN GRAFICADO DE FUNCIONES
-x-x-x-x-x-x-x-x-x-x-x-x-x-
'''