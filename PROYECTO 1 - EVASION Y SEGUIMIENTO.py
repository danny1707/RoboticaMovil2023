# -*- coding: utf-8 -*-
"""
Created on Thu Mar 16 12:51:50 2023

@author: dany_
"""

import numpy as np
import time
import math as m
import sim # access all the sim elements
import matplotlib.pyplot as plt
import scipy.interpolate as spi
import random as rm

def angdiff(t1, t2):
    # The angle magnitude comes from the dot product of two vectors
    angmag = m.acos(m.cos(t1)*m.cos(t2)+m.sin(t1)*m.sin(t2))
    # The direction of rotation comes from the sign of the cross product of two vectors
    angdir = m.cos(t1)*m.sin(t2)-m.sin(t1)*m.cos(t2)
    return m.copysign(angmag, angdir)

def v2u(v, omega, r, L):
    ur = v/r + L*omega/(2*r)
    ul = v/r - L*omega/(2*r)
    return ur, ul


########
# MAIN #
########
'''
# Getting handles for the motors and robot
err, motorL = sim.GetObjectHandle('/PioneerP3DX/leftMotor', sim.sim_opmode_blocking)
err, motorR = sim.GetObjectHandle('/PioneerP3DX/rightMotor', sim.sim_opmode_blocking)
err, robot = sim.GetObjectHandle('/PioneerP3DX', sim.sim_opmode_blocking)
'''

# Getting handles for the motors and robot
err, motorL = sim.GetObject("/PioneerP3DX/leftMotor")
err, motorR = sim.GetObject("/PioneerP3DX/rightMotor")
err, robot = sim.GetObject("/PioneerP3DX")

# Assigning handles to the ultrasonic sensors
usensor = []
nSensor = 4

for i in range(0,nSensor):
    err, sensor = sim.GetObject("/PioneerP3DX/ultrasonicSensor["+str(i)+"]")
    usensor.append(sensor)

# Sensor initialization
for i in range(nSensor):
    err, state, point, detectedObj, detectedSurfNormVec = sim.ReadProximitySensor(sensor)
time.sleep(1)

xarr = []
yarr = []


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
'''

# Maximo numero aleatorio en la generacion de la trayectoria
# ( Modificar si es requerido )

Min = -5
Max = 5

xarr.append(0)
yarr.append(0)

# Inizializar trayectoria aleatoria.
for i in range(0,10):
    nr = rm.randint(Min, Max)
    xarr.append(nr)
    nr = rm.randint(Min, Max)
    yarr.append(nr)

# xarr = [0, 5]
# yarr = [0, 0]
xarr = np.array(xarr)
yarr = np.array(yarr)
print(f'X-Y:\n{xarr}\n{yarr}\n')

# Controller gains (linear and heading)
Kv = 0.5
Kh = 0.9

# Medidas del robot
hd = 0
r = 0.5*1.9502e-01
L = 2.0*1.6550e-01
errp = 10


xpos = []
ypos = []

Tt = 300
tnew = np.linspace(0, Tt, 100)
tarr = np.linspace(0, Tt, xarr.shape[0])
# Interpolador
pcix = spi.PchipInterpolator(tarr, xarr)
pciy = spi.PchipInterpolator(tarr, yarr)

ts = time.time()
td = time.time() - ts

tarr = []
xnewArr = []
ynewArr = []

while td < Tt:
    
    td = time.time() - ts
    # tarr.append(td)

    sensor = {}
    for i in range(nSensor):
        err, state, point, detectedObj, detectedSurfNormVec = sim.ReadProximitySensor(usensor[i], sim.sim_opmode_buffer)
        sensor[i] = state

    if sensor[1] == True and sensor[2] == True:
        uRight, uLeft = v2u(0.05, -np.pi/9, r, L) # omega = angulo / tiempo
        err = sim.SetJointTargetVelocity(motorL, uLeft, sim.sim_opmode_oneshot)
        err = sim.SetJointTargetVelocity(motorR, uRight, sim.sim_opmode_oneshot)
        # time.sleep(0.35)
        time.sleep(0.5)
        td = time.time() - ts
    if sensor[0]:
        # con -pi/8 funciona para cilindros
        uRight, uLeft = v2u(0.05, -np.pi/10, r, L) # omega = angulo / tiempo
        err = sim.SetJointTargetVelocity(motorL, uLeft, sim.sim_opmode_oneshot)
        err = sim.SetJointTargetVelocity(motorR, uRight, sim.sim_opmode_oneshot)
        # time.sleep(0.3)
        time.sleep(0.5)
        td = time.time() - ts
    elif sensor[1]:
        uRight, uLeft = v2u(0.05, -np.pi/9, r, L) # omega = angulo / tiempo
        err = sim.SetJointTargetVelocity(motorL, uLeft, sim.sim_opmode_oneshot)
        err = sim.simSetJointTargetVelocity(motorR, uRight, sim.sim_opmode_oneshot)
        # time.sleep(0.35)
        time.sleep(0.5)
        td = time.time() - ts
    elif sensor[2]:
        uRight, uLeft = v2u(0.05, np.pi/9, r, L) # omega = angulo / tiempo
        err = sim.SetJointTargetVelocity(motorL, uLeft, sim.sim_opmode_oneshot)
        err = sim.SetJointTargetVelocity(motorR, uRight, sim.sim_opmode_oneshot)
        # time.sleep(0.35)
        time.sleep(0.5)
        td = time.time() - ts
    elif sensor[3]:
        uRight, uLeft = v2u(0.05, np.pi/10, r, L) # omega = angulo / tiempo
        err = sim.SetJointTargetVelocity(motorL, uLeft, sim.sim_opmode_oneshot)
        err = sim.SetJointTargetVelocity(motorR, uRight, sim.sim_opmode_oneshot)
        # time.sleep(0.3)
        time.sleep(0.5)
        td = time.time() - ts
    else:

        ret, carpos = sim.GetObjectPosition(robot, -1, sim.sim_opmode_blocking)
        ret, carrot = sim.GetObjectOrientation(robot, -1, sim.sim_opmode_blocking)

        td = time.time() - ts

        xnew = pcix(td)
        ynew = pciy(td)
        
        xnewArr.append(xnew)
        ynewArr.append(ynew)

        errp = m.sqrt((xnew-carpos[0])**2 + (ynew-carpos[1])**2)
        angd = m.atan2(ynew-carpos[1], xnew-carpos[0])
        errh = angdiff(carrot[2], angd)
        #print('Distance to goal: {}   Heading error: {}'.format(errp, errh))

        v = Kv*errp
        omega = Kh*errh

        if v > 1:
            v = 0.5
        if omega > np.pi/2:
            omega = np.pi/4
        ur, ul = v2u(v, omega, r, L)
        errf = sim.SetJointTargetVelocity(motorL, ul, sim.sim_opmode_oneshot)
        errf = sim.SetJointTargetVelocity(motorR, ur, sim.sim_opmode_oneshot)
            
    xpos.append(carpos[0])
    ypos.append(carpos[1])
    tarr.append(td)
    # time.sleep(0.1)

for i in range(5):
    errf = sim.SetJointTargetVelocity(motorL, 0, sim.sim_opmode_streaming)
    errf = sim.SetJointTargetVelocity(motorR, 0, sim.sim_opmode_streaming)
    #time.sleep(0.1)

diff = np.diff(tarr)
diff = np.mean(diff)
print(f'TIME MEAN:{diff}')

# Graficar tiempo
plt.figure(1)
plt.plot(tarr, xpos)
plt.xlabel('t')
plt.ylabel('x')
plt.title('X - T')

# Graficar tiempo
plt.figure(2)
plt.plot(tarr, ypos)
plt.xlabel('t')
plt.ylabel('y')
plt.title('Y - T')


# Graficar desplazamiento
plt.figure(3)
plt.plot(xpos, ypos)
plt.xlabel('x')
plt.ylabel('y')
plt.title('Desplazamiento')
for x, y in zip(xarr, yarr):
    plt.plot(x, y, 'ro')
    plt.text(x+0.2, y+0.4, '%d, %d' % (int(x), int(y)))


#%%################
# Graficar Camino #
###################

plt.figure(4)
# xnew = pcix(td)
# ynew = pciy(td)
plt.plot(xnewArr, ynewArr, 'b')
# plt.plot(xarr, yarr, '.')
for x, y in zip(xarr, yarr):
    plt.plot(x, y, 'ro')
    plt.text(x+0.2, y+0.4, '%d, %d' % (int(x), int(y)))
plt.title('Path')

plt.figure(5)
plt.plot(xpos, ypos, 'm')
plt.plot(xnewArr, ynewArr, 'b')
# plt.plot(xarr, yarr, '.')
for x, y in zip(xarr, yarr):
    plt.plot(x, y, 'ro')
    plt.text(x+0.2, y+0.4, '%d, %d' % (int(x), int(y)))
plt.title('Path')
plt.show()

# #%%##############################
# # Graficar posicion y velocidad #
# #################################
# plt.figure(2)
# xdot = spi.splev(tnew, pcix, der=1)
# ydot = spi.splev(tnew, pciy, der=1)
# plt.plot(tnew, xnew, 'b', label='x')
# plt.plot(tnew, ynew, 'r', label='y')
# plt.plot(tnew, xdot, 'c', label='xdot')
# plt.plot(tnew, ydot, 'm', label='ydot')
# plt.plot(tarr, xarr, '.')
# plt.plot(tarr, yarr, '.')
# plt.legend()
# plt.title('Position and velocity over time')
# plt.show()

sim.StopSimulation(sim.sim_opmode_oneshot)
