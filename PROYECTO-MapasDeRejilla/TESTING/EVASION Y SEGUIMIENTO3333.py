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

nSensor = 4
sensors = [sim.getObject(f"/PioneerP3DX/ultrasonicSensor[{i}]") for i in range (nSensor)]

def RSens (sim, sensor):
    maxD = 0.8
    detect = False
    LAct = False
    RAct = False
    detected = [0] * len(sensors)
    act_l = [0] * len(sensor)
    vel = [0] * len(sensors)
    for i in range(len(sensor)):
        result, distance, _, _, _ = sim.readProximitySensor(sensors[i])
        if result and distance < maxD:
            detect = True
            if i < len(sensors)/2:
                LAct = True
            else:
                RAct = True
            detected[i] = distance
            act_l[i] = 1
            vel[i] = 1 - ((distance - 0.2) / (maxD - 0.2))
            if distance < 0.2:
                vel[i] = 2
    return detect, detected, LAct, RAct, act_l, vel        


# Definir la rutina de evasión de obstáculos
def evadir_obstaculo(sim, motorL, motorR):
    # Detener el robot
    sim.setJointTargetVelocity(motorL, 0)
    sim.setJointTargetVelocity(motorR, 0)
    
    # Girar el robot hacia la derecha
    sim.setJointTargetVelocity(motorL, -0.5)
    sim.setJointTargetVelocity(motorR, 0.5)
    
    # Esperar a que el robot gire lo suficiente
    time.sleep(0.5)
    
    # Detener el robot
    sim.setJointTargetVelocity(motorL, 0)
    sim.setJointTargetVelocity(motorR, 0)
    
    # Mover el robot hacia adelante
    sim.setJointTargetVelocity(motorL, 0.5)
    sim.setJointTargetVelocity(motorR, 0.5)
    
    # Esperar a que el robot evada el obstáculo
    time.sleep(1)
    
    # Detener el robot
    sim.setJointTargetVelocity(motorL, 0)
    sim.setJointTargetVelocity(motorR, 0)

sim.startSimulation()

# Definir el radio de las ruedas y la distancia entre ellas para el cálculo de la velocidad de los motores
R, L = 0.0975, 0.331

# Guardar la posición inicial del robot
x, y, _ = sim.getObjectPosition(robot, -1)
coords = [(x, y)]

fig, ax = plt.subplots() #Figura del trayecto
move, rotate = .7, .7 #Avance y rotación
'''
-x-x-x-x-x-x-x-x-x-x-x-x-x
INICIO DE TRAYECTORIA
-x-x-x-x-x-x-x-x-x-x-x-x-x
'''
    
# Seguir la trayectoria con el robot
for i in range(len(xnew)):
    # Obtener la posición actual del robot
    x, y, _ = sim.getObjectPosition(robot, -1)
    coords.append((x, y))

    # Obtener la dirección del robot
    _, _, theta = sim.getObjectOrientation(robot, -1)

    # Obtener el punto actual del spline
    x_target, y_target = xnew[i], ynew[i]

    # Calcular la distancia al punto actual
    distance_to_target = np.sqrt((x_target - x)**2 + (y_target - y)**2)

    # Si hay un obstáculo cerca, evadirlo
    detectado, _, izquierda, derecha, _, _ = RSens(sim, sensors)
    if detectado:
        if izquierda:
            evadir_obstaculo(sim, motorL, motorR)
        elif derecha:
            evadir_obstaculo(sim, motorR, motorL)

    # Si el robot está lo suficientemente cerca del punto actual, avanzar al siguiente
    if distance_to_target < 0.0001:
        if i < len(xnew)-1:
            x_target, y_target = xnew[i+1], ynew[i+1]
        else:
            x_target, y_target = xnew[-1], ynew[-1]

    # Calcular el ángulo de dirección hacia el siguiente punto
    target_angle = np.arctan2(y_target - y, x_target - x)

    # Calcular la velocidad lineal y angular del robot
    delta_angle = target_angle - theta
    if delta_angle > np.pi:
        delta_angle -= 2 * np.pi
    elif delta_angle < -np.pi:
        delta_angle += 2 * np.pi

    # Ajustar la velocidad lineal y angular del robot
    v = move
    omega = rotate * delta_angle

    # mover el robot durante un breve período de tiempo
    MovR(sim, motorL, motorR, v, omega, 0.1)
    
    x, y, _ = sim.getObjectPosition(robot, -1) #Guardado de coordenadas
    coords.append((x, y))

    '''
    -x-x-x-x-x-x-x-x-x
    FIN DE TRAYECTORIA
    -x-x-x-x-x-x-x-x-x
    '''
    
    
# Pausar y detener la simulación
sim.pauseSimulation()
sim.stopSimulation()

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
