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
import cbor

def q2R(x,y,z,w):
    R = np.zeros((3,3))
    R[0,0] = 1-2*(y**2+z**2)
    R[0,1] = 2*(x*y-z*w)
    R[0,2] = 2*(x*z+y*w)
    R[1,0] = 2*(x*y+z*w)
    R[1,1] = 1-2*(x**2+z**2)
    R[1,2] = 2*(y*z-x*w)
    R[2,0] = 2*(x*z-y*w)
    R[2,1] = 2*(y*z+x*w)
    R[2,2] = 1-2*(x**2+y**2)
    return R

def stop_moving():
    errf = sim.setJointTargetVelocity(motorL, 0)
    errf = sim.setJointTargetVelocity(motorR, 0)
    return errf

def front_sensors(): #lee el buffer de los sensores 3,4, y regresa un array de booleanos 
    sensores_state =  []
    for s in range(3,4): 
        state_temp, distance, point, detectedObj, _ = sim.readProximitySensor(usensor[s])
        #bresenham(state_temp, s, point)
        sensores_state.append(state_temp)
        if state_temp is True:
            break
    return sensores_state, point

'''
def bresenham(state, n_sensor, point):
    global occgrid, tocc, grid_cols, grid_rows, x_ajuste, y_ajuste, xr,yr
    srot = sim.getObjectQuaternion(usensor[n_sensor], -1)
    spos = sim.getObjectPosition(usensor[n_sensor], -1)
    R = q2R(srot[0], srot[1], srot[2], srot[3])
    spos = np.array(spos).reshape((3,1))
    
    if state == True:
        #posicion del objeto en el grid
        opos = np.array(point).reshape((3,1))
        pobs = np.matmul(R, opos) + spos
        xs = pobs[0]
        ys = pobs[1]
        xo = int(initial_cols/2 + m.ceil(xs/0.1)) + x_ajuste
        yo = int(initial_rows/2 - m.floor(ys/0.1)) + y_ajuste        
        if xo >= grid_cols:
            extra_cols = 0.5*np.ones((grid_rows,10)) #creamos las columnas que se agregarán al mapa
            occgrid = np.append(occgrid,extra_cols,1) #extendemos el occgrid y tocc
            extra_cols = np.zeros((grid_rows,10))
            tocc = np.append(tocc,extra_cols,1)#0 = row 1 = col
            grid_cols += 10
            position_in_grid()
            print("SE EXPANDE A LA DERECHA EN: ", grid_cols)
            plot_map()
        if xo <=0:
            extra_cols = 0.5*np.ones((grid_rows,10))
            occgrid = np.append(extra_cols,occgrid,1) #0 = row 1 = col
            extra_cols = np.zeros((grid_rows,10))
            tocc = np.append(extra_cols,tocc,1)
            grid_cols += 10
            x_ajuste += 10 
            position_in_grid()  
            print("SE EXPANDE A LA IZQUIERDA EN: ", grid_cols)  
            plot_map()
            xo +=10
        if yo >= grid_rows:
            extra_rows = 0.5*np.ones((10,grid_cols))
            occgrid = np.append(occgrid,extra_rows,0) #0 = row 1 = col
            extra_rows = np.zeros((10,grid_cols))
            tocc = np.append(tocc,extra_rows,0)
            grid_rows += 10
            position_in_grid()
            print("SE EXPANDE ABAJO EN: ", grid_rows)
            plot_map()
        if yo <= 0:
            extra_rows = 0.5*np.ones((10,grid_cols))
            occgrid = np.append(extra_rows,occgrid,0) #0 = row 1 = col
            extra_rows = np.zeros((10,grid_cols))
            tocc = np.append(extra_rows,tocc,0)
            grid_rows += 10
            y_ajuste += 10
            # print("Expanded up")
            position_in_grid()
            print("SE EXPANDE ARRIBA EN: ", grid_rows)
            plot_map()
            yo +=10
        #Line dibuja una linea en el grid y encuentra cuales celdas ocupa en el grid
        # y pone en 0 si no ve obstaculo y un 1 en el punto 
        rows, cols = line(yr-1, xr-1, yo-1, xo-1)
        occgrid[rows, cols] = 0
        tocc[yo-1, xo-1] = 1
    else:
        opos = np.array([0,0,1]).reshape((3,1))
        pobs = np.matmul(R, opos) + spos
        xs = pobs[0]
        ys = pobs[1]
        xo = int(initial_cols/2 + m.ceil(xs/0.1)) + x_ajuste
        yo = int(initial_rows/2 - m.floor(ys/0.1)) + y_ajuste
        if xo >= grid_cols:
            
            extra_cols = 0.5 * np.ones((occgrid.shape[0], 10))
            occgrid = np.append(occgrid, extra_cols, 1)
            extra_cols = np.zeros((tocc.shape[0], 10))
            tocc = np.append(tocc, extra_cols, 1)
            
            #extra_cols = 0.5*np.ones((grid_rows,10)) #creamos las columnas que se agregarán al mapa
            #occgrid = np.append(occgrid,extra_cols,1) #extendemos el occgrid y tocc
            #extra_cols = np.zeros((grid_rows,10))
            #tocc = np.append(tocc,extra_cols,1)#0 = row 1 = col
            grid_cols += 10
            position_in_grid()
            print("SE EXPANDE A LA DERECHA EN: ", grid_cols)
            plot_map()
        if xo <=0:
            
            extra_cols = 0.5 * np.ones((occgrid.shape[0], 10))
            occgrid = np.append(extra_cols, occgrid, 1)
            extra_cols = np.zeros((tocc.shape[0], 10))
            tocc = np.append(extra_cols, tocc, 1)
            
            #extra_cols = 0.5*np.ones((grid_rows,10))
            #occgrid = np.append(extra_cols,occgrid,1) #0 = row 1 = col
            #extra_cols = np.zeros((grid_rows,10))
            #tocc = np.append(extra_cols,tocc,1)
            grid_cols += 10
            x_ajuste += 10 
            position_in_grid() 
            print("SE EXPANDE A LA IZQUIERDA EN: ", grid_cols)     
            plot_map() 
            xo += 10
        if yo >= grid_rows:
            
            extra_cols = 0.5 * np.ones((10, occgrid.shape[0]))
            occgrid = np.append(occgrid, extra_cols, 0)
            extra_cols = np.zeros((10, tocc.shape[0]))
            tocc = np.append(tocc, extra_cols, 0)
            
            #extra_rows = 0.5*np.ones((10,grid_cols))
            #occgrid = np.append(occgrid,extra_rows,0) #0 = row 1 = col
            #extra_rows = np.zeros((10,grid_cols))
            #tocc = np.append(tocc,extra_rows,0)
            grid_rows += 10
            position_in_grid()
            print("SE EXPANDE ABAJO EN: ", grid_rows)
            plot_map()
        if yo <= 0:
            
            extra_cols = 0.5 * np.ones((10, occgrid.shape[0]))
            occgrid = np.append(extra_cols, occgrid, 0)
            extra_cols = np.zeros((10, tocc.shape[0]))
            tocc = np.append(extra_cols, tocc, 0)
            
            #extra_rows = 0.5*np.ones((10,grid_cols))
            #occgrid = np.append(extra_rows,occgrid,0) #0 = row 1 = col
            #extra_rows = np.zeros((10,grid_cols))
            #tocc = np.append(extra_rows,tocc,0)
            #grid_rows += 10
            y_ajuste += 10
            # print("Expanded up")
            position_in_grid()
            print("SE EXPANDE ARRIBA EN: ", grid_rows)
            plot_map()
            yo += 10
        rows, cols = line(yr-1, xr-1, yo-1, xo-1)
        occgrid[rows, cols] = 0
    return
'''

def position_in_grid():
    
    global xr,yr, grid_cols, grid_rows, initial_cols, initial_rows
    carpos = sim.getObjectPosition(robot, -1)
    carrot = sim.getObjectOrientation(robot, -1)
    #get robot position of robot in grid
    xw = carpos[0]
    yw = carpos[1]
    xr = int(initial_cols/2 + m.ceil(xw/0.1)) + x_ajuste
    if xr >= grid_cols:
        xr = grid_cols-1
    yr = int(initial_rows/2 - m.floor(yw/0.1)) + y_ajuste       
    if yr >= grid_rows:
        yr = grid_rows-1
    return carpos,carrot

def plot_map():
    global tocc, occgrid
    plt.imshow(tocc+occgrid)
    plt.show()
    
def esquivar():
    global uread,upt,ustate
    for i in range(16): 
        state, distance, point, detectedObj, _ = sim.readProximitySensor(usensor[i])
        #bresenham(state, i, point)
        uread.append(np.linalg.norm(point))
        upt.append(point)
        ustate.append(state)
        #dirección default de esquivo DERECHA
        # vel_motor = np.array([1,-1]) #velocidad del motor L y motor R
        # sensor_ancla = 15
        if state and np.linalg.norm(point)<0.6 and i in [1,2]:# 4, 5  detectan algo, comienza la evitación de obstáculos
              print("obstaculo en sensor: ", i)
              plot_map()
              errf = stop_moving()
              while state is False: #gira hasta que el sensor que detectó el obtejo deje de verlo 
                  state, distance, point, detectedObj, _ = sim.readProximitySensor(usensor[i])
                  #bresenham(state,i,point)
                  errf = sim.setJointTargetVelocity(motorL, 0.7)
                  errf = sim.setJointTargetVelocity(motorR, -0.7)
              errf = stop_moving()
              sensores =  front_sensors()
              while False in sensores:
                  sensores =  front_sensors()
                  errf = sim.setJointTargetVelocity(motorL, 0.7)
                  errf = sim.setJointTargetVelocity(motorR, -0.7)
              errf = stop_moving()
              t_giro = time.time()
              while time.time()-t_giro < 0.5:
                  sensores =  front_sensors()
                  if False in sensores:
                      break
                  errf = sim.setJointTargetVelocity(motorL, 0.7)
                  errf = sim.setJointTargetVelocity(motorR, -0.7)
              while time.time()-t_giro < 1:
                  sensores =  front_sensors()
                  if False in sensores:
                      break
                  errf = sim.setJointTargetVelocity(motorL, 1)
                  errf = sim.setJointTargetVelocity(motorR, 1)
              stop_moving()
              
        #esquivar por la derecha       
        if state and np.linalg.norm(point)<0.6 and i in [5,6]:# 2, 3 detectan algo, comienza la evitación de obstáculos
              print("obstaculo en sensor: ", i)
              errf = stop_moving()
              while state is False:
                  state, distance, point, detectedObj, _ = sim.readProximitySensor(usensor[i])
                  #bresenham(state, i, point)
                  errf = sim.setJointTargetVelocity(motorL, -0.7)
                  errf = sim.setJointTargetVelocity(motorR, 0.7)
              errf = stop_moving()
              #ya que giramos hasta que el sensor 9 vio el obstaculo, checamos los sensores 3-6
              sensores =  front_sensors()
              #Si el sensor 9 y 4, 5, 6 o 7 están detectando algo al mismo tiempo, este ciclo while moverá el robot hacia atrás hasta que algunos de los sensores no detecten nada.
              while False in sensores:
                  sensores =  front_sensors()
                  errf = sim.setJointTargetVelocity(motorL, -0.7)
                  errf = sim.setJointTargetVelocity(motorR, 0.7)
              errf = stop_moving()
              t_giro = time.time()
              while time.time()-t_giro < 0.5:
                  sensores =  front_sensors()
                  if False in sensores:
                      break
                  errf = sim.setJointTargetVelocity(motorL, -0.7)
                  errf = sim.setJointTargetVelocity(motorR, 0.7)
              while time.time()-t_giro < 1:
                  sensores =  front_sensors()
                  if False in sensores:
                      break
                  errf = sim.setJointTargetVelocity(motorL, 1)
                  errf = sim.setJointTargetVelocity(motorR, 1)
              stop_moving()
              
        #esquivar por la derecha       
        if state and np.linalg.norm(point)<0.6 and i in [3,4]:# 2, 3 detectan algo, comienza la evitación de obstáculos
              print("obstaculo en sensor: ", i)
              errf = stop_moving()
              while state is True:
                  state, distance, point, detectedObj, _ = sim.readProximitySensor(usensor[i])
                  #bresenham(state, i, point)
                  errf = sim.setJointTargetVelocity(motorL, -0.7)
                  errf = sim.setJointTargetVelocity(motorR, -0.7)
              errf = stop_moving()
              t_giro = time.time()
              while time.time()-t_giro < 1:
                  sensores =  front_sensors()
                  if False in sensores:
                      break
                  errf = sim.setJointTargetVelocity(motorL, -0.7)
                  errf = sim.setJointTargetVelocity(motorR, 0.7)
              stop_moving()
    return

'''
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

# Definir una función que convierta la velocidad lineal y angular del robot en las velocidades de los motores
def v2u(v, omega, r, L):
    return (v/r + L*omega/(2*r), v/r - L*omega/(2*r))

# Definir una función que controle los motores del robot para moverlo a una velocidad y una velocidad angular durante un tiempo determinado
def MovR(sim, motorL, motorR, v, omega, duration):
    end_time = sim.getSimulationTime() + duration
    while sim.getSimulationTime() < end_time:
        sim.setJointTargetVelocity(motorL, v2u(v, omega, R, L)[1])
        sim.setJointTargetVelocity(motorR, v2u(v, omega, R, L)[0])
'''
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
'''
# ------ INICIO DE LECTURA Y CREACIÓN DE RUTA ---------

client = RemoteAPIClient()
sim = client.getObject('sim')

motorL=sim.getObject('/PioneerP3DX/leftMotor')
motorR=sim.getObject('/PioneerP3DX/rightMotor')
robot = sim.getObject('/PioneerP3DX')

sim.startSimulation()

# Assigning handles to the ultrasonic sensors
usensor = []
Rs = []
Vs = []
for i in range(0,16):
    s = sim.getObject('/PioneerP3DX/ultrasonicSensor['+str(i)+']')
    usensor.append(s)
    q = sim.getObjectQuaternion(s, robot)
    Rs.append(q2R(q[0], q[1], q[2], q[3]))
    Vs.append(np.reshape(sim.getObjectPosition(s, robot), (3,1)))

carpos = sim.getObjectPosition(robot, -1)
carrot = sim.getObjectOrientation(robot, -1)

Kv = 0.5
Kh = 2.5
xd = 3
yd = 3
hd = 0
r = 0.1
L = 0.2
errp = 10
    # Tamaño del mapa de ocupación
initial_cols = 150
initial_rows = 150
grid_cols = initial_cols
grid_rows = initial_rows
x_ajuste = 0 #por si se expande hacia la izquierda
y_ajuste = 0 #por si se expande hacia arriba 
xr = 0
yr = 0

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

t = time.time()

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

# ------ FIN DE LECTURA Y CREACIÓN DE RUTA ---------

initt = t
niter = 0
while time.time()-t < 10:
    carpos, carrot = position_in_grid()

    carrot = sim.getObjectQuaternion(robot, -1)

    uread = []
    ustate = []
    upt = []
    etime = []
    for i in range(0,16,2):
        
        state, distance, point, detectedObj, _ = sim.readProximitySensor(usensor[i])
        
        uread.append(distance)
        upt.append(point)
        ustate.append(state)

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
        for i in range(len(rr)):
            # Obtener la posición actual del robot
            x, y, _ = sim.getObjectPosition(robot, -1)
            coords.append((x, y))
        
            # Obtener la dirección del robot
            _, _, theta = sim.getObjectOrientation(robot, -1)
        
            # Obtener el punto actual del spline
            x_target, y_target = rr[i], cc[i]
        
            # Calcular la distancia al punto actual
            distance_to_target = np.sqrt((x_target - x)**2 + (y_target - y)**2)
        
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
            
            esquivar()
            front_sensors()
        niter = niter + 1

finalt = time.time()
print('Avg time per iteration ', (finalt-initt)/niter)

sim.setJointTargetVelocity(motorL, 0)
sim.setJointTargetVelocity(motorR, 0)
            
sim.stopSimulation()
'''
# Graficar la trayectoria del robot
x, y = zip(*coords)
ax.plot(x, y, ".", ls="-", c="g")
ax.plot(x[0], y[0], "X", c="r")
plt.plot(rr, cc)
plt.plot(rr, cc, '.')
ax.set_title("TRAYECTORIA")
plt.show()
'''