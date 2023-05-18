# -*- coding: utf-8 -*-
"""
Created on Wed May 17 22:08:10 2023

@author: dany_
"""

function sysCall_init()
    -- do some initialization here
end

function sysCall_actuation()
    -- put your actuation code here
end

function sysCall_sensing()
    -- put your sensing code here
    
    -- This code snippet will move a cuboid primitive shape around the scene,
    -- checking for collisions and building an occupancy grid. The cuboid shape is
    -- assumed to be 0.1x0.1x1.0 m^3, with zero rotation.
    s = 0.1
    xmin = -2.5
    xmax = 2.5
    ymin = -2.5
    ymax = 2.5
    file = io.open ('C:/Users/dany_/OneDrive - Universidad de Guanajuato/Universidad/DECIMO SEGUNDO/ROB?TICA MOVIL/Pr?cticas/PROYECTOS GIT/RoboticaMovil2023/PROYECTO-NAVEGACI?N EN MAPAS DE REJILLA/test.txt', 'w')
    io.output(file)
    sens = sim.getObject('/CuboidSensor')
    for x = xmin, xmax, s do
        for y = ymin, ymax, s do
            sim.setObjectPosition(sens, -1, {x, y, 0.5})
            result, pairHandles = sim.checkCollision(sens, sim.handle_all)
            io.write(result .. ' ')
        end
        io.write('\n')
    end
    io.close(file)
    
    sim.stopSimulation()
end

function sysCall_cleanup()
    -- do some clean-up here
end

-- See the user manual or the available code snippets for additional callback functions and details
