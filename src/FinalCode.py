from controller import Robot
import sys
import numpy as np
import cv2 as cv
#REMEMBER TO COPY-PASTE THIS FUNCTIONS ON TO FINAL CODE
sys.path.append(r"C:\\Users\\ANA\\Desktop\\Webots - Erebus\\Mini challenge 2020\\SimulationDemonstration-2021-MiniChallenge\\Participants\\Alejandro")
from AbstractionLayer import AbstractionLayer
from StateMachines import StateManager
timeStep = 16 * 2 


stMg = StateManager("init")
r = AbstractionLayer()


# While the simulation is running
while r.doLoop():
    # Update the robot
    r.update()
    print("rotation: " + str(r.rotation))
    print("position: " + str(r.position))
    print("State:", stMg.state)
    

    if stMg.checkState("init"):
        if r.calibrate():
            stMg.changeState("move")
    
    if stMg.checkState("move"):
        r.seqMg.startSequence()
        r.seqRotateToDegs(90)
        r.seqMoveWheels(0, 0)
        r.seqDelaySec(1)
        r.seqRotateToDegs(180)
        r.seqMoveWheels(0, 0)
        r.seqDelaySec(1)
        r.seqRotateToDegs(270)
        r.seqMoveWheels(0, 0)
        r.seqDelaySec(1)
        r.seqRotateToDegs(360)
        r.seqMoveWheels(0, 0)
        r.seqDelaySec(1)
        r.seqResetSequence()


    print("--------------------------------------------------------------------")