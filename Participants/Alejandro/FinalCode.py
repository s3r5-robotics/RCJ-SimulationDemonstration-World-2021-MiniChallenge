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
    

    if not stMg.checkState("init"):
        if r.isEnded():
            r.seqResetSequence()
            stMg.changeState("end")


    if stMg.checkState("init"):
        if r.calibrate():
            stMg.changeState("followBest")
    
    if stMg.checkState("stop"):
        r.seqMg.startSequence()
        r.seqMoveWheels(0, 0)

    if stMg.checkState("moveForward"):
        r.seqMg.startSequence()
        r.seqMoveWheels(0.5, -0.5)
        r.seqDelaySec(0.1)
        r.seqMoveWheels(-0.5, 0.5)
        r.seqDelaySec(0.1)
        r.seqResetSequence()

    if stMg.checkState("followBest"):
        r.seqMg.startSequence()
        bestPos = r.getBestPos()
        if bestPos is not None:
            r.seqMoveToCoords(bestPos)
        r.seqResetSequence()

        if r.isTrap:
            r.seqResetSequence()
            stMg.changeState("hole")
    
    if stMg.checkState("hole"):
        r.seqMg.startSequence()
        r.seqMoveWheels(-0.5, -0.5)
        r.seqDelaySec(0.5)
        r.seqMoveWheels(0, 0)
        if r.seqMg.simpleSeqEvent(): r.recalculatePath()
        r.seqResetSequence()
        stMg.changeState("followBest")
    
    if stMg.checkState("end"):
        r.seqMg.startSequence()
        if r.seqMg.simpleSeqEvent(): r.endGame()
        r.seqMoveWheels(0, 0)
        
    print("--------------------------------------------------------------------")