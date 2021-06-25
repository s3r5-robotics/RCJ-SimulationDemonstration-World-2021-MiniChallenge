import math
import numpy as np
from controller import *
import sys
import struct
import time
import cv2 as cv
import copy





# Corrects the given angle in degrees to be in a range from 0 to 360
def normalizeDegs(ang):
    ang = ang % 360
    if ang < 0:
        ang += 360
    if ang == 360:
        ang = 0
    return ang


# Corrects the given angle in radians to be in a range from 0 to a full rotaion
def normalizeRads(rad):
    ang = radsToDegs(rad)
    normAng = normalizeDegs(ang)
    return degsToRads(normAng)


# Converts from degrees to radians
def degsToRads(deg):
    return deg * math.pi / 180


# Converts from radians to degrees
def radsToDegs(rad):
    return rad * 180 / math.pi


# Converts a number from a range of value to another
def mapVals(val, in_min, in_max, out_min, out_max):
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


# Gets x, y coordinates from a given angle in radians and distance
def getCoordsFromRads(rad, distance):
    y = float(distance * math.cos(rad))
    x = float(distance * math.sin(rad))
    return (x, y)


# Gets x, y coordinates from a given angle in degrees and distance
def getCoordsFromDegs(deg, distance):
    rad = degsToRads(deg)
    y = float(distance * math.cos(rad))
    x = float(distance * math.sin(rad))
    return (x, y)


def getRadsFromCoords(coords):
    return math.atan2(coords[0], coords[1])


def getDegsFromCoords(coords):
    rads = math.atan2(coords[0], coords[1])
    return radsToDegs(rads)


# Gets the distance to given coordinates
def getDistance(position):
    return math.sqrt((position[0] ** 2) + (position[1] ** 2))


# Checks if a value is between two values
def isInRange(val, minVal, maxVal):
    return minVal < val < maxVal


def roundDecimal(number, decimal):
    return (round(number * decimal) / decimal)


def multiplyLists(list1, list2):
    finalList = []
    for item1, item2 in zip(list1, list2):
        finalList.append(item1 * item2)
    return finalList


def sumLists(list1, list2):
    finalList = []
    for item1, item2 in zip(list1, list2):
        finalList.append(item1 + item2)
    return finalList


def substractLists(list1, list2):
    finalList = []
    for item1, item2 in zip(list1, list2):
        finalList.append(item1 - item2)
    return finalList


def divideLists(list1, list2):
    finalList = []
    for item1, item2 in zip(list1, list2):
        finalList.append(item1 / item2)
    return finalList

def imageSetUp(camera):
    imageResult = camera.getImage()
    imageResult = np.frombuffer(imageResult, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    imageResult = np.array(imageResult ,dtype=np.uint8)
    return imageResult
# Manages states
class StateManager:
    def __init__(self, initialState):
        self.state = initialState

    # Sets the state to a certain value
    def changeState(self, newState):
        self.state = newState
        return True

    # Checks if the state corresponds to a specific value
    def checkState(self, state):
        return self.state == state


# Makes it possible to run arbitrary code sequentially without interrupting other code that must run continuoulsy
class SequenceManager:
    def __init__(self):
        self.lineIdentifier = 0
        self.linePointer = 1
        self.done = False

    # Resets the sequence and makes it start from the first event
    def resetSequence(self):
        self.linePointer = 1
        print("----------------")
        print("reseting sequence")
        print("----------------")

    def seqResetSequence(self):
        if self.check():
            self.resetSequence()

            return True
        return False

    # This has to be at the start of any sequence of events
    def startSequence(self):
        self.lineIdentifier = 0
        self.done = False

    # Returns if the line pointer and identifier match and increases the identifier
    # Must be included at the end of any sequential function
    def check(self):
        self.done = False
        self.lineIdentifier += 1
        return self.lineIdentifier == self.linePointer

    # Changes to the next event
    def nextSeq(self):
        self.linePointer += 1
        self.done = True

    # returns if the sequence has reached its end
    def seqDone(self):
        return self.done

    # Can be used to make a function sequential or used in an if statement to make a code block sequential
    def simpleSeqEvent(self, function=None, *args, **kwargs):
        if self.check():
            if function is not None:
                function(*args, **kwargs)
            self.nextSeq()
            return True
        return False

    # The function inputted must return True when it ends
    def complexSeqEvent(self, function, *args, **kwargs):
        if self.check():
            if function(*args, **kwargs):
                self.nextSeq()
                return True
        return False

    # When inpuuted any function it returns a sequential version of it that can be used in a sequence
    def makeSimpleSeqEvent(self, function):
        def event(*args, **kwargs):
            if self.check():
                function(*args, **kwargs)
                self.nextSeq()
                return True
            return False

        return event

    # When inputted a function that returns True when it ends returns a sequential version of it that can be used in a sequence
    def makeComplexSeqEvent(self, function):
        def event(*args, **kwargs):
            if self.check():
                if function(*args, **kwargs):
                    self.nextSeq()
                    return True
            return False

        return event






# REMEMBER TO COPY-PASTE THIS FUNCTIONS ON TO FINAL CODE
# v Put files directory here v

# Imports all utility functions




# Captures images and processes them
class Camera:
    def __init__(self, camera, timeStep):
        self.camera = camera
        self.camera.enable(timeStep)
        self.height = self.camera.getHeight()
        self.width = self.camera.getWidth()
        self.distanceSensor = DistanceSensor()
    def getImg(self):
        imageData = self.camera.getImage()
        return np.array(np.frombuffer(imageData, np.uint8).reshape((self.height, self.width, 4)))

    def getDistance(self):
        self.distanceSensor.distanceCalculation(self.getImg())
    # Gets an image from the raw camera data





# Tracks global rotation
class Gyroscope:
    def __init__(self, gyro, index, timeStep):
        self.sensor = gyro
        self.sensor.enable(timeStep)
        self.oldTime = 0.0
        self.index = index
        self.rotation = 0
        self.lastRads = 0

    # Do on every timestep
    def update(self, time):
        # print("Gyro Vals: " + str(self.sensor.getValues()))
        timeElapsed = time - self.oldTime  # Time passed in time step
        radsInTimestep = (self.sensor.getValues())[self.index] * timeElapsed
        self.lastRads = radsInTimestep
        finalRot = self.rotation + radsInTimestep
        self.rotation = normalizeRads(finalRot)
        self.oldTime = time

    # Gets the actual angular Velocity
    def getDiff(self):
        if self.lastRads < 0:
            return self.lastRads * -1

        return self.lastRads

    # Returns the rotation on degrees
    def getDegrees(self):
        return radsToDegs(self.rotation)

    # Returns the rotation on radians
    def getRadians(self):
        return self.rotation

    # Sets the rotation in radians
    def setRadians(self, rads):
        self.rotation = rads

    # Sets the rotation in degrees
    def setDegrees(self, degs):
        self.rotation = degsToRads(degs)


# Tracks global rotation
class Accelerometer:
    def __init__(self, accelerometer, indexes, timeStep):
        self.sensor = accelerometer
        self.sensor.enable(timeStep)
        self.oldTime = 0.0
        self.indexes = indexes
        self.position = [0, 0, 0]
        self.speedInTimeStep = [0, 0, 0]
        self.lastPos = []

    # Do on every timestep
    def update(self, time):
        timeElapsed = time - self.oldTime  # Time passed in time step
        accValues = substractLists(self.sensor.getValues(), [0, 9.81, 0])
        accValues = multiplyLists(accValues, [timeElapsed, timeElapsed, timeElapsed])
        self.speedInTimeStep = sumLists(accValues, self.speedInTimeStep)
        print("Acc values:" + str(
            [roundDecimal(self.speedInTimeStep[0], 10000), roundDecimal(self.speedInTimeStep[2], 10000)]))
        posInTimeStep = multiplyLists(self.speedInTimeStep, [timeElapsed, timeElapsed,
                                                             timeElapsed])  # [timeElapsed ** 2, timeElapsed ** 2, timeElapsed ** 2])
        self.lastPos = []
        for i in posInTimeStep:
            self.lastPos.append(roundDecimal(i, 1000))
        self.position = sumLists(self.position, self.lastPos)
        self.oldTime = time

    # Returns the position in meters
    def getPos(self):
        return [self.position[0], self.position[2]]

    # Sets the postion in meters
    def setPos(self, pos):
        self.postion = pos


# Controlls a wheel
class Wheel:
    def __init__(self, wheel, maxVelocity):
        self.maxVelocity = maxVelocity
        self.wheel = wheel
        self.velocity = 0
        self.diameter = 0.0205 * 2
        self.circumference = math.pi * self.diameter
        self.wheel.setPosition(float("inf"))
        self.wheel.setVelocity(0)

    # Moves the wheel at a ratio of the maximum speed (between 0 and 1)
    def move(self, ratio):
        if ratio > 1:
            ratio = 1
        elif ratio < -1:
            ratio = -1
        self.velocity = ratio * self.maxVelocity
        self.wheel.setVelocity(self.velocity)

    def getLinearVelocity(self):
        return (self.velocity / (2 * math.pi)) * self.circumference




# Reads the colour sensor
class ColourSensor:
    def __init__(self, sensor, distancefromCenter, timeStep):
        self.distance = distancefromCenter
        self.sensor = sensor
        self.sensor.enable(timeStep)
        self.r = 0
        self.g = 0
        self.b = 0

    def getPosition(self, robotGlobalPosition, robotGlobalRotation):
        realPosition = getCoordsFromDegs(robotGlobalRotation, self.distance)
        return [robotGlobalPosition[0] + realPosition[0], robotGlobalPosition[1] + realPosition[1]]

    def __update(self):
        colour = self.sensor.getImage()
        # print("Colourimg:", colour)
        self.r = self.sensor.imageGetRed(colour, 1, 0, 0)
        self.g = self.sensor.imageGetGreen(colour, 1, 0, 0)
        self.b = self.sensor.imageGetBlue(colour, 1, 0, 0)
        # print("Colour:", self.r, self.g, self.b)

    def __isTrap(self):
        return (35 < self.r < 45 and 35 < self.g < 45)

    def __isSwamp(self):
        return (200 < self.r < 210 and 165 < self.g < 175 and 95 < self.b < 105)

    def __isCheckpoint(self):
        return (self.r > 232 and self.g > 232 and self.b > 232)

    def __isNormal(self):
        return self.r == 227 and self.g == 227

    def __isBlue(self):
        return (55 < self.r < 65 and 55 < self.g < 65 and 245 < self.b < 255)

    def __isPurple(self):
        return (135 < self.r < 145 and 55 < self.g < 65 and 215 < self.b < 225)

    def __isRed(self):
        return (245 < self.r < 255 and 55 < self.g < 65 and 55 < self.b < 65)

    # Returns the type of tyle detected from the colour data
    def getTileType(self):
        self.__update()
        tileType = "undefined"
        if self.__isNormal():
            tileType = "normal"
        elif self.__isTrap():
            tileType = "hole"
        elif self.__isSwamp():
            tileType = "swamp"
        elif self.__isCheckpoint():
            tileType = "checkpoint"
        elif self.__isBlue():
            tileType = "connection1-2"
        elif self.__isPurple():
            tileType = "connection2-3"
        elif self.__isRed():
            tileType = "connection1-3"

        # print("Color: " + tileType)
        # print("r: " + str(self.r) + "g: " + str(self.g) + "b: " +  str(self.b))
        return tileType


# Emmiter and reciever
class Comunicator:
    def __init__(self, emmiter, receiver, timeStep):
        self.receiver = receiver
        self.emmiter = emmiter
        self.receiver.enable(timeStep)
        self.lackOfProgress = False
        self.doGetWordInfo = True
        self.gameScore = 0
        self.remainingTime = 0

    def sendVictim(self, position, victimtype):
        self.doGetWordInfo = False
        letter = bytes(victimtype, "utf-8")
        position = multiplyLists(position, [100, 100])
        position = [int(position[0]), int(position[1])]
        message = struct.pack("i i c", position[0], position[1], letter)
        self.emmiter.send(message)

    def sendLackOfProgress(self):
        self.doGetWordInfo = False
        message = struct.pack('c', 'L'.encode())  # message = 'L' to activate lack of progress
        self.emmiter.send(message)

    def sendEndOfPlay(self):
        self.doGetWordInfo = False
        exit_mes = struct.pack('c', b'E')
        self.emmiter.send(exit_mes)

        print("Ended!!!!!")

    def sendMap(self, npArray):
        ## Get shape
        print(npArray)
        s = npArray.shape
        ## Get shape as bytes
        s_bytes = struct.pack('2i', *s)
        ## Flattening the matrix and join with ','
        flatMap = ','.join(npArray.flatten())
        ## Encode
        sub_bytes = flatMap.encode('utf-8')
        ## Add togeather, shape + map
        a_bytes = s_bytes + sub_bytes
        ## Send map data
        self.emmiter.send(a_bytes)
        # STEP3 Send map evaluate request
        map_evaluate_request = struct.pack('c', b'M')
        self.emmiter.send(map_evaluate_request)
        self.doGetWordInfo = False

    def requestGameData(self):
        if self.doGetWordInfo:
            message = struct.pack('c', 'G'.encode())  # message = 'G' for game information
            self.emmiter.send(message)  # send message

    def update(self):

        if self.doGetWordInfo:
            """
            self.requestGameData()
            if self.receiver.getQueueLength() > 0: # If receiver queue is not empty
                receivedData = self.receiver.getData()
                if len(receivedData) > 2:
                    tup = struct.unpack('c f i', receivedData) # Parse data into char, float, int
                    if tup[0].decode("utf-8") == 'G':
                        self.gameScore = tup[1]
                        self.remainingTime = tup[2]
                        self.receiver.nextPacket() # Discard the current data packet
            """

            # print("Remaining time:", self.remainingTime)
            self.lackOfProgress = False
            if self.receiver.getQueueLength() > 0:  # If receiver queue is not empty
                receivedData = self.receiver.getData()
                print(receivedData)
                if len(receivedData) < 2:
                    tup = struct.unpack('c', receivedData)  # Parse data into character
                    if tup[0].decode("utf-8") == 'L':  # 'L' means lack of progress occurred
                        print("Detected Lack of Progress!")
                        self.lackOfProgress = True
                    self.receiver.nextPacket()  # Discard the current data packetelse:
        else:
            self.doGetWordInfo = True

class Odometry:
    def __init__(self, timeStep):
        self.timeStep = timeStep
        self.oldTime = 0
        self.position = [0, 0]


    def update(self, time, rotation, robotVelocity):
        timeElapsed = time - self.oldTime
        print("Time elapsed:", timeElapsed)
        distanceTraveled = robotVelocity * timeElapsed
        positionInTimestep = list(getCoordsFromDegs(rotation, distanceTraveled))
        print("pos in time Step:", positionInTimestep)
        self.position = [self.position[0] + positionInTimestep[0], self.position[1] + positionInTimestep[1]] #sumLists(self.position, positionInTimestep)
        print("odom Position:", self.position)
        self.oldTime = time

    def getPosition(self):
        return tuple(self.position)


# Abstraction layer for robot
class RobotLayer:
    def __init__(self, timeStep):

        # Important variables
        self.maxWheelSpeed = 6.28
        self.timeStep = timeStep
        # robot
        self.robot = Robot()

        # Location and orientation variables
        self.prevRotation = 0
        self.rotation = 0
        self.globalPosition = [0, 0]
        self.prevGlobalPosition = [0, 0]
        self.positionOffsets = [0, 0]
        self.time = 0

        # Flags for functions
        self.rotateToDegsFirstTime = True
        self.delayFirstTime = True

        # Sensors and actuators
        self.gyroscope = Gyroscope(self.robot.getDevice("gyro"), 1, self.timeStep)
        self.accelerometer = Accelerometer(self.robot.getDevice("accelerometer"), [0, 1, 2], self.timeStep)
        self.leftWheel = Wheel(self.robot.getDevice("wheel1 motor"), self.maxWheelSpeed)
        self.odometry = Odometry(self.timeStep)
        self.rightWheel = Wheel(self.robot.getDevice("wheel2 motor"), self.maxWheelSpeed)
        self.colorSensor = ColourSensor(self.robot.getDevice("colour_sensor"), 0.037, 32)
        self.comunicator = Comunicator(self.robot.getDevice("emitter"), self.robot.getDevice("receiver"), self.timeStep)
        self.rightCamera = Camera(self.robot.getDevice("camera3"), self.timeStep)
        self.centerCamera = Camera(self.robot.getDevice("camera2"), self.timeStep)
        self.leftCamera = Camera(self.robot.getDevice("camera1"), self.timeStep)

    # Sends and array to the controller
    def sendArray(self, array):
        self.comunicator.sendMap(array)

    # Sends the end of play to the controller
    def sendEnd(self):
        print("End sended")
        self.comunicator.sendEndOfPlay()

    # Returns True if a certain time has passed
    def delaySec(self, delay):
        if self.delayFirstTime:
            self.delayStart = self.robot.getTime()
            self.delayFirstTime = False
        else:
            if self.time - self.delayStart >= delay:
                self.delayFirstTime = True
                return True
        return False

    # Moves the wheels at the specified ratio
    def moveWheels(self, leftRatio, rightRatio):
        self.leftWheel.move(leftRatio)
        self.rightWheel.move(rightRatio)

    # Rotates to the inputted degrees
    def rotateToDegs(self, degs, orientation="closest", maxSpeed=0.5):
        accuracy = 2
        if self.rotateToDegsFirstTime:
            # print("STARTED ROTATION")
            self.rotateToDegsFirstTime = False
        self.seqRotateToDegsInitialRot = self.rotation
        self.seqRotateToDegsinitialDiff = round(self.seqRotateToDegsInitialRot - degs)
        diff = self.rotation - degs
        moveDiff = max(round(self.rotation), degs) - min(self.rotation, degs)
        if diff > 180 or diff < -180:
            moveDiff = 360 - moveDiff
        speedFract = min(mapVals(moveDiff, accuracy, 90, 0.2, 0.8), maxSpeed)
        if accuracy * -1 < diff < accuracy or 360 - accuracy < diff < 360 + accuracy:
            self.rotateToDegsFirstTime = True
            return True
        else:
            if orientation == "closest":
                if 180 > self.seqRotateToDegsinitialDiff > 0 or self.seqRotateToDegsinitialDiff < -180:
                    direction = "right"
                else:
                    direction = "left"
            elif orientation == "farthest":
                if 180 > self.seqRotateToDegsinitialDiff > 0 or self.seqRotateToDegsinitialDiff < -180:
                    direction = "left"
                else:
                    direction = "right"
            else:
                direction = orientation

            if direction == "right":
                self.moveWheels(speedFract * -1, speedFract)
            elif direction == "left":
                self.moveWheels(speedFract, speedFract * -1)
            # print("speed fract: " +  str(speedFract))
            # print("target angle: " +  str(degs))
            # print("moveDiff: " + str(moveDiff))
            # print("diff: " + str(diff))
            # print("orientation: " + str(orientation))
            # print("direction: " + str(direction))
            # print("initialDiff: " + str(self.seqRotateToDegsinitialDiff))

        # print("ROT IS FALSE")
        return False

    # Rotates to the inputted degrees smoothly
    def rotateSmoothlyToDegs(self, degs, orientation="closest", maxSpeed=0.5):
        accuracy = 2
        seqRotateToDegsinitialDiff = round(self.rotation - degs)
        diff = self.rotation - degs
        moveDiff = max(round(self.rotation), degs) - min(self.rotation, degs)
        if diff > 180 or diff < -180:
            moveDiff = 360 - moveDiff
        speedFract = min(mapVals(moveDiff, accuracy, 90, 0.2, 0.8), maxSpeed)
        if accuracy * -1 < diff < accuracy or 360 - accuracy < diff < 360 + accuracy:
            self.rotateToDegsFirstTime = True
            return True
        else:
            if orientation == "closest":
                if 180 > seqRotateToDegsinitialDiff > 0 or seqRotateToDegsinitialDiff < -180:
                    direction = "right"
                else:
                    direction = "left"
            elif orientation == "farthest":
                if 180 > seqRotateToDegsinitialDiff > 0 or seqRotateToDegsinitialDiff < -180:
                    direction = "left"
                else:
                    direction = "right"
            else:
                direction = orientation
            if direction == "right":
                self.moveWheels(speedFract * -0.5, speedFract)
            elif direction == "left":
                self.moveWheels(speedFract, speedFract * -0.5)
            # print("speed fract: " +  str(speedFract))
            # print("target angle: " +  str(degs))
            # print("moveDiff: " + str(moveDiff))
            # print("diff: " + str(diff))
            # print("orientation: " + str(orientation))
            # print("direction: " + str(direction))
            # print("initialDiff: " + str(seqRotateToDegsinitialDiff))

        # print("ROT IS FALSE")
        return False

    # Moves to the inputted coordinates
    def moveToCoords(self, targetPos):
        errorMargin = 0.01
        descelerationStart = 0.5 * 0.12
        diffX = targetPos[0] - self.globalPosition[0]
        diffY = targetPos[1] - self.globalPosition[1]
        # print("Target Pos: ", targetPos)
        # print("Used global Pos: ", self.globalPosition)
        # print("diff in pos: " + str(diffX) + " , " + str(diffY))
        dist = getDistance((diffX, diffY))
        # print("Dist: "+ str(dist))
        if errorMargin * -1 < dist < errorMargin:
            # self.robot.move(0,0)
            # print("FinisehedMove")
            return True
        else:
            ang = getDegsFromCoords((diffX, diffY))
            ang = normalizeDegs(ang)
            # print("traget ang: " + str(ang))
            ratio = min(mapVals(dist, 0, descelerationStart, 0.1, 1), 1)
            ratio = max(ratio, 0.8)
            if self.rotateToDegs(ang):
                self.moveWheels(ratio, ratio)
                # print("Moving")
        return False

    # Returns the position and tile type detected by the colour sensor
    def getColorDetection(self):
        pos = self.colorSensor.getPosition(self.globalPosition, self.rotation)
        detection = self.colorSensor.getTileType()
        return pos, detection

    # Returns True if the simulation is running
    def doLoop(self):
        return self.robot.step(self.timeStep) != -1

    # Returns if the wheels are generally moving forward or backwards
    def getWheelVelocity(self):
        if self.rightWheel.velocity + self.leftWheel.velocity == 0:
            return 0
        return (self.rightWheel.velocity + self.leftWheel.velocity) / 2

    def getWheelLinearVelocity(self):
        if self.rightWheel.getLinearVelocity() + self.leftWheel.getLinearVelocity() == 0:
            return 0
        return (self.rightWheel.getLinearVelocity() + self.leftWheel.getLinearVelocity()) / 2

    # Must run every TimeStep
    def update(self):
        # Updates the current time
        self.time = self.robot.getTime()
        # Updates the gps, gyroscope
        self.gyroscope.update(self.time)
        #self.accelerometer.update(self.time)

        # Saves the previous rotation
        self.prevRotation = self.rotation

        # Gets global rotation
        self.rotation = self.gyroscope.getDegrees()

        self.odometry.update(self.time, self.rotation, self.getWheelLinearVelocity())

        # Saves the previous global position
        self.prevGlobalPosition = self.globalPosition
        # Gets global position and applies offsets
        self.globalPosition = list(self.odometry.getPosition()) #self.accelerometer.getPos()
        print("wheelVelocity:", self.getWheelLinearVelocity())

        # print("Acc position:", str())
        self.globalPosition[0] += self.positionOffsets[0]
        self.globalPosition[1] += self.positionOffsets[1]


        # Updates emmiter and reciever
        self.comunicator.update()

        #time.sleep(0.1)
        print(self.centerCamera.getDistance())





# REMEMBER TO COPY-PASTE THIS FUNCTIONS ON TO FINAL CODE







class PlottingArray:
    def __init__(self, size, offsets, scale, tileSize):
        self.scale = scale
        self.size = size
        self.offsets = offsets
        self.scale = scale
        self.tileSize = tileSize
        self.gridPlottingArray = np.zeros(self.size, np.uint8)

        for y in range(0, len(self.gridPlottingArray), int(self.tileSize * scale)):
            for x in range(len(self.gridPlottingArray[0])):
                self.gridPlottingArray[x][y] = 50
        for x in range(0, len(self.gridPlottingArray), int(self.tileSize * scale)):
            for y in range(len(self.gridPlottingArray[0])):
                self.gridPlottingArray[x][y] = 50

    def plotPoint(self, point, value):
        procPoint = [int(point[0] * self.scale), int(point[1] * self.scale * -1)]
        finalx = procPoint[0] + int(self.offsets[0] * self.tileSize)
        finaly = procPoint[1] + int(self.offsets[1] * self.tileSize)

        if self.size[0] * -1 < finalx < self.size[0] and self.size[0] * -1 < finaly < self.size[1]:
            self.gridPlottingArray[finalx][finaly] = value

    def getPoint(self, point):
        procPoint = [int(point[0] * self.scale), int(point[1] * self.scale * -1)]
        finalx = procPoint[0] + int(self.offsets[0] * self.tileSize)
        finaly = procPoint[1] + int(self.offsets[1] * self.tileSize)

        if self.size[0] * -1 < finalx < self.size[0] and self.size[0] * -1 < finaly < self.size[1]:
            return self.gridPlottingArray[finalx][finaly]

    def reset(self):
        self.gridPlottingArray = np.zeros(self.size, np.uint8)

        for y in range(0, len(self.gridPlottingArray), round(self.tileSize * self.scale)):
            for x in range(len(self.gridPlottingArray[0])):
                self.gridPlottingArray[x][y] = 50
        for x in range(0, len(self.gridPlottingArray), round(self.tileSize * self.scale)):
            for y in range(len(self.gridPlottingArray[0])):
                self.gridPlottingArray[x][y] = 50


class AbstractionLayer():

    def __init__(self):
        # Variables
        self.tileSize = 0.12
        self.timeStep = 32
        self.gridPlotter = PlottingArray((300, 300), [1500, 1500], 150, self.tileSize)
        self.doWallMapping = False
        self.actualTileType = "undefined"
        self.isTrap = False
        self.timeInRound = 8 * 60
        self.timeWithoutMoving = 0
        self.__timeWithoutMovingStart = 0

        # Components
        self.robot = RobotLayer(self.timeStep)
        self.seqMg = SequenceManager()
        self.analyst = Analyst(self.tileSize)

        # -- Functions --
        self.seqPrint = self.seqMg.makeSimpleSeqEvent(print)
        self.seqDelaySec = self.seqMg.makeComplexSeqEvent(self.robot.delaySec)
        self.seqMoveWheels = self.seqMg.makeSimpleSeqEvent(self.robot.moveWheels)
        self.seqRotateToDegs = self.seqMg.makeComplexSeqEvent(self.robot.rotateToDegs)
        self.seqMoveToCoords = self.seqMg.makeComplexSeqEvent(self.robot.moveToCoords)
        self.seqResetSequenceFlags = self.seqMg.makeSimpleSeqEvent(self.resetSequenceFlags)
        self.seqResetSequence = self.seqMg.makeSimpleSeqEvent(self.resetSequence)

    def resetSequence(self):
        self.seqMg.resetSequence()
        self.resetSequenceFlags()
        self.seqMg.linePointer = 0

    def resetSequenceFlags(self):
        self.robot.delayFirstTime = True

    def calibrate(self):
        self.seqMg.startSequence()
        self.seqDelaySec(0.1)
        if self.seqMg.simpleSeqEvent():
            actualTile = [self.position[0] // self.tileSize, self.position[1] // self.tileSize]
            """
            self.robot.positionOffsets =  [round((actualTile[0] * self.tileSize) - self.position[0]) + self.tileSize // 2, round((actualTile[1] * self.tileSize) - self.position[1]) + self.tileSize // 2]
            self.robot.positionOffsets = [self.robot.positionOffsets[0] % self.tileSize, self.robot.positionOffsets[1] % self.tileSize]
            """

            self.robot.positionOffsets = [0.06, 0.06]

            print("positionOffsets: ", self.robot.positionOffsets)
        if self.seqMg.simpleSeqEvent(): self.analyst.registerStart()
        self.seqMoveWheels(0, 0)
        if self.seqMg.simpleSeqEvent(): self.doWallMapping = True
        return self.seqMg.seqResetSequence()

    @property
    def rotation(self):
        return self.robot.rotation

    @property
    def position(self):
        return self.robot.globalPosition

    @property
    def prevPosition(self):
        return self.robot.prevGlobalPosition

    def getBestPos(self):
        return self.analyst.getBestPosToMove()

    def doLoop(self):
        return self.robot.doLoop()

    def recalculatePath(self):
        self.analyst.calculatePath = True

    def endGame(self):
        self.sendFinalArray()
        self.robot.sendEnd()

    def sendFinalArray(self):
        self.robot.sendArray(self.analyst.getArrayRepresentation())

    def isEnded(self):
        return self.analyst.ended

    @property
    def timeLeft(self):
        return self.timeInRound - self.robot.time

    def update(self):
        self.robot.update()

        # print("Time:", self.robot.time)
        # print("time without moving: ", self.timeWithoutMoving)
        # print("time left:", self.timeLeft)
        diff = [self.position[0] - self.prevPosition[0], self.position[1] - self.prevPosition[1]]
        if self.robot.getWheelVelocity() < 0.1:
            self.timeWithoutMoving = 0
        elif -0.0001 < getDistance(diff) < 0.0001:
            if self.timeWithoutMoving == 0:
                self.__timeWithoutMovingStart = self.robot.time
                self.timeWithoutMoving = 0.000000001
            else:
                self.timeWithoutMoving = self.robot.time - self.__timeWithoutMovingStart
        else:
            self.timeWithoutMoving = 0

        if self.doWallMapping:
            # print("Doing wall mapping")

            if self.timeWithoutMoving > 1:
                self.analyst.stoppedMoving = True
            else:
                self.analyst.stoppedMoving = False

            """
            for point in pointCloud:

                if self.gridPlotter.getPoint(point) < 250:
                    self.gridPlotter.plotPoint(point, self.gridPlotter.getPoint(point) + 5)
            """

        colorPos, self.actualTileType = self.robot.getColorDetection()
        # print("Tile type: ", self.actualTileType)
        self.analyst.loadColorDetection(colorPos, self.actualTileType)
        self.isTrap = self.actualTileType == "hole"
        self.analyst.update(self.position, self.rotation)

        self.gridPlotter.reset()

        """
        bestPos = self.analyst.getStartRawNodePos()
        if bestPos is not None:
            self.gridPlotter.plotPoint(bestPos, 255)
        """

        bestPos = self.analyst.getBestPosToMove()
        if bestPos is not None:
            self.gridPlotter.plotPoint(bestPos, 200)

        self.gridPlotter.plotPoint(self.position, 150)


        bestPoses = self.analyst.getBestPoses()
        for bestPos in bestPoses:
            self.gridPlotter.plotPoint(bestPos, 255)

        self.analyst.showGrid()

        cv.imshow("raw detections",
                  cv.resize(self.gridPlotter.gridPlottingArray, (600, 600), interpolation=cv.INTER_NEAREST))
        cv.waitKey(1)








class WallFilter():
    def __init__(self):
        self.hue_min = 0
        self.hue_max = 40
        self.saturation_min = 11
        self.saturation_max = 255
        self.min_value = 0
        self.max_value = 22
        self.lower = np.array([self.hue_min, self.saturation_min, self.min_value])
        self.upper = np.array([self.hue_max, self.saturation_max, self.max_value])

    def showWallFilterMask(self, cameraImage):
        hsv_image = cv.cvtColor(cameraImage, cv.COLOR_BGRA2BGR)
        mask = cv.inRange(hsv_image, self.lower, self.upper)
        return mask

    def showWallFilterImgResult(self, cameraImage):
        imgResult = cv.bitwise_and(cameraImage, cameraImage, mask= self.showWallFilterMask(cameraImage))
        return imgResult

class DistanceSensor():
    def __init__(self):
        self.filters = WallFilter()

    

    def __distanceMeasuring(self, binaryImage):
        counter = 0
        for i in range(0,128):
            if binaryImage[i][64] == 255:
                counter += 1
        return counter

    def distanceCalculation(self, cameraImage):
        filters = self.filters.showWallFilterMask(cameraImage)
        pixels_height = self.__distanceMeasuring(filters) #ToDo: can improve
        distanceBase = 37.11126
        pixels_height_base = 52
        if pixels_height != 0:
            distance = ( pixels_height_base * distanceBase)/ pixels_height
            print(f"distanceCamera Normal--> {distance}\t distanceCamera Mapped--> {mapVals(distance, 0, 80, 0, 32)}")
        else: pass









# Class that defines a tile node in the grid
class TileNode:
    # Tuple with all allowed tile types
    __allowedTypes = ("undefined", "normal", "hole", "swamp", "checkpoint", "start", "connection1-2", "connection2-3")
    __typesToNumbers = {"undefined": "0", "normal": "0", "hole": "2", "swamp": "3", "checkpoint": "4", "start": "5",
                        "connection1-2": "6", "connection1-3": "7", "connection2-3": "8"}

    def __init__(self, tileType="undefined", curvedWall=[0, 0], fixtures=[], obstacles=[]):
        self.dimensions = [0.06, 0.06]  # Dimensions of the tile
        self.__tileType = tileType  # Can be undefined, start, normal, swamp, hole, checkpoint, connection1-2, connection2-3
        self.traversed = False

    @property
    def tileType(self):
        return self.__tileType

    @tileType.setter
    def tileType(self, value):
        if self.__tileType in ("normal", "undefined") or value in ("start",):
            self.__tileType = value

    def getString(self):
        return self.__typesToNumbers[self.tileType]


# Class that defines a wall node in the grid
class WallNode:
    def __init__(self, occupied=False, fixtures=[]):
        self.dimensions = [0.06, 0.06, 0.01]  # Dimensions of the wall
        self.__occupied = occupied  # If there is a wall. Can be True or false.
        self.traversed = False

    @property
    def occupied(self):
        return self.__occupied

    @occupied.setter
    def occupied(self, value):
        if value and not self.traversed:
            self.__occupied = True
        else:
            self.__occupied = False

    def getString(self):
        if len(self.fixtures):
            returnString = "".join(self.fixtures)
        elif self.occupied:
            returnString = "1"
        else:
            returnString = "0"
        return returnString


# Class that defines a vortex node in the grid
class VortexNode:
    def __init__(self, occupied=False):
        self.dimensions = [0.01, 0.01, 0.06]  # Dimensions of the vortex
        self.occupied = occupied  # If there is a vortex. Can be True or false.
        self.traversed = False

    @property
    def occupied(self):
        return self.__occupied

    @occupied.setter
    def occupied(self, value):
        if value and not self.traversed:
            self.__occupied = True
        else:
            self.__occupied = False

    def getString(self):
        return str(int(self.occupied))


# A virtual representation of the competition map
class Grid:
    def __init__(self, chunk, initialSize):
        self.startingSize = initialSize  # The initial size of the grid, cant be 0 and has to be divisible by the size of the chunk
        self.size = [2, 2]  # The actual size of the grid
        self.offsets = [0, 0]  # Offsets of the grid to allow negative indexes
        self.grid = [[]]  # The grid containing the data
        self.chunk = chunk  # A chunk of nodes constituting the grid
        self.chunkSize = (len(chunk), len(chunk[0]))
        self.__constructGrid()

    # Given a string indicating direction returns an array directing to that direction
    def directionToNumber(self, direction):
        if direction == "center" or direction == "centre":
            n = [0, 0]
        elif direction == "right":
            n = [0, 1]
        elif direction == "left":
            n = [0, -1]
        elif direction == "up":
            n = [-1, 0]
        elif direction == "down":
            n = [1, 0]
        elif direction == "right-up" or direction == "up-right":
            n = [1, -1]
        elif direction == "right-down" or direction == "down-right":
            n = [1, 1]
        elif direction == "left-down" or direction == "down-left":
            n = [-1, 1]
        elif direction == "left-up" or direction == "up-left":
            n = [-1, -1]
        return n

    # Constructs the grid
    def __constructGrid(self):
        self.grid = copy.deepcopy(self.chunk)
        for _ in range((self.startingSize[0] // self.chunkSize[0]) - 1):
            self.addColumnAtEnd()
        for _ in range((self.startingSize[1] // self.chunkSize[1]) - 1):
            self.addRowAtEnd()

        self.offsets[0] = self.startingSize[0] // self.chunkSize[0]
        self.offsets[1] = self.startingSize[1] // self.chunkSize[1]
        if not self.offsets[0] % self.chunkSize[0]:
            self.offsets[0] -= 1
        if not self.offsets[1] % self.chunkSize[1]:
            self.offsets[1] -= 1

        self.size = self.startingSize

    # Adds a row at the end of the grid
    def addRowAtEnd(self):
        row = copy.deepcopy(self.chunk)
        if self.size[0] > 1:
            for _ in range((self.size[0] // self.chunkSize[0]) - 1):
                row = np.hstack((row.copy(), copy.deepcopy(self.chunk)))
            self.grid = np.vstack((self.grid.copy(), copy.deepcopy(row)))
            self.size[1] += self.chunkSize[0]

    # Adds a row at the start of the grid
    def addRowAtStart(self):
        row = copy.deepcopy(self.chunk)
        if self.size[0] > 1:
            for _ in range((self.size[0] // self.chunkSize[0]) - 1):
                row = np.hstack((row.copy(), copy.deepcopy(self.chunk)))
            self.grid = np.vstack((copy.deepcopy(row), self.grid.copy()))
            self.size[1] += self.chunkSize[0]
            self.offsets[1] += self.chunkSize[0]

    # Adds a column at the end of the grid
    def addColumnAtEnd(self):
        column = self.chunk.copy()
        if self.size[1] > 1:
            for _ in range((self.size[1] // self.chunkSize[1]) - 1):
                column = np.vstack((column.copy(), copy.deepcopy(self.chunk)))
            self.grid = np.hstack((self.grid.copy(), copy.deepcopy(column)))
            self.size[0] += self.chunkSize[1]

    # Adds a column at the start of the grid
    def addColumnAtStart(self):
        column = copy.deepcopy(self.chunk)
        if self.size[1] > 1:
            for _ in range((self.size[1] // self.chunkSize[1]) - 1):
                column = np.vstack((column.copy(), copy.deepcopy(self.chunk)))
            self.grid = np.hstack((copy.deepcopy(column), self.grid.copy()))
            self.size[0] += self.chunkSize[1]
            self.offsets[0] += self.chunkSize[1]

    # returns the node in the position in the grid taking in to account offsets
    def getRawNode(self, position):
        x = position[0] + self.offsets[0]
        y = position[1] + self.offsets[1]
        return self.grid[y][x]

    # Sets a value in the position in the grid taking in to account offsets
    def setRawNode(self, position, value):
        x = position[0] + self.offsets[0]
        y = position[1] + self.offsets[1]
        self.grid[y][x] = value

    def processedToRawNode(self, position, side=[0, 0]):
        if isinstance(side, str):
            x = position[0] * self.chunkSize[0] + self.directionToNumber(side)[0]
            y = position[1] * self.chunkSize[1] + self.directionToNumber(side)[1]
        else:
            x = position[0] * self.chunkSize[0] + side[0]
            y = position[1] * self.chunkSize[1] + side[1]
        if x < self.offsets[0] * -1:
            raise IndexError("Index too small for list with min index " + str(self.offsets[0] * -1))
        if y < self.offsets[1] * -1:
            raise IndexError("Index too small for list with min index " + str(self.offsets[0] * -1))
        return (x, y)

    def rawToProcessedNode(self, rawNode):
        x = rawNode[0] // self.chunkSize[0]
        y = rawNode[1] // self.chunkSize[1]
        sideX = rawNode[0] % self.chunkSize[0]
        sideY = rawNode[1] % self.chunkSize[1]
        quadrant = [0, 0]
        if sideX > 0: quadrant[0] = 1
        if sideY > 0: quadrant[1] = 1
        return (x, y), quadrant

    # Returns a node given the position of a tile and directions to indicate walls and vertices
    def getNode(self, position, side=[0, 0]):
        return self.getRawNode(self.processedToRawNode(position, side))

    # Sets a node given the position of a tile and directions to indicate walls and vertices
    def setNode(self, position, value, side=[0, 0]):
        self.setRawNode(self.processedToRawNode(position, side), value)

    def getArrayRepresentation(self):
        grid = []
        for y in self.grid:
            row = []
            for node in y:
                row.append(node.getString())
            grid.append(row)
        return np.array(grid)

    def getNumpyPrintableArray(self):
        printableArray = np.zeros(self.size, np.uint8)
        for y in range(len(self.grid)):
            for x, node in enumerate(self.grid[y]):

                if isinstance(node, TileNode):
                    if node.tileType == "start":
                        printableArray[x][y] = 100
                    elif node.tileType == "hole":
                        # print("NEW HOLE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                        printableArray[x][y] = 255
                    elif node.tileType == "checkpoint":
                        printableArray[x][y] = 60
                    elif node.tileType == "swamp":
                        printableArray[x][y] = 80
                    elif node.traversed:
                        printableArray[x][y] = 150

                elif isinstance(node, VortexNode):
                    if node.occupied:
                        printableArray[x][y] = 255
                    else:
                        printableArray[x][y] = 50

                elif isinstance(node, WallNode):
                    if node.occupied:
                        printableArray[x][y] = 255
                    elif node.traversed:
                        printableArray[x][y] = 150
                    else:
                        printableArray[x][y] = 50

        return np.flip(printableArray, 1)


# aStarNode class for A* pathfinding (Not to be confused with the node grid)
class aStarNode():
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


# Finds the best path to follow
class PathFinder:
    def __init__(self, vortexNode, wallNode, tileNode, grid, searchLimit, startNode):
        self.grid = grid
        self.startTile = [0, 0]
        self.prevTile = [0, 0]
        self.objectiveTile = [0, 0]
        self.vortexNode = vortexNode
        self.wallNode = wallNode
        self.tileNode = tileNode
        self.searchLimit = searchLimit
        self.startNode = startNode

    def isTraversable(self, index):
        node = self.grid.getRawNode(index)
        if isinstance(node, VortexNode):
            raise ValueError("Invalid instance")
        if isinstance(node, WallNode):
            return not node.occupied
        if isinstance(node, TileNode):
            if node.tileType == "hole":
                return False
            return True
        return False

    # Returns a list of tuples as a path from the given start to the given end in the given maze
    def aStar(self, start, end):
        # Create start and end node
        startNode = aStarNode(None, (start[0], start[1]))
        startNode.g = startNode.h = startNode.f = 0
        endNode = aStarNode(None, (end[0], end[1]))
        endNode.g = endNode.h = endNode.f = 0
        # Initialize open and closed list
        openList = []
        closedList = []
        # Add the start node
        openList.append(startNode)
        # Loop until end
        while len(openList) > 0:
            # Get the current node
            currentNode = openList[0]
            currentIndex = 0
            for index, item in enumerate(openList):
                if item.f < currentNode.f:
                    currentNode = item
                    currentIndex = index
            # Pop current off open list, add to closed list
            openList.pop(currentIndex)
            closedList.append(currentNode)
            # If found the goal
            if currentNode == endNode:
                path = []
                current = currentNode
                while current is not None:
                    path.append(current.position)
                    current = current.parent
                return path[::-1]  # Return reversed path
            # Generate children
            children = []
            for newPosition in ((0, 1), (0, -1), (-1, 0), (1, 0)):  # Adjacent squares
                # Get node position
                wallPosition = (currentNode.position[0] + newPosition[0], currentNode.position[1] + newPosition[1])
                nodePosition = (
                currentNode.position[0] + (newPosition[0] * 2), currentNode.position[1] + (newPosition[1] * 2))
                # Make sure walkable terrain
                if not self.isTraversable(wallPosition):
                    continue
                if not self.isTraversable(nodePosition):
                    continue

                # Create new node
                newNode = aStarNode(currentNode, nodePosition)
                # Append
                children.append(newNode)
            # Loop through children
            for child in children:
                continueLoop = False
                # Child is on the closed list
                for closedChild in closedList:
                    if child == closedChild:
                        continueLoop = True
                        break
                # Create the f, g, and h values
                child.g = currentNode.g + 1
                child.h = ((child.position[0] - endNode.position[0]) ** 2) + (
                    (child.position[1] - endNode.position[1]) ** 2)
                child.f = child.g + child.h
                # Child is already in the open list
                for openNode in openList:
                    if child == openNode and child.g > openNode.g:
                        continueLoop = True
                        break
                if continueLoop:
                    continue
                # Add the child to the open list
                openList.append(child)

    def isBfsAddable(self, index):
        node = self.grid.getRawNode(index)
        if isinstance(node, self.tileNode):
            return not node.traversed

    # Breath First Search algorithm
    # Returns the tiles with in order and with the distance of each one
    def bfs(self, start, limit="undefined"):
        visited = []
        queue = []
        found = []
        start = [start[0], start[1], 0]
        visited.append(start)
        queue.append(start)
        while queue:
            if len(found) > 3:
                break
            coords = queue.pop(0)
            y = coords[1]
            x = coords[0]
            dist = coords[2]
            if limit != "undefined":
                if dist > limit:
                    break

            if self.isBfsAddable(coords):
                found.append(coords)
            for newPosition in (0, 1), (0, -1), (-1, 0), (1, 0):
                neighbour = [x + newPosition[0] * 2, y + newPosition[1] * 2, dist + 1]
                neighbourWall = [x + newPosition[0], y + newPosition[1]]
                inList = False
                for node in visited:
                    if node[0] == neighbour[0] and node[1] == neighbour[1]:
                        inList = True
                        break
                if inList:
                    continue

                # Make sure walkable terrain
                try:
                    if self.isTraversable(neighbour) and self.isTraversable(neighbourWall):
                        visited.append(neighbour)
                        queue.append(neighbour)
                except IndexError:
                    pass
        return found

    def setStartTile(self, startRawTilePos):
        if not isinstance(self.grid.getRawNode(startRawTilePos), self.tileNode):
            raise ValueError("Inputed position does not correspond to a tile node")
        if self.startTile != startRawTilePos:
            self.prevVortex = self.startTile

        self.startTile = startRawTilePos
        if not self.isTraversable(startRawTilePos):
            print("INITIAL TILE NOT TRAVERSABLE")

    def setGrid(self, grid):
        self.grid = grid

    def getBestPath(self, orientation):
        bfsLimits = ("undefined",)
        possibleNodes = []
        if self.isTraversable(self.startTile):
            bfsStart = self.startTile
        else:
            bfsStart = self.prevTile
        for limit in bfsLimits:
            possibleNodes = self.bfs(bfsStart, limit)
            if len(possibleNodes) > 0:
                break

        if len(possibleNodes) > 0:
            bestNode = possibleNodes[0]
            if bestNode[:2] == list(self.startTile):
                bestNode = possibleNodes[1]
            for posNode in possibleNodes:
                diff = substractLists(self.startTile, posNode[:2])
                # print("Diff:", diff)
                # print("Multiplied orientation: ", multiplyLists(orientation, [-2, -2]))
                if posNode[2] > 1:
                    break

                elif diff == multiplyLists(orientation, [-2, -2]):
                    bestNode = posNode
                    break
        else:
            bestNode = self.startNode

        bestPath = self.aStar(bfsStart, bestNode)
        print("BFS NODES: ", possibleNodes)
        print("Best Node:", bestNode)
        print("AStar PATH: ", bestPath)
        print("Start Tile: ", self.startTile)
        return bestPath


class Analyst:
    def __init__(self, tileSize):
        # Important variables
        self.tileSize = tileSize
        self.posMultiplier = 100
        # Grid
        gridChunk = np.array([[VortexNode(), WallNode()],
                              [WallNode(), TileNode()]])
        self.grid = Grid(gridChunk, (100, 100))
        # Path finder
        self.pathFinder = PathFinder(VortexNode, WallNode, TileNode, self.grid, 10, [0, 0])
        self.pathFinder.setStartTile((0, 0))
        # self.pathFinder.getBestPath()
        # Variables
        self.direction = None
        self.__bestPath = []
        self.calculatePath = True
        self.stoppedMoving = False
        self.pathIndex = 0
        self.positionReachedThresh = 0.04
        self.prevRawNode = [0, 0]
        self.ended = False

    def getRawAdjacents(self, node, side):
        rawNode = self.grid.processedToRawNode(node, side)
        adjacents = []
        for i in ((0, 1), (1, 0), (0, -1), (-1, 0)):
            adjacents.append(sumLists(rawNode, i))
        return adjacents

    def loadColorDetection(self, colorSensorPosition, tileType):
        convPos = self.getTile(colorSensorPosition)
        self.grid.getNode(convPos).tileType = tileType
        if tileType == "hole":
            self.calculatePath = True

    def getQuadrant(self, posInTile):
        if posInTile[0] > self.tileSize / 2:
            x = 1
        else:
            x = -1
        if posInTile[1] > self.tileSize / 2:
            y = 1
        else:
            y = -1
        return [x, y]

    def getTile(self, position):
        return (int(position[0] // self.tileSize), int(position[1] // self.tileSize))

    def getPosInTile(self, position):
        return ((position[0] % self.tileSize), (position[1] % self.tileSize))

    def getVortexPosInTile(self, quadrant):
        return [(self.tileSize / 2) + (quadrant[0] * (self.tileSize / 2)),
                (self.tileSize / 2) + (quadrant[1] * (self.tileSize / 2))]

    def getTilePos(self, tile):
        return (tile[0] * self.tileSize, tile[1] * self.tileSize)

    def multiplyPos(self, position):
        return (position[0] * self.posMultiplier, position[1] * self.posMultiplier)

    def getStartRawNodePos(self):
        node, quadrant = self.grid.rawToProcessedNode(self.pathFinder.startTile)
        nodePos = self.getTilePos(node)

        vortexPos = self.getVortexPosInTile(quadrant)
        return [nodePos[0] + vortexPos[0], nodePos[1] + vortexPos[1]]

    def getQuadrantFromDegs(self, degs):
        if 315 <= degs < 360 or 0 <= degs < 45:
            quadrant = (0, 1)
        elif 45 <= degs < 135:
            quadrant = (1, 0)
        elif 135 <= degs < 225:
            quadrant = (0, -1)
        elif 255 <= 315:
            quadrant = (-1, 0)
        return quadrant

    def registerStart(self):
        self.pathFinder.startNode = self.startRawNode
        self.grid.getRawNode(self.startRawNode).tileType = "start"

    def getArrayRepresentation(self):
        return self.grid.getArrayRepresentation()

    def update(self, position, rotation):
        self.direction = self.getQuadrantFromDegs(rotation)

        posInTile = self.getPosInTile(position)
        quadrant = [0, 0]
        self.tile = self.getTile(position)
        startRawNode = self.grid.processedToRawNode(self.tile)
        self.startRawNode = startRawNode
        # print("startRawNode: ", startRawNode)
        self.pathFinder.setStartTile(startRawNode)
        self.pathFinder.setGrid(self.grid)

        vortexPosInTile = self.getVortexPosInTile(quadrant)
        diff = [vortexPosInTile[0] - posInTile[0], vortexPosInTile[1] - posInTile[1]]
        distToVortex = getDistance(diff)
        if distToVortex < self.positionReachedThresh:
            self.grid.getRawNode(self.startRawNode).traversed = True

        """
        if self.stoppedMoving:
            self.blockFront
            self.calculatePath = True
        """

        if len(self.__bestPath):
            # print("Dist to Vortex: ", distToVortex)
            if distToVortex < self.positionReachedThresh and startRawNode == self.__bestPath[self.pathIndex]:
                self.pathIndex += 1

        # print("PathLenght: ", len(self.__bestPath))
        if self.pathIndex >= len(self.__bestPath):
            self.calculatePath = True

        else:
            bestNode = self.getBestRawNodeToMove()
            if bestNode is not None:
                if not self.pathFinder.isTraversable(bestNode):
                    self.calculatePath = True

        if self.calculatePath:
            # print("Calculating path")
            self.__bestPath = self.pathFinder.getBestPath(self.direction)
            self.pathIndex = 0
            if len(self.__bestPath) < 2:
                self.ended = True
            self.calculatePath = False

    def getBestRawNodeToMove(self):
        # print("Best path: ", self.__bestPath)
        # print("Index: ", self.pathIndex)
        if len(self.__bestPath):
            return self.__bestPath[self.pathIndex]
        else:
            return None

    def getBestPosToMove(self):
        bestRawNode = self.getBestRawNodeToMove()
        # print("BEST PATH: ", bestRawNode)
        if bestRawNode is None:
            return None
        node, quadrant = self.grid.rawToProcessedNode(bestRawNode)

        nodePos = self.getTilePos(node)

        vortexPos = self.getVortexPosInTile(quadrant)
        return [nodePos[0] + vortexPos[0], nodePos[1] + vortexPos[1]]
        # return nodePos

    def getBestPoses(self):
        bestPoses = []
        for bestRawNode in self.__bestPath:
            node, quadrant = self.grid.rawToProcessedNode(bestRawNode)

            nodePos = self.getTilePos(node)

            vortexPos = self.getVortexPosInTile(quadrant)
            # print("Vortex pos: ", vortexPos)
            # return
            bestPoses.append([nodePos[0] + vortexPos[0], nodePos[1] + vortexPos[1]])
        return bestPoses

    def getGrid(self):
        return self.grid.grid

    def showGrid(self):
        cv.imshow("Analyst grid",
                  cv.resize(self.grid.getNumpyPrintableArray(), (400, 400), interpolation=cv.INTER_NEAREST))





# REMEMBER TO COPY-PASTE THIS FUNCTIONS ON TO FINAL CODE




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
            stMg.changeState("moveForward")

    if stMg.checkState("stop"):
        r.seqMg.startSequence()
        r.seqMoveWheels(0, 0)

    if stMg.checkState("moveForward"):
        r.seqMg.startSequence()
        r.seqMoveWheels(0.8, 0.8)
        r.seqDelaySec(2.2)
        r.seqMoveWheels(0, 0)
        r.seqRotateToDegs(270)
        r.seqMoveWheels(0.8, 0.8)
        r.seqDelaySec(3.5)
        r.seqMoveWheels(0, 0)
        r.seqRotateToDegs(180)
        r.seqMoveWheels(0.8, 0.8)
        r.seqDelaySec(2.2)
        r.seqMoveWheels(0, 0)
        r.seqRotateToDegs(90)
        r.seqMoveWheels(0.8, 0.8)
        r.seqDelaySec(3.5)
        r.seqMoveWheels(0, 0)
        r.seqRotateToDegs(0)
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
