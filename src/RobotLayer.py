from controller import *
import sys
import numpy as np
import struct
import time

# REMEMBER TO COPY-PASTE THIS FUNCTIONS ON TO FINAL CODE
# v Put files directory here v
sys.path.append(
    r"C:\Users\Maxi\Documents\program_robots\arg-slovenia\src")
# Imports all utility functions
from UtilityFunctions import *  # li
from DistanceSensor import DistanceSensor # li


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
