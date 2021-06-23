from controller import Robot, Camera
import cv2 as cv
import numpy as np

class WallFilter():
    def __init__(self, cameraImage):
        self.cameraImage = cameraImage
        self.hue_min = 0
        self.hue_max = 40
        self.saturation_min = 11
        self.saturation_max = 255
        self.min_value = 0
        self.max_value = 22
        self.lower = np.array([self.hue_min, self.saturation_min, self.min_value])
        self.upper = np.array([self.hue_max, self.saturation_max, self.max_value])

    def showWallFilterMask(self):
        listOfMasks = []
        for image in self.cameraImage:
            hsv_image = cv.cvtColor(image, cv.COLOR_BGRA2BGR)
            mask = cv.inRange(hsv_image, self.lower, self.upper)
            listOfMasks.append(mask)
        return listOfMasks
      
    def showWallFilterImgResult(self):
        listOfMasks = self.showWallFilterImgResult()
        listOfImgResult = []
        for image in self.cameraImage:
            imgResult = cv.bitwise_and(image, image, mask= listOfMasks[image])
            listOfImgResult.append(imgResult)
        return listOfImgResult

class DistanceSensor():
    def __init__(self, camera1, camera2, camera3):
        self.camera1 = camera1
        self.camera2 = camera2
        self.camera3 = camera3
        self.filters = WallFilter([self.camera1, self.camera2, self.camera3])
        
    def __distanceMeasuring(self, binaryImage):
        counter = 0
        for i in range(0,128):
            if binaryImage[i][64] == 255:
                counter += 1
        return counter        
    
    def distanceCalculation(self):
        filters = self.filters.showWallFilterMask()
        pixels_height = self.__distanceMeasuring(filters[0]) #ToDo: can improve
        distanceBase = 37.11126
        pixels_height_base = 52
        if pixels_height != 0:
            distance = ( pixels_height_base * distanceBase)/ pixels_height
            print(f"distanceCamera --> {distance}")
        else: pass
        



def imageSetUp(camera):
    imageResult = camera.getImage()
    imageResult = np.frombuffer(imageResult, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    imageResult = np.array(imageResult ,dtype=np.uint8)
    return imageResult


#test_______
robot = Robot()
timestep = int(robot.getBasicTimeStep())
camera1 = robot.getDevice("camera1")
camera2 = robot.getDevice("camera2")
camera3 = robot.getDevice("camera3")
camera1.enable(timestep)
camera2.enable(timestep)
camera3.enable(timestep)

while robot.step(timestep) != -1:
    image1 = imageSetUp(camera1)
    image2 = imageSetUp(camera2)
    image3 = imageSetUp(camera3)
    distanceSensorCamera = DistanceSensor(image1, image2, image3)
    distanceSensorCamera.distanceCalculation()