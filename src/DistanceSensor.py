import cv2 as cv
import numpy as np
import sys


sys.path.append(
    r"C:\Users\Maxi\Documents\program_robots\arg-slovenia\src")
from UtilityFunctions import *  # li

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
