from controller import Robot, Camera
import cv2  as cv
import numpy as np

robot = Robot()
timestep = int(robot.getBasicTimeStep())

camera1 = robot.getDevice("camera1")
camera2 = robot.getDevice("camera2")
camera3 = robot.getDevice("camera3")
distanceSensor = robot.getDevice("distance2")
camera1.enable(timestep)
camera2.enable(timestep)
camera3.enable(timestep)
distanceSensor.enable(timestep)


def empty(a):
    pass


cv.namedWindow("trackBars")
cv.resizeWindow("trackBars", 640,240)
cv.createTrackbar("hue mini", "trackBars", 0, 255, empty),
cv.createTrackbar("hue max", "trackBars", 40, 255, empty),
cv.createTrackbar("saturation mini", "trackBars", 11, 255, empty),
cv.createTrackbar("saturation max", "trackBars", 255, 255, empty),
cv.createTrackbar("min value", "trackBars", 0 , 255, empty),
cv.createTrackbar("max value", "trackBars", 22, 255, empty),


def imageSetUp(camera):
    imageResult = camera.getImage()
    imageResult = np.frombuffer(imageResult, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    imageResult = np.array(imageResult ,dtype=np.uint8)
    return imageResult

def distanceMeasuring(binaryImage):
    counter = 0
    for i in range(0,128):
        if binaryImage[i][64] == 255:
            counter += 1
    #print(f"Pixel height --> {counter}")
    return counter


def distanceCalculation(pixels_height):
    distanceBase = 37.11126
    pixels_height_base = 52
    pixels_height = pixels_height
    distance = ( pixels_height_base * distanceBase)/ pixels_height
    print(f"distanceCamera --> {distance}")
    
while robot.step(timestep) != -1:
    image1 = imageSetUp(camera1)
    distance = distanceSensor.getValue()


    hsv_image1 = cv.cvtColor(image1, cv.COLOR_BGRA2BGR)

    hue_min=cv.getTrackbarPos("hue mini", "trackBars")
    hue_max=cv.getTrackbarPos("hue max", "trackBars")

    saturation_min=cv.getTrackbarPos("saturation mini", "trackBars")
    saturation_max=cv.getTrackbarPos("saturation max", "trackBars")
    
    min_value=cv.getTrackbarPos("min value", "trackBars")
    max_value=cv.getTrackbarPos("max value", "trackBars")

    lower = np.array([hue_min, saturation_min, min_value])
    upper = np.array([hue_max, saturation_max, max_value])
    
    mask1 = cv.inRange(hsv_image1, lower, upper)

    imgResult1 = cv.bitwise_and(image1, image1, mask=mask1)

    pixel_height = distanceMeasuring(mask1)
    if pixel_height != 0:
        distanceCalculation(pixel_height)
    else: pass

    cv.imshow("camera_mask", mask1)
    cv.imshow("camera_result", imgResult1)
    

    cv.waitKey(1)
    

