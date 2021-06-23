from controller import Robot, Camera
import cv2  as cv
import numpy as np

robot = Robot()
timestep = int(robot.getBasicTimeStep())

camera1 = robot.getDevice("camera1")
camera2 = robot.getDevice("camera2")
camera3 = robot.getDevice("camera3")
camera1.enable(timestep)
camera2.enable(timestep)
camera3.enable(timestep)

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
    allPoints = np.where(binaryImage == 255)
    Y_points = allPoints[0]
    print(allPoints[0][0])



while robot.step(timestep) != -1:
    image1 = imageSetUp(camera1)
    image2 = imageSetUp(camera2)
    image3 = imageSetUp(camera3)

    hsv_image1 = cv.cvtColor(image1, cv.COLOR_BGRA2BGR)
    hsv_image2 = cv.cvtColor(image2, cv.COLOR_BGRA2BGR)
    hsv_image3 = cv.cvtColor(image3, cv.COLOR_BGRA2BGR)

    hue_min=cv.getTrackbarPos("hue mini", "trackBars")
    hue_max=cv.getTrackbarPos("hue max", "trackBars")

    saturation_min=cv.getTrackbarPos("saturation mini", "trackBars")
    saturation_max=cv.getTrackbarPos("saturation max", "trackBars")
    
    min_value=cv.getTrackbarPos("min value", "trackBars")
    max_value=cv.getTrackbarPos("max value", "trackBars")

    lower = np.array([hue_min, saturation_min, min_value])
    upper = np.array([hue_max, saturation_max, max_value])
    
    mask1 = cv.inRange(hsv_image1, lower, upper)
    mask2 = cv.inRange(hsv_image2, lower, upper)
    mask3 = cv.inRange(hsv_image3, lower, upper)
    imgResult1 = cv.bitwise_and(image1, image1, mask=mask1)
    imgResult2 = cv.bitwise_and(image2, image2, mask=mask2)
    imgResult3 = cv.bitwise_and(image3, image3, mask=mask3)

    mask_images_result = cv.hconcat([mask1, mask2, mask3])
    final_results = cv.hconcat([imgResult1, imgResult2, imgResult3])
    
    cropped_image = mask2[]
    cv.imshow("Camera panel masks", mask_images_result)
    cv.imshow("Camera panel final results", final_results)
    distanceMeasuring(mask2)
    cv.waitKey(1)
    

