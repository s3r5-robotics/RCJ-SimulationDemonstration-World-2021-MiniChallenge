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
cv.createTrackbar("hue max", "trackBars", 255, 255, empty),
cv.createTrackbar("saturation mini", "trackBars", 0, 255, empty),
cv.createTrackbar("saturation max", "trackBars", 255, 255, empty),
cv.createTrackbar("min value", "trackBars", 0 , 255, empty),
cv.createTrackbar("max value", "trackBars", 255, 255, empty),



while robot.step(timestep) != -1:
    image1 = camera1.getImage()
    image1 = np.frombuffer(image1, np.uint8).reshape((camera1.getHeight(), camera1.getWidth(), 4))
    image1 = np.array(image1 ,dtype=np.uint8)
    
    image2 = camera2.getImage()
    image2 = np.frombuffer(image2, np.uint8).reshape((camera2.getHeight(), camera2.getWidth(), 4))
    image2 = np.array(image2 ,dtype=np.uint8)
    
    image3 = camera3.getImage()
    image3 = np.frombuffer(image3, np.uint8).reshape((camera3.getHeight(), camera3.getWidth(), 4))
    image3 = np.array(image3, dtype=np.uint8)

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
    
    cv.imshow("Camera panel masks", mask_images_result)
    cv.imshow("Camera panel final results", final_results)
    cv.waitKey(1)