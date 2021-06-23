import cv2 as cv
import numpy as np

grid = np.zeros((100,100,3), np.uint8)
cv.rectangle(grid, (0,50), (100,30), (255,255,255), -1)
cv.imshow("a",grid)
cropped_image = grid[50:50, 100:60]
cv.imshow("Cropped_image", cropped_image)
all_points = np.where(grid == 255)
print(f"Y--> {all_points[0][0]}")
#print(f"x--> {all_points[0][0]}")
cv.waitKey(0)
cv.destroyAllWindows()
