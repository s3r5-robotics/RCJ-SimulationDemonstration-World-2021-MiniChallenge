import cv2 as cv
import numpy as np

grid = np.zeros((100,100, 3))
startTile = grid[50,50]
grid[50,50] = 0,255,0
grid = cv.resize(grid, (500,500))
cv.imshow("grid", grid)
cv.waitKey(0)
cv.destroyAllWindows()