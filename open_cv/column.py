from typing import Counter
import cv2 as cv
import numpy as np

counter = 0
grid = np.zeros((200,200,3), np.uint8)
for i in range(100):
    grid[i,100] = 255
for i in grid:
    if i[100][0]== 255:
        counter+=1
        print(f"counter: {counter}")
        
cv.imshow("grid2", grid)
cv.waitKey(0)
cv.destroyAllWindows()

""" 
def mathematical(pixel_height):
    (altura_base * distancia_base)/altura_nueva """

"""
*sacar altura en base y distancia base con el sensor de distancia (ejemplo base)
*obtener altura en pixeles (altura nueva)
*distancia = (altura_base * distancia_base)/altura_nueva
"""