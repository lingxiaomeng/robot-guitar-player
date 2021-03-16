import cv2 as cv
import sys
import numpy as np
import math

img = cv.imread("/home/mlx/Pictures/guitar_Color.png")

K = np.array([[537.21281491, 0, 285.67613294],
              [0, 535.5753867, 264.40982954],
              [0, 0, 1],
              ])

R = np.array(
    [[-1 / math.sqrt(2), -1 / math.sqrt(3), 1 / math.sqrt(6)],
     [1 / math.sqrt(2), 1 / math.sqrt(3), -1 / math.sqrt(6)],
     [0, 1 / math.sqrt(3), 2 / math.sqrt(6)]])

print (R)
R = np.linalg.inv(R + 1e-10)

print R

T = R.dot(np.array([[-0.5], [0.5], [0]]))

d = 1
dinv = 1 / d

N = R.dot(np.array([[0], [0], [-0.8]]))

H = R + (T / d).dot(N.transpose())
print H
K_top = np.array([[320, 0, 320],
                  [0, 240, 240],
                  [0, 0, 1]])

Homography = K_top.dot(H).dot(np.linalg.inv(K))
# Homography = np.linalg.inv(Homography)

# Homography = np.float32([[1, 0.1, 0], [0, 1, 0], [0, 0, 1]])

out = cv.warpPerspective(img, Homography, (1000, 1000))
cv.imshow("Display window", out)
cv.waitKey(0)
