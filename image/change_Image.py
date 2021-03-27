# coding=utf-8
import glob

import cv2 as cv
import sys
import numpy as np
import math


def undistort(img):
    mtx = np.array([[892.622005, 0.000000, 637.957594],
                    [0.000000, 891.284636, 357.766994],
                    [0.000000, 0.000000, 1.000000]])

    dist = np.array([[0.083420, -0.163709, -0.001863, 0.000386, 0.000000]])

    w = 1280
    h = 720

    newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1,
                                                     (w, h))

    # Checking to make sure the new camera materix was properly generated
    # print(newcameramtx)

    # Undistorting
    dst = cv.undistort(img, mtx, dist, None, newcameramtx)

    # Cropping the image
    x, y, w, h = roi
    dst = dst[y:+y + h, x:x + w]
    return dst


if __name__ == "__main__":

    K = np.array([[719.86743513, 0., 316.97955052],
                  [0., 749.4460099, 238.44845194],
                  [0., 0., 1.]])

    K = np.array([[892.622005, 0.000000, 637.957594],
                  [0.000000, 891.284636, 357.766994],
                  [0.000000, 0.000000, 1.000000]])

    xa = np.array([1 / math.sqrt(2), 1 / math.sqrt(2), 0])
    za = np.array([math.sqrt(2) / 4, -math.sqrt(2) / 4, math.sqrt(3) / 2])
    # print np.cross(xa, za)

    R = np.array(
        [xa,
         np.cross(za, xa),
         za]).transpose()

    # R = np.linalg.inv(R + 1e-10)
    T = (np.array([[-0.3], [0.2], [0]]))

    d = 1
    dinv = 1 / d

    N = (np.array([[0], [0], [0.5]]))

    H = R + (T / d).dot(N.transpose())
    # print H
    K_top = np.array([[892.622005, 0.000000, 300],
                      [0.000000, 891.284636, 800],
                      [0.000000, 0.000000, 1.000000]])

    Homography = K_top.dot(H).dot(np.linalg.inv(K))
    Homography_inv = np.linalg.inv(Homography)

    # Homography = np.float32([[1, 0.1, 0], [0, 1, 0], [0, 0, 1]])
    images = glob.glob('/home/mlx/project/src/image/data_set/*.jpg')  # 拍摄的十几张棋盘图片所在目录

    i = 0
    for fname in images:
        img = cv.imread(fname)
        img = undistort(img)
        out = cv.warpPerspective(img, Homography, (1200, 1000))
        cv.imwrite('/home/mlx/project/src/image/data_set_img/%d.jpg' % i, out)
        i += 1
        print(i)
        # break
        # break
        # cv.waitKey(0)
