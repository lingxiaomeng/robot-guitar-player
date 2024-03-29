# coding=utf-8
import glob
import os
import cv2 as cv
import sys
import numpy as np
from math import sqrt
#
# K_left = [606.4096069335938, 0.0, 322.3016052246094, 0.0, 0.0, 606.3513793945312, 235.92373657226562, 0.0, 0.0, 0.0,
#           1.0, 0.0]
# K_right = [614.1268310546875, 0.0, 321.2118835449219, 0.0, 614.7515869140625, 234.63755798339844, 0.0, 0.0, 1.0]


def get_H():
    R = np.array(
        [[sqrt(0.5), -sqrt(0.375), sqrt(0.125)],
         [sqrt(0.5), sqrt(0.375), -sqrt(0.125)],
         [0, 0.5, sqrt(0.75)]])
    T = (np.array([[-0.287954], [0.178989], [0.0824499]]))
    d = 0.4205 - 0.05
    N = (np.array([[0], [0.5], [sqrt(0.75)]]))
    H = R + (T / d).dot(N.transpose())
    return H


if __name__ == "__main__":

    K_left = np.array([[606.4096069335938, 0., 322.3016052246094],
                       [0., 606.3513793945312, 235.92373657226562],
                       [0., 0., 1.]])

    K_right = np.array([[614.1268310546875, 0., 321.2118835449219],
                        [0., 614.7515869140625, 234.63755798339844],
                        [0., 0., 1.]])
    K_top_left = np.array([[606.4096069335938, 0., 480],
                           [0., 606.3513793945312, 250],
                           [0., 0., 1.]])

    K_top_right = np.array([[614.1268310546875, 0., 480],
                            [0., 614.7515869140625, 250],
                            [0., 0., 1.]])
    H = get_H()
    Homography_left = K_top_left.dot(H).dot(np.linalg.inv(K_left))
    Homography_right = K_top_right.dot(H).dot(np.linalg.inv(K_right))
    guitar_images = glob.glob('/home/mlx/project/src/image/data_set/guitar_data_set/*.jpg')
    keyboard_images = glob.glob('/home/mlx/project/src/image/data_set/key_board_data_set/*.jpg')

    i = 0
    for fname in guitar_images:
        name = 'guitar_' + (os.path.basename(fname))
        print(name)
        img = cv.imread(fname)
        if 'left' in name:
            out = cv.warpPerspective(img, Homography_left, (762, 500))
        else:
            out = cv.warpPerspective(img, Homography_right, (762, 500))
        cv.imwrite('/home/mlx/project/src/image/data_set_img/' + name, out)
    for fname in keyboard_images:
        name = 'keyboard_' + (os.path.basename(fname))
        print(name)
        img = cv.imread(fname)
        if 'left' in name:
            out = cv.warpPerspective(img, Homography_left, (762, 500))
        else:
            out = cv.warpPerspective(img, Homography_right, (762, 500))
        cv.imwrite('/home/mlx/project/src/image/data_set_img/' + name, out)
        # cv.imshow('', out)
        # k = cv.waitKey(0)
        # if k == 27:
        #     break
        # elif k=='a':
        #     continue
