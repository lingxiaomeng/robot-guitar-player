#!/usr/bin/env python

# from __future__ import print_function
from math import sqrt

from std_srvs.srv import Trigger, TriggerResponse
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image

from yolov3.detect import YoloDetector
import json


def get_H():
    R = np.array(
        [[sqrt(0.5), -sqrt(0.375), sqrt(0.125)],
         [sqrt(0.5), sqrt(0.375), -sqrt(0.125)],
         [0, 0.5, sqrt(0.75)]])
    T = (np.array([[-0.287954], [0.178989], [0.0824499]]))
    d = 0.4205 - 0.05
    N = (np.array([[0], [0.5], [sqrt(0.75)]]))
    H = R + (T / d).dot(N.transpose())
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
    Homography_left = K_top_left.dot(H).dot(np.linalg.inv(K_left))
    Homography_right = K_top_right.dot(H).dot(np.linalg.inv(K_right))
    return Homography_left, Homography_right


def image2np(image, dtype):
    return np.frombuffer(image.data, dtype=dtype).reshape(image.height, image.width, -1)


def get_image():
    print("try to get image")
    right_topic = "/right_camera/color/image_raw"
    left_topic = "/left_camera/color/image_raw"
    right_depth_topic = "/right_camera/aligned_depth_to_color/image_raw"
    left_depth_topic = "/left_camera/aligned_depth_to_color/image_raw"
    while rospy:
        left_image = rospy.wait_for_message(left_topic, Image)
        right_image = rospy.wait_for_message(right_topic, Image)
        left_depth_image = rospy.wait_for_message(left_depth_topic, Image)
        right_depth_image = rospy.wait_for_message(right_depth_topic, Image)
        if left_image and right_image:
            # print(left_image)
            left_image = image2np(left_image, np.uint8)
            left_image = cv2.cvtColor(left_image, cv2.COLOR_RGB2BGR)
            right_image = image2np(right_image, np.uint8)
            right_image = cv2.cvtColor(right_image, cv2.COLOR_RGB2BGR)
            left_depth_image = image2np(left_depth_image, np.uint16)
            right_depth_image = image2np(right_depth_image, np.uint16)
            return left_image, right_image, left_depth_image, right_depth_image
        else:
            print("no image")


def perspective_image(image, H):
    out = cv2.warpPerspective(image, H, (762, 500))
    return out


def perspective(x, y, H):
    H = np.linalg.inv(H)
    c = np.array([[x], [y], [1]])
    # print(H.dot(c))
    p = H.dot(c)
    px = p[0][0] / p[2][0]
    py = p[1][0] / p[2][0]
    return px, py


def yolo_detect(image, depth_image, H):
    poss, label, im0 = detector.detect(image)
    if len(poss) > 0:
        x = (poss[0][0] + poss[0][2]) / 2
        y = (poss[0][1] + poss[0][3]) / 2
        print(f"detected x: {x} y: {y} label:{label[0]}")
        x, y = perspective(x, y, H)
        print(f"original x: {x} y: {y}")
        depth = depth_image[int(y)][int(x)]
        print(f"distance: {depth}")
        depth = get_depth(depth_image, x, y)
        print(f"distance: {depth}")
        result = {
            "x": x,
            "y": y,
            "d": int(depth[0]),
            "label": label[0]
        }
        return result
    else:
        return {}


def handle_pose_req(req):
    print("receive req")
    left_image, right_image, left_depth, right_depth = get_image()
    H_left, H_right = get_H()
    left_image_perspective = perspective_image(left_image, H_left)
    right_image_perspective = perspective_image(right_image, H_right)
    res_left = yolo_detect(left_image_perspective, left_depth, H_left)
    res_right = yolo_detect(right_image_perspective, right_depth, H_left)
    res = {"left": res_left, "right": res_right}
    res = json.dumps(res)
    print(res)
    return TriggerResponse(True, res)


def get_depth(depth_image, x, y):
    x = int(x)
    y = int(y)
    h, w, c = depth_image.shape
    depth = depth_image[y][x]
    if depth == 0:
        d1, d2, d3, d4 = 0, 0, 0, 0
        w1, w2, w3, w4 = 0, 0, 0, 0
        for i in range(0, x):
            d = depth_image[y][x - i]
            if d != 0:
                d1 = d
                w1 = i
                break
        for i in range(0, w - x - 1):
            d = depth_image[y][x + i]
            if d != 0:
                d2 = d
                w2 = i
                break
        for i in range(0, y):
            d = depth_image[y - i][x]
            if d != 0:
                d3 = d
                w3 = i
                break
        for i in range(0, h - y - 1):
            d = depth_image[y + i][x]
            if d != 0:
                d4 = d
                w4 = i
                break
        print(f"d:{d1} {d2} {d3} {d4}")
        print(f"w:{w1} {w2} {w3} {w4}")

        if w1 == w2 == w3 == w4 == 0:
            return 0
        else:
            w = w1 + w2 + w3 + w4
            d = d1 * (w - w1) + d2 * (w - w2) + d3 * (w - w3) + d4 * (w - w4)
            d = d / (3 * w)
            return d
    else:
        return depth


if __name__ == "__main__":
    rospy.init_node('yolov3_service')

    detector = YoloDetector()
    # handle_pose_req(None)
    #
    s = rospy.Service('get_pose', Trigger, handle_pose_req)
    print("Ready to get pose")
    rospy.spin()
