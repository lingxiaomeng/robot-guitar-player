#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import time

import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import numpy as np
import os

# Instantiate CvBridge
bridge = CvBridge()


def main():
    rospy.init_node('image_listener')
    # Define your image topic
    i = 6
    data_type = 'guitar'
    dirs = '/home/mlx/project/src/image/data_set/%s_data_set/' % data_type
    if not os.path.exists(dirs):
        os.makedirs(dirs)
    while rospy:
        right_topic = "/right_camera/color/image_raw"
        left_topic = "/left_camera/color/image_raw"

        left_image = rospy.wait_for_message(left_topic, Image)
        right_image = rospy.wait_for_message(right_topic, Image)
        if left_image and right_image:
            # print(left_image)
            left_image = np.frombuffer(left_image.data, dtype=np.uint8).reshape(left_image.height, left_image.width, -1)
            left_image = cv2.cvtColor(left_image, cv2.COLOR_RGB2BGR)
            right_image = np.frombuffer(right_image.data, dtype=np.uint8).reshape(right_image.height, right_image.width,
                                                                                  -1)
            right_image = cv2.cvtColor(right_image, cv2.COLOR_RGB2BGR)
            image = np.hstack((left_image, right_image))
            cv2.imshow('right', image)
            k = cv2.waitKey(1)

            if k == ord('s'):

                cv2.imwrite(dirs + 'left_%d.jpg' % (i), left_image)
                cv2.imwrite(dirs + 'right_%d.jpg' % (i), right_image)
                print("save image %d" % i)
                i += 1
            elif k == 27:
                break
        time.sleep(1 / 30)
    # Set up your subscriber and define its callback
    # Spin until ctrl + c
    # rospy.spin()


if __name__ == '__main__':
    main()
