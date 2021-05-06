#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from std_srvs.srv import *
import json
import tf


def get_pose():
    rospy.wait_for_service('get_pose')
    try:
        pose = rospy.ServiceProxy('get_pose', Trigger)
        resp1 = pose()
        # print(resp1)
        res = (json.loads(resp1.message))
        br = tf.TransformBroadcaster()
        rate = rospy.Rate(10.0)
        print(resp1)
        while not rospy.is_shutdown():
            if res['left']:
                br.sendTransform((res['left']['x'], res['left']['y'], res['left']['z']),
                                 (0.0026716, 0.96528, -0.26119, -0.0010496),
                                 rospy.Time.now(),
                                 "left_" + res['left']['label'],
                                 "left_camera_color_optical_frame")
                # rospy.loginfo("add left")

            if res['right']:
                br.sendTransform((res['right']['x'], res['right']['y'], res['right']['z']),
                                 (0.0026716, 0.96528, -0.26119, -0.0010496),
                                 rospy.Time.now(),
                                 "right_" + res['right']['label'],
                                 "right_camera_color_optical_frame")
                # rospy.loginfo("add right")

            rate.sleep()


    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def listen():
    # rospy.init_node('turtle_tf_listener')

    listener = tf.TransformListener()
    try:
        listener.waitForTransform('/left_arm_link', '/left_base_link', rospy.Time(), rospy.Duration(4.0))
        (trans, rot) = listener.lookupTransform('/left_arm_link', '/left_base_link', rospy.Time(0))
        print(trans)
        print(rot)
    except Exception as e:
        print(e)


#

if __name__ == "__main__":
    rospy.init_node('fixed_tf_broadcaster')
    get_pose()
    # add_frame()
    # listen()
