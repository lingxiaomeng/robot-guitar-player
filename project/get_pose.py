#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from std_srvs.srv import *
import json

def get_pose():
    rospy.wait_for_service('get_pose')
    try:
        add_two_ints = rospy.ServiceProxy('get_pose', Trigger)
        resp1 = add_two_ints()
        res = (json.loads(resp1.message))
        print(res["left"]['x'])
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


if __name__ == "__main__":
    get_pose()
