#!/usr/bin/env python

import sys
import rospy

from std_srvs.srv import *

def get_pose(val):
    rospy.wait_for_service('/halcon')
    try:
        ser = rospy.ServiceProxy('/halcon', SetBool)
        res = ser(val)
        return res.message
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
    print("Requesting True")
    print(get_pose(True))
   