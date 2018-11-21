#!/usr/bin/env python

import sys
import rospy

from geometry_msgs.msg import Pose2D

from gluebot_app.srv import *

def get_pose():
    rospy.wait_for_service('/halcon')
    try:
        ser = rospy.ServiceProxy('/halcon', GetPose2D)
        res = ser()
        return res.pose
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
    print("Requesting True")
    print(get_pose())
   