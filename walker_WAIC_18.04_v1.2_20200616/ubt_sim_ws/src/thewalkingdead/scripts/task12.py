#!/usr/bin/env python

import rospy
import walker_srvs.srv
from sensor_msgs.msg import Image, JointState
from ubt_core_msgs.msg import JointCommand
from std_msgs.msg import Int64, String
from geometry_msgs.msg import Twist, WrenchStamped
from webots_api.srv import SceneSelection
from math import pi
import cv_bridge
import cv2
from thewalkingdead.srv import Solver

from task13 import task as move_to_room
from navigation_prepare import stop_dynamic

from task11 import OpenFridgeSolver




def task():
    move_to_room()

    stop_dynamic()
    # rospy.sleep(rospy.Duration(2.0))
    rospy.loginfo("Dynamic motion stopped.")

    solver = OpenFridgeSolver()
    solver.solve()

if __name__ == '__main__':
    try:
        rospy.init_node("task12", anonymous=True, log_level=rospy.DEBUG)
        task()
    except rospy.ROSInterruptException as e:
        print("ROS Interrupted: {}".format(e))

