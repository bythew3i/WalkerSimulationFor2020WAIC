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
from navigation_prepare import start_dynamic, stop_dynamic, goto_nav

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
        rospy.init_node("task12", anonymous=True, log_level=rospy.INFO)
        goto_nav("OpenFridge")
        rospy.loginfo("Task12_Main: Waiting for leg_motion service to finish initializing...")
        start_dynamic()
        rospy.loginfo("Task12_Main: leg_motion service started, in dynamic mode...")
        task()
    except rospy.ROSInterruptException as e:
        print("ROS Interrupted: {}".format(e))

