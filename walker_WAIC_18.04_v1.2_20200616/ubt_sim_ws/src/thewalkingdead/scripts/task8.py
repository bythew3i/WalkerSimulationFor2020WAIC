#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist

import navigation
from navigation_prepare import start_dynamic, stop_dynamic, goto_nav

import task9 as NavModule
from task7 import main as push_cart_main

# Perform a second align attempt, do the measurement when
# static.
def static_correction():
    nav = navigation.Navigation()

    stop_dynamic()
    rospy.sleep(rospy.Duration(2.0))
    # TODO: improve the acquisition of yaw
    with nav._lock:
        cur_yaw = nav.yaw
    angle, sign = nav.compute_min_turn_angle_dir(cur_yaw, -1.5708)
    start_dynamic()
    nav.turn_for(angle*sign)
    stop_dynamic()

def task():
    rospy.loginfo("Performing Navigation...")
    NavModule.task()

    rospy.loginfo("Performing second align...")
    static_correction()

    rospy.sleep(rospy.Duration(2.0))
    rospy.loginfo("Performing Push Cart...")
    push_cart_main()

if __name__ == '__main__':
    try:
        rospy.init_node('task8', anonymous=True, log_level=rospy.INFO)
        goto_nav("PushCart")
        rospy.loginfo("Task8_Main: Waiting for leg_motion service to finish initializing...")
        start_dynamic()
        rospy.loginfo("Task8_Main: leg_motion service started, in dynamic mode...")
        task()

    except rospy.ROSInterruptException:
        pass