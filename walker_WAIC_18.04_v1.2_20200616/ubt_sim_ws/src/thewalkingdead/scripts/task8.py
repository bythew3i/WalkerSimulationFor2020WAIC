#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist

import navigation
from navigation_prepare import start_dynamic, stop_dynamic

import task9 as NavModule
from task7 import main as push_cart_main

# def final_correction():
    # Dynamic correction
    # MoveCartModule.nav.turn_to(-1.5708)
    # stop_dynamic()
    # dy = MoveCartModule.walker.relocate3()
    # diff = 1.8236 - dy
    # start_dynamic()
    # MoveCartModule.nav.move_sideway_for(diff)
    # stop_dynamic()


# Perform a second align attempt, this time do the measurement when
# static.
def static_correction():
    nav = navigation.Navigation()

    stop_dynamic()
    rospy.sleep(rospy.Duration(2.0))
    # TODO: improve the acquisition of yaw
    with nav._lock:
        cur_yaw = nav.yaw
    # diff = (3.1415 - cur_yaw)
    angle, sign = nav.compute_min_turn_angle_dir(cur_yaw, -1.5708)
    start_dynamic()
    nav.turn_for(angle*sign)
    stop_dynamic()


# def push_cart():
#     rospy.loginfo("Waiting to be stable...")
#     rospy.sleep(rospy.Duration(2.0))
#     ex, ey = MoveCartModule.walker.goto_cart()

#     rospy.sleep(rospy.Duration(1.0))
#     rospy.loginfo("Performing final correction...")
#     final_correction()

#     rospy.sleep(rospy.Duration(2.0))
#     pc = MoveCartModule.PushCart()
#     pc.mu = 5e-9
#     pc.solve(offx=-ex * 0.03, offy=0.0) #max compensation in y dir is 5e-3, otherwise IK fail

    # PushCart_Zhou_main()

def task():
    rospy.loginfo("Performing Navigation...")
    NavModule.task()

    stop_dynamic()
    rospy.loginfo("Performing second align...")
    static_correction()

    rospy.sleep(rospy.Duration(2.0))
    rospy.loginfo("Performing Push Cart...")
    # push_cart()
    push_cart_main()

if __name__ == '__main__':
    try:
        rospy.init_node('task8', anonymous=True, log_level=rospy.DEBUG)

        # MoveCartModule.nav = navigation.Navigation()
        # MoveCartModule.walker = MoveCartModule.Walker()

        task()

    except rospy.ROSInterruptException:
        pass