#!/usr/bin/env python

import navigation
from navigation_prepare import goto_nav, start_dynamic
import rospy

waypoints = [[1.00, -7.00], #About the center of the left-sofa-passage
                [1.00, -4.00], #Intermediate waypoint, for collision avoidance
                [0.00, 1.705], #In align with the pushcart task blue box
                [-1.754, 1.705] #The blue box
            ]
final_yaw = -1.5708

def task():
    global nav
    nav = navigation.Navigation()
    rospy.loginfo("Attempting Relocalize...")
    nav.relocalize()

    nav.halt(1.0)

    initial_waypoint = waypoints[0]
    initial_yaw = 1.5708

    rospy.loginfo("Going to initial waypoint.")
    nav.go_to(initial_waypoint, head_on=False)

    nav.halt(1.0)
    rospy.loginfo("Going to initial yaw.")
    nav.turn_to(initial_yaw)

    nav.halt(1.0)
    for i in range(1, len(waypoints), 1):
        rospy.loginfo("Going to %dth waypoint, position is :%s", i, str(waypoints[i]))
        nav.go_to(waypoints[i], head_on=True)
        nav.halt(1.0)
    rospy.loginfo("Turning to final yaw.")
    nav.turn_to(final_yaw)
    nav.halt()

if __name__ == '__main__':
    try:
        rospy.init_node("task9_runner", anonymous=True, log_level=rospy.INFO)
        goto_nav("PushCart")
        rospy.loginfo("Task9_Main: Waiting for leg_motion service to finish initializing...")
        start_dynamic()
        rospy.loginfo("Task9_Main: leg_motion service started, in dynamic mode...")
        task()
    except rospy.ROSInterruptException:
        pass