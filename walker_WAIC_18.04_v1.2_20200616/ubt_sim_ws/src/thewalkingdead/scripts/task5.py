#!/usr/bin/env python

import navigation
import rospy

waypoints = [[1.00, -7.00], #About the center of the left-sofa-passage
                [1.00, -4.00], #Intermediate waypoint, for collision avoidance
                [0.00, 0.00],
                [1.00, 2.00], #Right the center of graspcup doorway
                [3.00, 2.00], #Inside the room
                [3.00, 0.00] #In front of the table
            ]
final_yaw = 0.0 #Facing East(X direction)

def task():
    nav = navigation.Navigation()
    rospy.loginfo("Attempting Relocalize...")
    nav.relocalize()

    # nav.halt(1.0)

    initial_waypoint = waypoints[0]
    initial_yaw = 1.5708

    rospy.loginfo("Going to initial waypoint.")
    nav.go_to(initial_waypoint, head_on=False)

    # nav.halt(1.0)
    rospy.loginfo("Going to initial yaw.")
    nav.turn_to(initial_yaw)

    # nav.halt(1.0)
    for i in range(1, len(waypoints), 1):
        rospy.loginfo("Going to %dth waypoint, position is :%s", i, str(waypoints[i]))
        nav.go_to(waypoints[i], head_on=True)
        # nav.halt(1.0)
    rospy.loginfo("Turning to final yaw.")
    nav.turn_to(final_yaw)
    nav.halt()

if __name__ == '__main__':
    try:
        rospy.init_node("task5_runner", anonymous=True, log_level=rospy.INFO)
        task()
    except rospy.ROSInterruptException:
        pass