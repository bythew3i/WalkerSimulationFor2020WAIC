#!/usr/bin/env python

import rospy
import walker_srvs.srv
from geometry_msgs.msg import Twist


def walk_demo():
    rospy.init_node('walk_demo', anonymous=True)
    
    # start leg_motion
    scheduler = rospy.ServiceProxy(
        "/Leg/TaskScheduler", 
        walker_srvs.srv.leg_motion_MetaFuncCtrl)

    try:
        rospy.wait_for_service('/Leg/TaskScheduler', timeout=10)
        
        req = walker_srvs.srv.leg_motion_MetaFuncCtrlRequest()
        req.func_name = "dynamic"
        req.param_json = ""
        req.cmd = "start"

        # response from service
        res = scheduler(req)
        print(res)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return
    

    # control leg_motion by publishing msg
    pub = rospy.Publisher('/nav/cmd_vel_nav', Twist, queue_size=10)
    
    rate = rospy.Rate(10) # 10HZ
    while not rospy.is_shutdown():

        # Create Twist MSG
        twist_msg = Twist()
        twist_msg.linear.x = 0.1
        rospy.logdebug(twist_msg)

        pub.publish(twist_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        walk_demo()
    except rospy.ROSInterruptException:
        pass