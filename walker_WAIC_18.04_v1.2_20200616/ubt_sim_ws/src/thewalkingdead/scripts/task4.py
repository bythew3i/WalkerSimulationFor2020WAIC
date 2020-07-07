#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Int64, String
from geometry_msgs.msg import Twist

import walker_srvs.srv
from ubt_core_msgs.msg import JointCommand

import cv_bridge
import cv2

from math import pi

import navigation
from navigation_prepare import stop_dynamic
from task3 import grasp_cup
from task5 import task as move_to_room


#######################################################
# Align Robot Code
#######################################################
class Walker(object):
    def __init__(self):
        self.log_msg = ""
        self.head_depth = Image()
        self.step_num = Int64()
        self.status = String()

        self.rate = rospy.Rate(1000)
        
        ### Subscribers
        rospy.Subscriber("/walker/camera/headDepth", Image, self.head_depth_cb)
        rospy.Subscriber("/Leg/StepNum", Int64, self.step_num_cb)
        rospy.Subscriber("/Leg/leg_status", String, self.leg_status_cb)

        ### Publishers
        self.head_ctrl_pub = rospy.Publisher('/walker/head/controller', JointCommand, queue_size=10)
        self.leg_motion_pub = rospy.Publisher('/nav/cmd_vel_nav', Twist, queue_size=10)
        
        ### Messages
        self.head_ctrl_msg = JointCommand()
        self.head_ctrl_msg.command = [0 for _ in range(2)]
        self.leg_motion_msg = Twist()

        rospy.wait_for_service('/Leg/TaskScheduler', timeout=10)
        self.leg_motion_service = rospy.ServiceProxy(
            "/Leg/TaskScheduler", 
            walker_srvs.srv.leg_motion_MetaFuncCtrl
        )

    ## Subscriber Call backs
    def head_depth_cb(self, depth):
        self.head_depth = depth
    
    def step_num_cb(self, step):
        self.step_num = step

    def leg_status_cb(self, status):
        self.leg_status = status



    def _trans(self, start, target, fac):
        return start + (target-start)*fac

    def _log(self, msg):
        if self.log_msg != msg:
            print msg
            self.log_msg = msg


    # Start legmotion
    def leg_motion_start(self):
        if self.leg_motion_service != None:
            req = walker_srvs.srv.leg_motion_MetaFuncCtrlRequest()
            req.func_name = "dynamic"
            req.param_json = ""
            req.cmd = "start"
            # response
            res = self.leg_motion_service(req)
            if res.success:
                return
        raise Exception("Failed: leg_motion_start")

    # Stop legmotion
    def leg_motion_stop(self):
        if self.leg_motion_service != None:
            req = walker_srvs.srv.leg_motion_MetaFuncCtrlRequest()
            req.func_name = "dynamic"
            req.param_json = ""
            req.cmd = "stop"
            # response
            res = self.leg_motion_service(req)
            if res.success:
                return
        raise Exception("Failed: leg_motion_stop")

    def measure_depth(self):
        r = 0
        c = self.head_depth.width / 2
        bridge = cv_bridge.CvBridge()
        cv_depth = bridge.imgmsg_to_cv2(self.head_depth, desired_encoding="passthrough")
        cv_depth = cv_depth.astype(float)
        while r < self.head_depth.height / 2:
            if cv_depth[r][c] < 5:
                return cv_depth[r+5][c]
            r += 1
        return cv_depth[r][c]

    def relocate(self):
        self._log("... Relocating ...")
        dur = 1.0
        wait = 0.2
        head_P_init = self.head_ctrl_msg.command[1]
        # Turn Head Left
        timer = 0.0
        while timer < dur + wait:
            timer += 0.001
            if timer < dur:
                fac = timer / dur
                self.head_ctrl_msg.mode = 5
                self.head_ctrl_msg.command[0] = self._trans(0, pi/2, fac)
                self.head_ctrl_msg.command[1] = self._trans(head_P_init, -20.0/180*pi, fac)
                self.head_ctrl_pub.publish(self.head_ctrl_msg)
            self.rate.sleep()

        # Measure Y depth
        dy = self.measure_depth()

        # Turn Head Back
        timer = 0.0
        while timer < dur + wait:
            timer += 0.001
            if timer < dur:
                fac = timer / dur
                self.head_ctrl_msg.mode = 5
                self.head_ctrl_msg.command[0] = self._trans(pi/2, 0, fac)
                self.head_ctrl_msg.command[1] = -20.0/180*pi
                self.head_ctrl_pub.publish(self.head_ctrl_msg)
            self.rate.sleep()
        
        # Measure X depth
        dx = self.measure_depth()

        return dx, dy

    
    def goto(self, x, y):
        ustep = 0.04
        dx, dy = self.relocate()
        mx = int(round((dx - x) / ustep))
        my = int(round((dy - y) / ustep * 2))

        self._log("Move X: {}".format(mx))
        self.leg_motion_start()
        while self.step_num.data < abs(mx)-1:
            self.leg_motion_msg.linear.x = ustep if mx > 0 else -ustep
            self.leg_motion_msg.linear.y = 0
            self.leg_motion_pub.publish(self.leg_motion_msg)
            self.rate.sleep()
        # self.leg_motion_stop()

        self._log("Move Y: {}".format(my))
        # self.leg_motion_start()
        while self.step_num.data < abs(my) + abs(mx)-1:
            self.leg_motion_msg.linear.x = 0
            self.leg_motion_msg.linear.y = ustep if my > 0 else -ustep
            self.leg_motion_pub.publish(self.leg_motion_msg)
            self.rate.sleep()
        self.leg_motion_stop()
        
        ## wait to be stable
        timer = 0.0
        while timer < 2:
            timer += 0.001
            self.rate.sleep()


    def goto_cup(self, n):
        gap = 0.26
        tx = 3.2727
        ty = 2.8298 + gap * (n-1)
        self.goto(tx, ty)

        dx, dy = self.relocate()
        ex = dx - tx 
        ey = dy - ty
        print "Error X:", ex
        print "Error Y:", ey
        return ex, ey

def task():
    """
    Main Program
        - get target cup num
        - navigate to graspcup room
        - stop and wait till stable
        - perform precise relocation
        - execute grasp action
    """
    walker = Walker()
    
    num = 0
    while num not in [1,2,3,4,5]:
        num = int(input("\n请输入要抓取的杯子号码（1-5）："))

    move_to_room()
    rospy.loginfo("Finished move_to_room")
    
    stop_dynamic()
    rospy.sleep(rospy.Duration(2.0))
    rospy.loginfo("Dynamic motion stopped.")
    
    rospy.loginfo("Going to cup %d.", num)
    ex, ey = walker.goto_cup(num)
    grasp_cup(ex*0.8, ey*0.85)

if __name__ == '__main__':
    try:
        rospy.init_node("task4", anonymous=True, log_level=rospy.INFO)
        task()
    except rospy.ROSInterruptException:
        pass