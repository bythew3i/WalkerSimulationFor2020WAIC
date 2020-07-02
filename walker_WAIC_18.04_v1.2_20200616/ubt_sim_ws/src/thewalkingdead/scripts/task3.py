#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import walker_srvs.srv
import tf
from ubt_core_msgs.msg import JointCommand
from sensor_msgs.msg import JointState, Image
from webots_api.srv import SceneSelection
from thewalkingdead.srv import Solver
from math import pi
import cv_bridge
import cv2
from std_msgs.msg import Int64, String
from geometry_msgs.msg import Twist


#######################################################
# Grasp Cup code (derived from task2)
#######################################################
def raize():
    # Send to solver

    limb_cur = rospy.wait_for_message('/walker/leftLimb/joint_states', JointState).position
    target_pos = [-0.015880440026308996, 0.3586736446689998, -0.494047684295732]
    target_eul = [1.570798949528835, -0.0016463071519337736, 1.5707896460175066]

    target_pos = [p+x for p, x in zip(target_pos, [0.1, 0.15, 0.35])]
    target_eul = [e+x for e, x in zip(target_eul, [-1.571+0.39, 0.0, 0.3927])]
    target_ori = tf.transformations.quaternion_from_euler(*target_eul)
    
    rospy.wait_for_service('inverse_kinematic_solver')
    try:
        kine_solver = rospy.ServiceProxy('inverse_kinematic_solver', Solver)
        resp = kine_solver(LeftRight='left', limbTwist=limb_cur, 
                                targetPos=target_pos, targetOri=target_ori)
    except rospy.ServiceException as e:
        print("Service call failed %s" % e)
        return
    
    limb_tar = resp.limbTwist
    # print("Solved: ", limb_tar)
    limb_pub = rospy.Publisher('/walker/leftLimb/controller', JointCommand, queue_size=10)

    time = 0
    duration = 1
    step = [(tar - cur) / duration * 0.001 for tar, cur in zip(limb_tar, limb_cur)]

    rate = rospy.Rate(1e3)
    while time < duration * 1000:
        time += 1
        limb_cur = [cur + st for cur, st in zip(limb_cur, step)]

        limb_msg = JointCommand()
        limb_msg.mode = 5
        limb_msg.command = limb_cur

        limb_pub.publish(limb_msg)

        rate.sleep()


def approach():
    # Send to solver

    limb_cur = rospy.wait_for_message('/walker/leftLimb/joint_states', JointState).position
    target_pos = [-0.015880440026308996, 0.3586736446689998, -0.494047684295732]
    target_eul = [1.570798949528835, -0.0016463071519337736, 1.5707896460175066]

    target_pos = [p+x for p, x in zip(target_pos, [0.2, 0.15, 0.35])]
    target_eul = [e+x for e, x in zip(target_eul, [-1.571, 0.0, 0.3927])]
    target_ori = tf.transformations.quaternion_from_euler(*target_eul)
    
    rospy.wait_for_service('inverse_kinematic_solver')
    try:
        kine_solver = rospy.ServiceProxy('inverse_kinematic_solver', Solver)
        resp = kine_solver(LeftRight='left', limbTwist=limb_cur, 
                                targetPos=target_pos, targetOri=target_ori)
    except rospy.ServiceException as e:
        print("Service call failed %s" % e)
        return
    
    limb_tar = resp.limbTwist
    # print("Solved: ", limb_tar)
    limb_pub = rospy.Publisher('/walker/leftLimb/controller', JointCommand, queue_size=10)

    time = 0
    duration = 0.2
    step = [(tar - cur) / duration * 0.001 for tar, cur in zip(limb_tar, limb_cur)]

    rate = rospy.Rate(1e3)
    while time < duration * 1000:
        time += 1
        limb_cur = [cur + st for cur, st in zip(limb_cur, step)]

        limb_msg = JointCommand()
        limb_msg.mode = 5
        limb_msg.command = limb_cur

        limb_pub.publish(limb_msg)

        rate.sleep()


def reach(offx=0., offy=0.):
    # Send to solver

    limb_cur = rospy.wait_for_message('/walker/leftLimb/joint_states', JointState).position
    target_pos = [-0.015880440026308996, 0.3586736446689998, -0.494047684295732]
    target_eul = [1.570798949528835, -0.0016463071519337736, 1.5707896460175066]

    target_pos = [p+x for p, x in zip(target_pos, [0.35+offx, 0.085+offy, 0.3])]
    target_eul = [e+x for e, x in zip(target_eul, [-1.571, 0.0, 0.3927-0.15])]
    target_ori = tf.transformations.quaternion_from_euler(*target_eul)
    
    rospy.wait_for_service('inverse_kinematic_solver')
    try:
        kine_solver = rospy.ServiceProxy('inverse_kinematic_solver', Solver)
        resp = kine_solver(LeftRight='left', limbTwist=limb_cur, 
                                targetPos=target_pos, targetOri=target_ori)
    except rospy.ServiceException as e:
        print("Service call failed %s" % e)
        return
    
    limb_tar = resp.limbTwist
    # print("Solved: ", limb_tar)
    limb_pub = rospy.Publisher('/walker/leftLimb/controller', JointCommand, queue_size=10)

    time = 0
    duration = 0.8
    step = [(tar - cur) / duration * 0.001 for tar, cur in zip(limb_tar, limb_cur)]

    rate = rospy.Rate(1e3)
    while time < duration * 1000:
        time += 1
        limb_cur = [cur + st for cur, st in zip(limb_cur, step)]

        limb_msg = JointCommand()
        limb_msg.mode = 5
        limb_msg.command = limb_cur

        limb_pub.publish(limb_msg)

        rate.sleep()



def grasp():
    hand_pub = rospy.Publisher('/walker/leftHand/controller', JointCommand, queue_size=10)

    duration = 0.4
    time = 0
    hand_cur = rospy.wait_for_message("/walker/leftHand/joint_states", JointState).position

    hand_tar = [0.34, 0.34, 0.8, 0.34, 0.8, 0.34, 0.8, 0.34, 0.8, 0.34]

    hand_per_step = [(tar - cur) / duration * 0.001 for cur, tar in zip(hand_cur, hand_tar)]

    rate = rospy.Rate(1000)
    while time < duration*1000:
        time += 1
        hand_cur = [cur+step for cur, step in zip(hand_cur, hand_per_step)]

        handmsg = JointCommand()
        handmsg.mode = 5
        handmsg.command = hand_cur

        hand_pub.publish(handmsg)

        rate.sleep()


def pick():
    # Send to solver

    limb_cur = rospy.wait_for_message('/walker/leftLimb/joint_states', JointState).position
    target_pos = [-0.015880440026308996, 0.3586736446689998, -0.394047684295732]
    target_eul = [1.370798949528835, -0.0016463071519337736, 1.2707896460175066]

    target_pos = [p+x for p, x in zip(target_pos, [0.35, 0.085, 0.4])]
    target_eul = [e+x for e, x in zip(target_eul, [-1.571, 0.0, 0.3927-0.15])]
    target_ori = tf.transformations.quaternion_from_euler(*target_eul)
    
    rospy.wait_for_service('inverse_kinematic_solver')
    try:
        kine_solver = rospy.ServiceProxy('inverse_kinematic_solver', Solver)
        resp = kine_solver(LeftRight='left', limbTwist=limb_cur, 
                                targetPos=target_pos, targetOri=target_ori)
    except rospy.ServiceException as e:
        print("Service call failed %s" % e)
        return
    
    limb_tar = resp.limbTwist
    # print("Solved: ", limb_tar)
    limb_pub = rospy.Publisher('/walker/leftLimb/controller', JointCommand, queue_size=10)

    time = 0
    duration = 0.3
    step = [(tar - cur) / duration * 0.001 for tar, cur in zip(limb_tar, limb_cur)]

    rate = rospy.Rate(1e3)
    while time < duration * 1000:
        time += 1
        limb_cur = [cur + st for cur, st in zip(limb_cur, step)]

        limb_msg = JointCommand()
        limb_msg.mode = 5
        limb_msg.command = limb_cur

        limb_pub.publish(limb_msg)

        rate.sleep()


def grasp_cup(offx=0., offy=0.):
    
    raize()
    approach()
    reach(offx, offy)
    grasp()
    pick()









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

        ### Services
        rospy.wait_for_service('/walker/sence', timeout=10)
        self.scene_service = rospy.ServiceProxy(
            "/walker/sence", 
            SceneSelection
        )
        rospy.wait_for_service('/Leg/TaskScheduler', timeout=10)
        self.leg_motion_service = rospy.ServiceProxy(
            "/Leg/TaskScheduler", 
            walker_srvs.srv.leg_motion_MetaFuncCtrl
        )

        self.scene_service(scene_name="GraspCup", nav=False, vision=True)


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


            



# Main Program
def task3():
    magic = 0.8
    rospy.init_node('task3', anonymous=True)
    walker = Walker()

    num = 0
    while num not in [1,2,3,4,5]:
        num = int(input("请输入要抓取的杯子号码（1-5）："))

    ex, ey = walker.goto_cup(num)
    grasp_cup(ex*magic, ey*magic)


if __name__ == '__main__':
    try:
        task3()
    except rospy.ROSInterruptException:
        pass