#!/usr/bin/env python

import rospy
from  webots_api.srv import SceneSelection, SceneSelectionRequest
from walker_srvs.srv import leg_motion_MetaFuncCtrl, leg_motion_MetaFuncCtrlRequest
from geometry_msgs.msg import Twist, WrenchStamped
from ubt_core_msgs.msg import JointCommand
from sensor_msgs.msg import JointState
from thewalkingdead.srv import Solver
import time
from math import pi,sin,cos
# import math
import numpy as np


# def callback(msg):
#     print(msg.position)
#     # msg.

def load_scene():
    """Load the scene for this task
    """
    rospy.wait_for_service("/walker/sence")

    try:
        scheduler = rospy.ServiceProxy("/walker/sence", SceneSelection)
        request = SceneSelectionRequest()
        request.scene_name = "PushCart"
        request.nav = False
        request.vision = False
        response = scheduler(request)
        print(response)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def startWalk():
    try:
        rospy.wait_for_service("/Leg/TaskScheduler", timeout=10)
        scheduler = rospy.ServiceProxy("/Leg/TaskScheduler", leg_motion_MetaFuncCtrl)
        request = leg_motion_MetaFuncCtrlRequest()
        request.func_name = "dynamic"
        request.param_json = ''
        request.cmd = "start"
        response = scheduler(request)
        print(response)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    
        # control leg_motion by publishing msg
    pub = rospy.Publisher('/nav/cmd_vel_nav', Twist, queue_size=10)
    

def stopWalk():
    try:
        rospy.wait_for_service("/Leg/TaskScheduler", timeout=10)
        scheduler = rospy.ServiceProxy("/Leg/TaskScheduler", leg_motion_MetaFuncCtrl)
        request = leg_motion_MetaFuncCtrlRequest()
        request.func_name = "dynamic"
        request.param_json = ''
        request.cmd = "stop"
        response = scheduler(request)
        print(response)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    
        # control leg_motion by publishing msg
    pub = rospy.Publisher('/nav/cmd_vel_nav', Twist, queue_size=10)

# class subscriber():
#     def __init__(self):
#         self.sub = None
#         self.msg = None


def move(node, tar_cmd=[0]*7, duration=0.1):
    pub = rospy.Publisher('/walker/'+node+'/controller', JointCommand, queue_size=10)

    time = 0
    cur_cmd = rospy.wait_for_message("/walker/"+node+"/joint_states", JointState).position

    move_per_step = [(tar - cur) / duration * 0.001 for cur, tar in zip(cur_cmd, tar_cmd)]
    rate = rospy.Rate(1000)
    while time < duration*1000:
        time += 1
        cur_cmd = [cur+step for cur, step in zip(cur_cmd, move_per_step)]
        msg = JointCommand()
        msg.mode = 5
        msg.command = cur_cmd
        pub.publish(msg)
        rate.sleep()
    pass

# def move(node, tar_cmd=[0]*7, duration=0.1):
#     pub = rospy.Publisher('/walker/'+node+'/controller', JointCommand, queue_size=10)

#     time = 0
#     cur_cmd = rospy.wait_for_message("/walker/"+node+"/joint_states", JointState).position

#     move_per_step = [(tar - cur) / duration * 0.001 for cur, tar in zip(cur_cmd, tar_cmd)]
#     rate = rospy.Rate(1000)
#     while time < duration*1000:
#         time += 1
#         cur_cmd = [cur+step for cur, step in zip(cur_cmd, move_per_step)]
#         msg = JointCommand()
#         msg.mode = 5
#         msg.command = cur_cmd
#         pub.publish(msg)
#         rate.sleep()
#     pass


class Robot():
    def __init__(self):
        self.leftLimb_cmd = None
        self.rightLimb_cmd = None
        rospy.Subscriber("/walker/leftLimb/joint_states", JointState, self.leftLimb_subscriber)
        rospy.Subscriber("/walker/rightLimb/joint_states", JointState, self.rightLimb_subscriber)
        # rospy.Subscriber("/walker/leftLimb/joint_states", JointState, self.leftLimb_subscriber)
        # rospy.Subscriber("/walker/rightLimb/joint_states", JointState, self.rightLimb_subscriber)

        rospy.Subscriber("/sensor/ft/lwrist", WrenchStamped, self.lwrist_subscriber)

        rospy.Subscriber("/sensor/ft/rwrist", WrenchStamped, self.rwrist_subscriber)


        # rospy.wait_for_service('/Leg/TaskScheduler', timeout=10)
        # self.legmotion_service = rospy.ServiceProxy(
        #     "/Leg/TaskScheduler", 
        #     walker_srvs.srv.leg_motion_MetaFuncCtrl
        # )

        rospy.wait_for_service('inverse_kinematic_solver', timeout=10)
        self.ik_service = rospy.ServiceProxy(
            "inverse_kinematic_solver",
            Solver
        )

        rospy.wait_for_service('forward_kinematic_solver', timeout=10)
        self.fk_service = rospy.ServiceProxy(
            "forward_kinematic_solver",
            Solver
        )
    
    def lwrist_subscriber(self, msg):
        self.lwrench = msg.wrench
        pass

    def rwrist_subscriber(self, msg):
        self.rwrench = msg.wrench
        pass


    def __cmd2pos__(self, cmd, left_right):
        if self.fk_service==None:
            raise Exception("Failed: fk_service")
        
        resp = self.fk_service(
            LeftRight = left_right,
            limbTwist = cmd,
            targetPos=[0, 0, 0], 
            targetOri=[0, 0, 0, 1]
        )
        return resp.limbPose


    def __pos2cmd__(self, pos, left_right, **kwargs):

        if "cur_cmd" in kwargs:
            cur_cmd = kwargs["cur_cmd"]
        else:
            if left_right == "left":
                cur_cmd = self.leftLimb_cmd
            else:
                cur_cmd = self.rightLimb_cmd


        if self.ik_service==None:
            raise Exception("Failed: ik_service")
        
        resp = self.ik_service(
            LeftRight = left_right,
            limbTwist = cur_cmd,
            targetPos=pos[:3], 
            targetOri=pos[3:7]
        )
        return resp.limbTwist

    def leftLimb_subscriber(self, msg):
        self.leftLimb_cmd = msg.position
        

    def rightLimb_subscriber(self, msg):
        self.rightLimb_cmd = msg.position


    def get_leftLimb_coordinate(self):
        return self.__cmd2pos__(self.leftLimb_cmd, "left")
    
    def get_rightLimb_coordinate(self):
        return self.__cmd2pos__(self.rightLimb_cmd, "right")

def stage1(robot):
    stopWalk()

    LShoulderPitch=-.15
    LShoulderRoll=-1.
    LShoulderYaw=-2.
    LElbowRoll=-2.0
    LElbowYaw=2.447
    LWristRoll=-.6
    LWristPitch=-.3

    tar_cmd = [LShoulderPitch,LShoulderRoll,LShoulderYaw,LElbowRoll,LElbowYaw,LWristRoll,LWristPitch]
    # tar_cmd = [-cmd for cmd in tar_cmd]
    move(node='leftLimb', tar_cmd=tar_cmd, duration=.5)

    LShoulderPitch=-.15
    LShoulderRoll=0
    LShoulderYaw=-1.
    LElbowRoll=-2.1
    LElbowYaw=2.447
    LWristRoll=-.6
    LWristPitch=-.3

    tar_cmd = [LShoulderPitch,LShoulderRoll,LShoulderYaw,LElbowRoll,LElbowYaw,LWristRoll,LWristPitch]
    # tar_cmd = [-cmd for cmd in tar_cmd]
    move(node='leftLimb', tar_cmd=tar_cmd, duration=.5)

    cur_pos = robot.__cmd2pos__(robot.leftLimb_cmd, left_right="left")
    cur_cmd = robot.__pos2cmd__(cur_pos, left_right="left")

    tar_pos = list(cur_pos)
    tar_pos[0] += .11
    tar_pos[1] += .023
    tar_pos[2] += -.05
    # robot.left_tar_pos = tar_pos
    # print("tar_pos", tar_pos)
    tar_cmd = robot.__pos2cmd__(tar_pos, left_right="left")
    print("tar_cmd", tar_cmd)
    tar_cmd = list(tar_cmd)
    tar_cmd[4] = 2.447
    tar_cmd[5] = -.3
    tar_cmd[6] = -.3
    move(node='leftLimb', tar_cmd=tar_cmd, duration=.5)
    robot.left_tar_pos = list(robot.__cmd2pos__(tar_cmd,"left"))
    # close left hand
    move(node="leftHand", tar_cmd=[1.5]*10)


    RShoulderPitch=.15
    RShoulderRoll=-1.
    RShoulderYaw=2.
    RElbowRoll=-2.0
    RElbowYaw=-2.447
    RWristRoll=.6
    RWristPitch=-.3
    tar_cmd = [RShoulderPitch,RShoulderRoll,RShoulderYaw,RElbowRoll,RElbowYaw,RWristRoll,RWristPitch]
    move(node='rightLimb', tar_cmd=tar_cmd, duration=.5)

    RShoulderPitch=.15
    RShoulderRoll=0
    RShoulderYaw=1.
    RElbowRoll=-2.1
    RElbowYaw=-2.447
    RWristRoll=.6
    RWristPitch=-.3
    tar_cmd = [RShoulderPitch,RShoulderRoll,RShoulderYaw,RElbowRoll,RElbowYaw,RWristRoll,RWristPitch]
    move(node='rightLimb', tar_cmd=tar_cmd, duration=.5)

    cur_pos = robot.__cmd2pos__(robot.rightLimb_cmd,left_right="right")
    cur_cmd = robot.__pos2cmd__(cur_pos,left_right="right")
    tar_pos = list(cur_pos)
    tar_pos[0] += .11
    tar_pos[1] += .01
    tar_pos[2] += -.05
    tar_cmd = robot.__pos2cmd__(tar_pos,left_right="right")
    tar_cmd = list(tar_cmd)
    tar_cmd[4] = -2.447
    tar_cmd[5] = .3
    tar_cmd[6] = -.3
    move(node='rightLimb', tar_cmd=tar_cmd, duration=.5)
    robot.right_tar_pos = list(robot.__cmd2pos__(tar_cmd,"right"))
    # # close right hand
    move(node="rightHand", tar_cmd=[1.5]*10)




def stage2(robot):

    print("start walking")
    startWalk()
    
    pub = rospy.Publisher('/nav/cmd_vel_nav', Twist, queue_size=10)
    left_limb_pub = rospy.Publisher('/walker/'+"left"+'/controller', JointCommand, queue_size=10)
    right_limb_pub = rospy.Publisher('/walker/'+"right"+'/controller', JointCommand, queue_size=10)
    rate = rospy.Rate(1000) # 10HZ
    while not rospy.is_shutdown():
        twist_msg = Twist()
        twist_msg.linear.x = 0.05
        
        pub.publish(twist_msg)
        # # close left hand
        # move(node="leftHand", tar_cmd=[1.5]*10, duration=0.002)
        # # close left hand
        # move(node="rightHand", tar_cmd=[1.5]*10, duration=0.002)
        
        # tar_pos[1] += robot.waistMeasured[1] #FIX

        mu = 1e-5 # OKay
        tar_pos = robot.left_tar_pos
        # mu = 5e-5
        cur_pos = robot.__cmd2pos__(robot.leftLimb_cmd, "left") # FIX
        tar_pos[0] = cur_pos[0]-robot.lwrench.force.x * mu if robot.lwrench.force.x < 5 else cur_pos[0] #FIX
        tar_pos[1] = cur_pos[1]-robot.lwrench.force.y * mu # FIX
        tar_pos[2] = cur_pos[2]-robot.lwrench.force.z * mu # FIX
        rospy.loginfo(tar_pos)
        tar_cmd = robot.__pos2cmd__(tar_pos, left_right="left")
        # print("tar_cmd", tar_cmd)
        tar_cmd = list(tar_cmd)
        tar_cmd[4] = 2.447
        tar_cmd[5] = -.3
        tar_cmd[6] = -.3
        # move(node='leftLimb', tar_cmd=tar_cmd, duration=.001)
        msg = JointCommand()
        msg.mode = 5
        msg.command = tar_cmd
        left_limb_pub.publish(msg)
        

        tar_pos = robot.right_tar_pos
        cur_pos = robot.__cmd2pos__(robot.rightLimb_cmd, "right")
        tar_pos[0] = cur_pos[0]-robot.rwrench.force.x * mu if robot.rwrench.force.x < 5 else cur_pos[0] #FIX
        tar_pos[1] = cur_pos[1]-robot.rwrench.force.y * mu
        tar_pos[2] = cur_pos[2]-robot.rwrench.force.z * mu

        tar_cmd = robot.__pos2cmd__(tar_pos,left_right="right")
        tar_cmd = list(tar_cmd)
        tar_cmd[4] = -2.447
        tar_cmd[5] = .3
        tar_cmd[6] = -.3
        # move(node='rightLimb', tar_cmd=tar_cmd, duration=.001)
        msg = JointCommand()
        msg.mode = 5
        msg.command = tar_cmd
        right_limb_pub.publish(msg)
        
        rate.sleep()
    pass

def main():
    rospy.init_node("task_6")
    load_scene()
    
    robot = Robot()

    # Grab the handle
    stage1(robot)
    
    # walk
    # exit()
    stage2(robot)
    
    rospy.spin()
if __name__ == "__main__":
    main()