#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import walker_srvs.srv
import tf
from ubt_core_msgs.msg import JointCommand
from sensor_msgs.msg import JointState
from webots_api.srv import SceneSelection
from thewalkingdead.srv import Solver

import math

def point():
    hand_pub = rospy.Publisher('/walker/leftHand/controller', JointCommand, queue_size=10)

    duration = 0.5
    time = 0
    hand_cur = rospy.wait_for_message("/walker/leftHand/joint_states", JointState).position

    hand_tar = [0.785, 0.785, 0, 0, 1.571, 1.571, 1.571, 1.571, 1.571, 1.571]

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

def test_solver():
    # Send to solver
    limb_cur = rospy.wait_for_message('/walker/leftLimb/joint_states', JointState).position
    target_pos = [0.3, 0.1, 0.0]
    target_ori = tf.transformations.quaternion_from_euler(-0.785, 0.3927, 0.785)
    
    rospy.wait_for_service('inverse_kinematic_solver')
    try:
        kine_solver = rospy.ServiceProxy('inverse_kinematic_solver', Solver)
        resp = kine_solver(LeftRight='left', limbTwist=limb_cur, 
                                targetPos=target_pos, targetOri=target_ori)
    except rospy.ServiceException as e:
        print("Service call failed %s" % e)
        return
    
    limb_tar = resp.limbTwist
    print("Solved: ", limb_tar)
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

def test_solver2():
    # Send to solver
    limb_cur = rospy.wait_for_message('/walker/leftLimb/joint_states', JointState).position
    target_pos = [0.3, 0.4, 0.0]
    target_ori = tf.transformations.quaternion_from_euler(-0.785, 0.3927, 1.571)
    
    rospy.wait_for_service('inverse_kinematic_solver')
    try:
        kine_solver = rospy.ServiceProxy('inverse_kinematic_solver', Solver)
        resp = kine_solver(LeftRight='left', limbTwist=limb_cur, 
                                targetPos=target_pos, targetOri=target_ori)
    except rospy.ServiceException as e:
        print("Service call failed %s" % e)
        return
    
    limb_tar = resp.limbTwist
    print("Solved: ", limb_tar)
    limb_pub = rospy.Publisher('/walker/leftLimb/controller', JointCommand, queue_size=10)

    time = 0
    duration = 5
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

def test_solver3():
    # Send to solver
    limb_cur = rospy.wait_for_message('/walker/leftLimb/joint_states', JointState).position
    target_pos = [0.45, 0.4, 0.0]
    target_ori = tf.transformations.quaternion_from_euler(-0.785, 0.3927, 1.571)
    
    rospy.wait_for_service('inverse_kinematic_solver')
    try:
        kine_solver = rospy.ServiceProxy('inverse_kinematic_solver', Solver)
        resp = kine_solver(LeftRight='left', limbTwist=limb_cur, 
                                targetPos=target_pos, targetOri=target_ori)
    except rospy.ServiceException as e:
        print("Service call failed %s" % e)
        return
    
    limb_tar = resp.limbTwist
    print("Solved: ", limb_tar)
    limb_pub = rospy.Publisher('/walker/leftLimb/controller', JointCommand, queue_size=10)

    time = 0
    duration = 5
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

def test_solver4():
    # Send to solver
    limb_cur = rospy.wait_for_message('/walker/leftLimb/joint_states', JointState).position
    target_pos = [0.45, 0.1, 0.0]
    target_ori = tf.transformations.quaternion_from_euler(-0.785, 0.3927, 0.785)
    
    rospy.wait_for_service('inverse_kinematic_solver')
    try:
        kine_solver = rospy.ServiceProxy('inverse_kinematic_solver', Solver)
        resp = kine_solver(LeftRight='left', limbTwist=limb_cur, 
                                targetPos=target_pos, targetOri=target_ori)
    except rospy.ServiceException as e:
        print("Service call failed %s" % e)
        return
    
    limb_tar = resp.limbTwist
    print("Solved: ", limb_tar)
    limb_pub = rospy.Publisher('/walker/leftLimb/controller', JointCommand, queue_size=10)

    time = 0
    duration = 5
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

def test_solver_circle():
    
    def goto(tar_joint, duration, rate, pub):
        r = rospy.Rate(rate)
        time = 0
        timestep = 1.0 / rate
        
        cur_joint = rospy.wait_for_message('/walker/leftLimb/joint_states', JointState).position
        step = [(tar - cur) / duration / rate for tar, cur in zip(tar_joint, cur_joint)]

        print("target joint: ", tar_joint)
        while time < duration:
            print ("time: ", time, "duration", duration)
            time += timestep
            cur_joint = [cur+st for cur, st in zip(cur_joint, step)]
            print(cur_joint)

            limb_msg = JointCommand()
            limb_msg.mode = 5
            limb_msg.command = cur_joint
            pub.publish(limb_msg)
            r.sleep()

    radius = 0.075
    theta = -1.571

    limb_pub = rospy.Publisher('/walker/leftLimb/controller', JointCommand, queue_size=10)
    kine_solver = rospy.ServiceProxy('inverse_kinematic_solver', Solver)

    cur = 0
    frames = 40
    theta_step = 3.1415*4 / frames

    while cur < frames:
        print ("theta: ", theta)
        cur += 1
        theta += theta_step

        limb_cur = rospy.wait_for_message('/walker/leftLimb/joint_states', JointState).position
        target_pos = [radius * math.cos(theta) + 0.3, radius * math.sin(theta) + 0.1, 0.0]
        target_ori = tf.transformations.quaternion_from_euler(-0.785, 0.3927, 1.571 * 0.5)

        print("target_pos", target_pos)

        rospy.wait_for_service('inverse_kinematic_solver')
        try:
            resp = kine_solver(LeftRight='left', limbTwist=limb_cur, 
                                    targetPos=target_pos, targetOri=target_ori)
        except rospy.ServiceException as e:
            print("Service call failed %s" % e)
            return
        
        limb_tar = resp.limbTwist
        print ("Solved: ", limb_tar)

        goto(limb_tar, 0.1, 100, limb_pub)

def initial_pose_left():
    # Send to solver
    limb_cur = [0, 0, 0, 0, 0, 0, 0] # 初始关节参数
    target_pos = [0, 0, 0] #dummy list
    target_ori = [0, 0, 0, 1] #dummy list
    
    rospy.wait_for_service('forward_kinematic_solver')
    try:
        # 由于FK和IK共用同一种Service type，FK的时候targetPose和targetOri只需要
        # 发送两个dummy list就好
        kine_solver = rospy.ServiceProxy('forward_kinematic_solver', Solver)
        resp = kine_solver(LeftRight='left', limbTwist=limb_cur, 
                                targetPos=target_pos, targetOri=target_ori)
    except rospy.ServiceException as e:
        print("Service call failed %s" % e)
        return
    
    # FK的返回值存在limbPose里面，IK的存在limbTwist里面
    limbPose = resp.limbPose
    print("Solved position: ", limbPose[0:3])
    print("Solved orientation: ", tf.transformations.euler_from_quaternion(limbPose[3:7]))


def task2():
    rospy.init_node('switch_light_motion', anonymous=True)

    # 前向运动学demo，计算初始左手掌的坐标和姿态角
    initial_pose_left()

    # 画方形demo
    point()
    test_solver()
    test_solver2()
    test_solver3()
    test_solver4()
    test_solver()

    # 画圈圈demo
    # point()
    # test_solver()
    # test_solver_circle()


if __name__ == '__main__':
    try:
        task2()
    except rospy.ROSInterruptException:
        pass