#!/usr/bin/env python

import rospy
import walker_srvs.srv
import tf
from ubt_core_msgs.msg import JointCommand
from sensor_msgs.msg import JointState
from webots_api.srv import SceneSelection
from thewalkingdead.srv import Solver

def goto_task():
    rospy.wait_for_service("/walker/sence")
    select = rospy.ServiceProxy("/walker/sence", SceneSelection)
    try:
        select(scene_name="GraspCup", nav=False, vision=False)
    except rospy.ServiceException as exc:
        print(str(exc))

def stage1():
    # Send to solver

    limb_cur = rospy.wait_for_message('/walker/leftLimb/joint_states', JointState).position
    target_pos = [-0.015880440026308996, 0.3586736446689998, -0.494047684295732]
    target_eul = [1.570798949528835, -0.0016463071519337736, 1.5707896460175066]

    target_pos = [p+x for p, x in zip(target_pos, [0.1, 0.2, 0.2])]
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


def stage2():
    # Send to solver

    limb_cur = rospy.wait_for_message('/walker/leftLimb/joint_states', JointState).position
    target_pos = [-0.015880440026308996, 0.3586736446689998, -0.494047684295732]
    target_eul = [1.570798949528835, -0.0016463071519337736, 1.5707896460175066]

    target_pos = [p+x for p, x in zip(target_pos, [0.23, 0.2, 0.28])]
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
    print("Solved: ", limb_tar)
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


def stage3():
    # Send to solver

    limb_cur = rospy.wait_for_message('/walker/leftLimb/joint_states', JointState).position
    target_pos = [-0.015880440026308996, 0.3586736446689998, -0.494047684295732]
    target_eul = [1.570798949528835, -0.0016463071519337736, 1.5707896460175066]

    target_pos = [p+x for p, x in zip(target_pos, [0.35, 0.08, 0.28])]
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
    print("Solved: ", limb_tar)
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

def stage4():
    # Send to solver

    limb_cur = rospy.wait_for_message('/walker/leftLimb/joint_states', JointState).position
    target_pos = [-0.015880440026308996, 0.3586736446689998, -0.494047684295732]
    target_eul = [1.570798949528835, -0.0016463071519337736, 1.5707896460175066]

    target_pos = [p+x for p, x in zip(target_pos, [0.358, 0.08, 0.26])]
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
    print("Solved: ", limb_tar)
    limb_pub = rospy.Publisher('/walker/leftLimb/controller', JointCommand, queue_size=10)

    time = 0
    duration = 0.25
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

    hand_tar = [0.3927, 0.3927, 0.785, 0.3927, 0.785, 0.3927, 0.785, 0.3927, 0.785, 0.3927]

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


def stage5():
    # Send to solver

    limb_cur = rospy.wait_for_message('/walker/leftLimb/joint_states', JointState).position
    target_pos = [-0.015880440026308996, 0.3586736446689998, -0.394047684295732]
    target_eul = [1.370798949528835, -0.0016463071519337736, 1.2707896460175066]

    target_pos = [p+x for p, x in zip(target_pos, [0.36, 0.08, 0.36])]
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
    print("Solved: ", limb_tar)
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


def task2():
    rospy.init_node('grasp_cup_motion', anonymous=True)
    
    print("Moving to task...")
    goto_task()
    print("Stage1...")
    stage1()
    print("Stage2...")
    stage2()
    print("Stage3...")
    stage3()
    print("Stage4...")
    stage4()
    print("Grasp...")
    grasp()
    print("Stage5...")
    stage5()

if __name__ == '__main__':
    try:
        task2()
    except rospy.ROSInterruptException:
        pass