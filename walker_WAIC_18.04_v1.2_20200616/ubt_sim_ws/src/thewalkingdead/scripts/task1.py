#!/usr/bin/env python

import rospy
import walker_srvs.srv
from ubt_core_msgs.msg import JointCommand
from sensor_msgs.msg import JointState
from webots_api.srv import SceneSelection

def stage1():
    hand_pub = rospy.Publisher('/walker/leftHand/controller', JointCommand, queue_size=10)
    limb_pub = rospy.Publisher('/walker/leftLimb/controller', JointCommand, queue_size=10)

    duration = 0.7
    time = 0
    hand_cur = rospy.wait_for_message("/walker/leftHand/joint_states", JointState).position
    limb_cur = rospy.wait_for_message("/walker/leftLimb/joint_states", JointState).position

    hand_tar = [0.785, 0.785, 0, 0, 1.571, 1.571, 1.571, 1.571, 1.571, 1.571]
    limb_tar = [0.4, -0.82, -1.571, -2.3, 2.355, -0.10, 0.555]

    hand_per_step = [(tar - cur) / duration * 0.001 for cur, tar in zip(hand_cur, hand_tar)]
    limb_per_step = [(tar - cur) / duration * 0.001 for cur, tar in zip(limb_cur, limb_tar)]

    rate = rospy.Rate(1000)
    while time < duration*1000:
        time += 1
        hand_cur = [cur+step for cur, step in zip(hand_cur, hand_per_step)]
        limb_cur = [cur+step for cur, step in zip(limb_cur, limb_per_step)]

        handmsg = JointCommand()
        limbmsg = JointCommand()
        handmsg.mode = 5
        limbmsg.mode = 5
        handmsg.command = hand_cur
        limbmsg.command = limb_cur

        hand_pub.publish(handmsg)
        limb_pub.publish(limbmsg)

        rate.sleep()

def stage2():
    limb_pub = rospy.Publisher('/walker/leftLimb/controller', JointCommand, queue_size=10)

    duration = 0.5
    time = 0
    limb_cur = rospy.wait_for_message("/walker/leftLimb/joint_states", JointState).position

    limb_tar = [0.95, -0.82, -1.571, -1.50, 2.355, -0.10, 0.555]

    limb_per_step = [(tar - cur) / duration * 0.001 for cur, tar in zip(limb_cur, limb_tar)]

    rate = rospy.Rate(1000)
    while time < duration*1000:
        time += 1
        limb_cur = [cur+step for cur, step in zip(limb_cur, limb_per_step)]

        limbmsg = JointCommand()
        limbmsg.mode = 5
        limbmsg.command = limb_cur

        limb_pub.publish(limbmsg)

        rate.sleep()


def goto_task1():
    rospy.wait_for_service("/walker/sence")
    select = rospy.ServiceProxy("/walker/sence", SceneSelection)
    try:
        select(scene_name="SwitchLight", nav=False, vision=False)
    except rospy.ServiceException as exc:
        print(str(exc))

def task1():
    rospy.init_node('switch_light_motion', anonymous=True)

    interllude = rospy.Duration(0.5, 0)
    
    print("Moving to task1...")
    goto_task1()
    print("stage1...")
    stage1()
    print("stage2...")
    stage2()

if __name__ == '__main__':
    try:
        task1()
    except rospy.ROSInterruptException:
        pass