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

def test_solver():
    # Send to solver
    limb_cur = rospy.wait_for_message('/walker/leftLimb/joint_states', JointState).position
    target_pos = [0.4, 0.0, 0.0]
    target_ori = tf.transformations.quaternion_from_euler(0, 0, 0)
    
    rospy.wait_for_service('kinematic_solver')
    try:
        kine_solver = rospy.ServiceProxy('kinematic_solver', Solver)
        resp = kine_solver(LeftRight='left', limbTwist=limb_cur, 
                                targetPos=target_pos, targetOri=target_ori)
    except rospy.ServiceException as e:
        print("Service call failed %s" % e)
        return
    
    limb_tar = resp.limbTwist
    print("Solved: ", limb_tar)
    limb_pub = rospy.Publisher('/walker/leftLimb/controller', JointCommand, queue_size=10)

    time = 0
    duration = 2
    step = [(tar - cur) / duration * 0.001 for tar, cur in zip(limb_tar, limb_cur)]

    rate = rospy.Rate(1e3)
    while time < duration * 1000:
        time += 1
        limb_cur = [cur + st for cur, st in zip(limb_cur, step)]

        limb_msg = JointCommand()
        limb_msg.mode = 5
        limb_msg.commad = limb_cur

        limb_pub.publish(limb_msg)

        rate.sleep()

def task2():
    rospy.init_node('switch_light_motion', anonymous=True)

    interllude = rospy.Duration(0.5, 0)
    
    # print("Moving to task...")
    # goto_task()
    test_solver()


if __name__ == '__main__':
    try:
        task2()
    except rospy.ROSInterruptException:
        pass