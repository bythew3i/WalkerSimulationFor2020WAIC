#!/usr/bin/env python

import rospy
from  webots_api.srv import SceneSelection, SceneSelectionRequest
# from geometry_msgs.msg import PoseStamped
from ubt_core_msgs.msg import JointCommand
from sensor_msgs.msg import JointState
# from std_msgs.msg import Header

import time
import math

def callback(msg):
    print(msg.position)
    # msg.

def load_scence():

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



def grab_callback(msg):
    pass

def grab():
    # expected_r_shoulder_pitch = math.pi/2
    # pub = rospy.Publisher("/walker/leftLimb/controller", JointCommand, queue_size=10)
    # msg = JointCommand()
    # r_shoulder_pitch_angle = 0
    # r_shoulder_pitch_velocity = 1
    # while r_shoulder_pitch_angle != expected_r_shoulder_pitch:
    #     msg.mode = 2
    #     msg.command = [0.0]*7
    #     msg.command[0] = r_shoulder_pitch_velocity
    #     pub.publish(msg)
    #     time.sleep(0.1)
    #     r_shoulder_pitch_angle += expected_r_shoulder_pitch

    # rospy.Subscriber("/walker/leftLimb/joint_states", PoseStamped, callback)
    rospy.Subscriber("/walker/leftLimb/joint_states", JointState, callback)
    rospy.spin()


def main():
    rospy.init_node("task_6")
    load_scence()
    
    grab()

if __name__ == "__main__":
    main()