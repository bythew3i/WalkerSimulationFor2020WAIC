#!/usr/bin/env python

import rospy
import roslaunch
from webots_api.srv import SceneSelection
from std_msgs.msg import String
import walker_srvs.srv

import threading

lock = threading.Lock()
leg_status = None

legmotion_service = rospy.ServiceProxy(
    "/Leg/TaskScheduler", 
    walker_srvs.srv.leg_motion_MetaFuncCtrl
)
req = walker_srvs.srv.leg_motion_MetaFuncCtrlRequest()
req.func_name = "dynamic"
req.param_json = ""
req.cmd = "start"

def leg_status_callback(d):
    with lock:
        leg_status = str(d.data)

leg_status_listener = rospy.Subscriber("/Leg/leg_status", String, leg_status_callback, queue_size=10)

# Goto navigation location
def goto_nav():
    rospy.wait_for_service("/walker/sence")
    select = rospy.ServiceProxy("/walker/sence", SceneSelection)
    try:
        select(scene_name="GraspCup", nav=True, vision=False)
    except rospy.ServiceException as exc:
        rospy.logerr("Navigation Prepare:: Error going to navigation position: %s", str(exc))

def start_dynamic():
    r = rospy.Rate(30)
    cur_leg_status = None
    while not cur_leg_status == "dynamic":
        rospy.wait_for_service('/Leg/TaskScheduler', timeout=10)
        res = legmotion_service.call(req)
        r.sleep()
        with lock:
            cur_leg_status = leg_status

def prepare():
    rospy.init_node('navigation_prepare', anonymous=True)
    
    rospy.loginfo("Navigation Prepare:: Moving behind sofa...")
    goto_nav()
    rospy.loginfo("Navigation Prepare:: Starting leg motion...")
    start_dynamic()
    rospy.loginfo("Navigation Prepare:: leg motion started...")

if __name__ == '__main__':
    try:
        prepare()
    except rospy.ROSInterruptException:
        pass