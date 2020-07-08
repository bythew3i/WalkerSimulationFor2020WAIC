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

def leg_status_callback(d):
    global leg_status
    with lock:
        leg_status = str(d.data)

leg_status_listener = rospy.Subscriber("/Leg/leg_status", String, leg_status_callback, queue_size=10)

def wait_for_leg_service_data():
    global leg_status
    while leg_status is None:
        pass

# Goto navigation location
def goto_nav(task):
    rospy.wait_for_service("/walker/sence")
    select = rospy.ServiceProxy("/walker/sence", SceneSelection)
    try:
        select(scene_name=task, nav=True, vision=False)
    except rospy.ServiceException as exc:
        rospy.logerr("Navigation Prepare:: Error going to navigation position: %s", str(exc))

def start_dynamic():
    global leg_status
    wait_for_leg_service_data()

    with lock:
        cur_leg_status = leg_status
    
    req.cmd = "start"
    r = rospy.Rate(30)
    while not cur_leg_status == "dynamic":
        rospy.wait_for_service('/Leg/TaskScheduler', timeout=10)
        res = legmotion_service.call(req)
        with lock:
            cur_leg_status = leg_status
        r.sleep()

def stop_dynamic():
    global leg_status
    wait_for_leg_service_data()

    with lock:
        cur_leg_status = leg_status

    req.cmd = "stop"
    r = rospy.Rate(30)
    while not cur_leg_status == "standing":
        rospy.wait_for_service('/Leg/TaskScheduler', timeout=10)
        res = legmotion_service.call(req)
        with lock:
            cur_leg_status = leg_status
        r.sleep()

def wait_for_standing():
    global leg_status
    wait_for_leg_service_data()

    with lock:
        cur_leg_status = leg_status

    while not cur_leg_status == "standing":
        with lock:
            cur_leg_status = leg_status