#!/usr/bin/env python

import rospy
import walker_srvs.srv
from sensor_msgs.msg import Image, JointState
from ubt_core_msgs.msg import JointCommand
from std_msgs.msg import Int64, String
from geometry_msgs.msg import Twist, WrenchStamped
from webots_api.srv import SceneSelection
from math import pi
import cv_bridge
import cv2
from thewalkingdead.srv import Solver

class Task11(object):
    def __init__(self):
        self.log_msg = None
        self.head_depth = Image()
        self.step_num = Int64()
        self.leg_status = String()
        self.r_limb_states = JointState()
        self.r_wrist_wrench = WrenchStamped()

        self.mu = 1e-6

        # service manage
        self.leg_motion_service = None
        self.fk_service = None 
        self.ik_service = None

    def _trans(self, start, target, fac):
        return start + (target-start)*fac

    def _log(self, msg):
        if self.log_msg != msg:
            print(msg)
            self.log_msg = msg

    # Subscriber call backs
    def head_depth_cb(self, depth):
        self.head_depth = depth

    def step_num_cb(self, step):
        self.step_num = step

    def leg_status_cb(self, status):
        self.leg_status = status

    def r_limb_states_cb(self, states):
        self.r_limb_states = states

    def r_wrist_wrench_cb(self, wrench):
        self.r_wrist_wrench = wrench


    def __cmd2pos__(self, cmd):
        if self.fk_service==None:
            raise Exception("Failed: fk_service")
        
        resp = self.fk_service(
            LeftRight = "right",
            limbTwist = cmd,
            targetPos=[0, 0, 0], 
            targetOri=[0, 0, 0, 1]
        )
        return resp.limbPose


    def __pos2cmd__(self, pos, cur_cmd):
        if self.ik_service==None:
            raise Exception("Failed: ik_service")
        
        resp = self.ik_service(
            LeftRight = "right",
            limbTwist = cur_cmd,
            targetPos=pos[:3], 
            targetOri=pos[3:7]
        )
        return resp.limbTwist


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


    def measure_depth(self, off_r=0, off_c=0):
        r = self.head_depth.height / 2 + off_r
        c = self.head_depth.width / 2 + off_c
        bridge = cv_bridge.CvBridge()
        cv_depth = bridge.imgmsg_to_cv2(self.head_depth, desired_encoding="passthrough")
        cv_depth = cv_depth.astype(float)
        return cv_depth[r][c]


    def solve(self):
        rospy.init_node('task11', anonymous=True)

        ### Subscribers
        rospy.Subscriber("/walker/camera/headDepth", Image, self.head_depth_cb)
        rospy.Subscriber("/Leg/StepNum", Int64, self.step_num_cb)
        rospy.Subscriber("/Leg/leg_status", String, self.leg_status_cb)
        rospy.Subscriber("/walker/rightLimb/joint_states", JointState, self.r_limb_states_cb)
        rospy.Subscriber("/sensor/ft/rwrist", WrenchStamped, self.r_wrist_wrench_cb)

        ### Publishers
        head_ctrl_pub = rospy.Publisher('/walker/head/controller', JointCommand, queue_size=10)
        leg_motion_pub = rospy.Publisher('/nav/cmd_vel_nav', Twist, queue_size=10)
        r_limb_ctrl_pub = rospy.Publisher('/walker/rightLimb/controller', JointCommand, queue_size=10)
        r_hand_ctrl_pub = rospy.Publisher('/walker/rightHand/controller', JointCommand, queue_size=10)

        ### Messages
        head_ctrl_msg = JointCommand()
        head_ctrl_msg.command = [0 for _ in range(2)]
        leg_motion_msg = Twist()
        r_limb_ctrl_msg = JointCommand()
        r_limb_ctrl_msg.command = [0 for _ in range(7)]
        r_hand_ctrl_msg = JointCommand()
        r_hand_ctrl_msg.command = [0 for _ in range(10)]

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



        ## Select the scene
        self.scene_service(scene_name="OpenFridge", nav=False, vision=True)


        # SOME CONSTANTS
        # TAR_DEPTH_X = 1.3445
        # TAR_DEPTH_Y = 2.9363
        TAR_DEPTH_X = 1.365
        TAR_DEPTH_Y = 2.925
        MEASURE_Y_TIME = 1
        MEASURE_X_TIME = 1
        UNIT_Y_STEP = 0.04
        UNIT_X_STEP = 0.04
        UNIT_GRASP_TIME = 0.6 
        RHANDQUATERNION = [-0.14585886490562033, 0.05340793202959183, 0.8055789320525842, 0.5717651340872281]

        # Timing marks
        rate = rospy.Rate(1000)
        timer = 0
        timer_for = 0

        action = 0
        move_x = 0 # forward is postive, backward is negative
        move_y = 0 # right is postive, left is negative

        while not rospy.is_shutdown():

            if self.leg_status.data in ["stopping", "standInit"]:
                self._log("... Waiting to stable ...")

            elif action == 0:
                self._log("Action 0: Measure Y")
                if timer_for != action:
                    timer = 0
                    timer_for = action
                timer += 0.001

                if timer < MEASURE_Y_TIME:
                    fac = timer
                    head_ctrl_msg.mode = 5
                    head_ctrl_msg.command[0] = self._trans(0, -pi/2, fac)
                    head_ctrl_msg.command[1] = self._trans(0, -20.0/180*pi, fac)
                    
                    head_ctrl_pub.publish(head_ctrl_msg)
                elif timer < MEASURE_Y_TIME + 0.2:
                    # wait for 0.2 sec to be stable
                    pass
                else:
                    dis = self.measure_depth()
                    move_y = int(round((TAR_DEPTH_Y-dis) / UNIT_Y_STEP * 2))
                    print "\tCurrent depth Y", dis
                    print "\tMove Y", move_y
                    self.leg_motion_start()
                    action += 1

            elif action == 1:
                self._log("Action 1: Align on Y")
                if self.step_num.data < abs(move_y):
                    leg_motion_msg.linear.x = 0
                    leg_motion_msg.linear.y = UNIT_Y_STEP if move_y > 0 else -UNIT_Y_STEP
                    leg_motion_pub.publish(leg_motion_msg)
                else:
                    move_y = 0
                    self.leg_motion_stop()
                    action += 1
            elif action == 2:
                self._log("Action 2: Measure X")
                if timer_for != action:
                    timer = 0
                    timer_for = action
                timer += 0.001

                if timer < MEASURE_X_TIME:
                    fac = timer
                    head_ctrl_msg.mode = 5
                    head_ctrl_msg.command[0] = self._trans(-pi/2, 0, fac)
                    head_ctrl_msg.command[1] = -20.0/180*pi
                    head_ctrl_pub.publish(head_ctrl_msg)
                elif timer < MEASURE_X_TIME + 0.2:
                    # wait for 0.2 sec to be stable
                    pass
                else:
                    dis = self.measure_depth()
                    move_x = -int(round((TAR_DEPTH_X-dis) / UNIT_X_STEP))
                    print "\tCurrent depth X", dis
                    print "\tMove X", move_x
                    self.leg_motion_start()
                    action += 1
            elif action == 3:
                self._log("Action 3: Align on X")
                if self.step_num.data < abs(move_x):
                    leg_motion_msg.linear.x = UNIT_X_STEP if move_x > 0 else -UNIT_X_STEP
                    leg_motion_msg.linear.y = 0
                    leg_motion_pub.publish(leg_motion_msg)
                else:
                    move_x = 0
                    self.leg_motion_stop()
                    action += 1
            elif action==4:
                self._log("Action 4: raise right limb and hold the fridge handle")
                if timer_for != action:
                    timer = 0
                    timer_for = action
                timer += 0.001

                if timer < UNIT_GRASP_TIME * 2:
                    fac = timer / (UNIT_GRASP_TIME * 2)
                    # hand
                    r_hand_ctrl_msg.mode = 5
                    r_hand_ctrl_msg.command[2] = self._trans(0, 0.8, fac)
                    r_hand_ctrl_msg.command[4] = self._trans(0, 0.8, fac)
                    r_hand_ctrl_msg.command[6] = self._trans(0, 0.8, fac)
                    r_hand_ctrl_msg.command[8] = self._trans(0, 0.8, fac)
                    r_hand_ctrl_pub.publish(r_hand_ctrl_msg)

                    # limb
                    r_limb_ctrl_msg.mode = 5
                    r_limb_ctrl_msg.command[0] = self._trans(0, -0.9, fac)
                    r_limb_ctrl_msg.command[1] = self._trans(0, -0.3, fac)
                    r_limb_ctrl_msg.command[2] = self._trans(0, 1.57, fac)
                    r_limb_ctrl_msg.command[3] = self._trans(0, -1.2, fac)
                    r_limb_ctrl_msg.command[4] = self._trans(0, -1.57, fac)
                    r_limb_ctrl_msg.command[5] = self._trans(0, 0.4, fac)
                    r_limb_ctrl_msg.command[6] = self._trans(0, 0.1, fac)
                    r_limb_ctrl_pub.publish(r_limb_ctrl_msg)
    
                elif timer < UNIT_GRASP_TIME * 3:
                    fac = (timer - UNIT_GRASP_TIME * 2) / UNIT_GRASP_TIME
                    r_limb_ctrl_msg.command[1] = self._trans(-0.3, 0, fac)
                    r_limb_ctrl_pub.publish(r_limb_ctrl_msg)

                elif timer < UNIT_GRASP_TIME * 4:
                    fac = (timer - UNIT_GRASP_TIME * 3) / UNIT_GRASP_TIME
                    r_hand_ctrl_msg.command[0] = self._trans(0, 1, fac)
                    r_hand_ctrl_msg.command[1] = self._trans(0, 1, fac)
                    r_hand_ctrl_msg.command[2] = self._trans(0.8, 1.57, fac)
                    r_hand_ctrl_msg.command[3] = self._trans(0, 1.57, fac)
                    r_hand_ctrl_msg.command[4] = self._trans(0.8, 1.57, fac)
                    r_hand_ctrl_msg.command[5] = self._trans(0, 1.57, fac)
                    r_hand_ctrl_msg.command[6] = self._trans(0.8, 1.57, fac)
                    r_hand_ctrl_msg.command[7] = self._trans(0, 1.57, fac)
                    r_hand_ctrl_msg.command[8] = self._trans(0.8, 1.57, fac)
                    r_hand_ctrl_msg.command[9] = self._trans(0, 1.57, fac)
                    r_hand_ctrl_pub.publish(r_hand_ctrl_msg)
                else:
                    action += 1
                    self.leg_motion_start()
            elif action==5:
                self._log("Action 5: hold the handle and move to open the door")
                
                fx = self.r_wrist_wrench.wrench.force.x
                fy = self.r_wrist_wrench.wrench.force.y
                fz = self.r_wrist_wrench.wrench.force.z

                cur_cmd = self.r_limb_states.position
                cur_pos = self.__cmd2pos__(cur_cmd)

                tar_pos = list(cur_pos[:3]) + RHANDQUATERNION
                tar_pos[0] -= fx * self.mu if fx > -5 else 0
                tar_pos[1] -= fy * self.mu
                tar_pos[2] -= fz * self.mu 

                tar_cmd = self.__pos2cmd__(tar_pos, cur_cmd)
                for i in range(len(tar_cmd)):
                    r_limb_ctrl_msg.command[i] = tar_cmd[i]
                
                r_limb_ctrl_pub.publish(r_limb_ctrl_msg)

                if self.step_num.data < 42:
                    leg_motion_msg.linear.x = -0.04
                    leg_motion_msg.linear.y = -0.04
                    leg_motion_msg.angular.z = 0.1
                    leg_motion_pub.publish(leg_motion_msg)

                else:
                    action += 1
                    self.leg_motion_stop()
            else:
                break



            rate.sleep()


if __name__ == '__main__':
    try:
        task = Task11()
        task.solve()
    except rospy.ROSInterruptException as e:
        print("ROS Interrupted: {}".format(e))

