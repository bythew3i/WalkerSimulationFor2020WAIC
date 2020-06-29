#!/usr/bin/env python

import rospy
import walker_srvs.srv
from ubt_core_msgs.msg import JointCommand
from std_msgs.msg import Int64, String
from geometry_msgs.msg import Twist, WrenchStamped
from sensor_msgs.msg import JointState
from thewalkingdead.srv import Solver
from webots_api.srv import SceneSelection
import time


class Solver10(object):
    def __init__(self):

        self.logmsg = ""
        self.stepnum = Int64()
        self.legstatus = String()
        self.rightlimbstates = JointState()
        self.rwristwrench = WrenchStamped()

        self.mu = 1e-6

        # service manage
        self.scene_service = None
        self.legmotion_service = None 
        self.ik_service = None 
        self.fk_service = None



    @staticmethod
    def resize(arr, sz):
        for _ in range(sz):
            arr.append(0)

    @staticmethod
    def copy(frm, to):
        for i in range(len(frm)):
            to[i] = frm[i]

    def log(self, msg):
        if msg!=self.logmsg:
            self.logmsg = msg
            print(self.logmsg)


    # stepnum subscriber callback
    def stepnum_cb(self, stepnum):
        self.stepnum.data = stepnum.data

    # legstatus subscriber callback
    def legstatus_cb(self, legstatus):
        self.legstatus.data = legstatus.data

    # rightlimbstates subscriber callback
    def rightlimbstates_cb(self, rightlimbstates):
        self.rightlimbstates = rightlimbstates

    # rwristwrench subscriber callback
    def rwristwrench_cb(self, rwristwrench):
        self.rwristwrench = rwristwrench

    # Start legmotion
    def legmotion_start(self):
        if self.legmotion_service != None:
            req = walker_srvs.srv.leg_motion_MetaFuncCtrlRequest()
            req.func_name = "dynamic"
            req.param_json = ""
            req.cmd = "start"
            # response
            res = self.legmotion_service(req)
            if res.success:
                return
        raise Exception("Failed: legmotion_start")

    # Stop legmotion
    def legmotion_stop(self):
        if self.legmotion_service != None:
            req = walker_srvs.srv.leg_motion_MetaFuncCtrlRequest()
            req.func_name = "dynamic"
            req.param_json = ""
            req.cmd = "stop"
            # response
            res = self.legmotion_service(req)
            if res.success:
                return
        raise Exception("Failed: legmotion_stop")


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



    def solve(self):
        rospy.init_node('task10', anonymous=True)

        ### Subscribers
        rospy.Subscriber("/Leg/StepNum", Int64, self.stepnum_cb)
        rospy.Subscriber("/Leg/leg_status", String, self.legstatus_cb)
        rospy.Subscriber("/walker/rightLimb/joint_states", JointState, self.rightlimbstates_cb)
        rospy.Subscriber("/sensor/ft/rwrist", WrenchStamped, self.rwristwrench_cb)

        ### Publishers
        legmotion_pub = rospy.Publisher('/nav/cmd_vel_nav', Twist, queue_size=10)
        rightlimb_pub = rospy.Publisher('/walker/rightLimb/controller', JointCommand, queue_size=10)
        righthand_pub = rospy.Publisher('/walker/rightHand/controller', JointCommand, queue_size=10)

        ### Messages
        legmotion_msg = Twist()
        rightlimb_msg = JointCommand()
        Solver10.resize(rightlimb_msg.command, 7)
        righthand_msg = JointCommand()
        Solver10.resize(righthand_msg.command, 10)


        ### Services
        rospy.wait_for_service('/walker/sence', timeout=10)
        self.scene_service = rospy.ServiceProxy(
            "/walker/sence", 
            SceneSelection
        )

        rospy.wait_for_service('/Leg/TaskScheduler', timeout=10)
        self.legmotion_service = rospy.ServiceProxy(
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
        self.scene_service(scene_name="OpenFridge", nav=False, vision=False)

        # Start Leg Motion 
        self.legmotion_start()

        # Timing marks
        rate = rospy.Rate(1000)
        timer = 0
        timer_for = 0


        action = 1

        # some CONSTANTS
        ACTSTEPTIME = 0.6
        RHANDQUATERNION = [-0.14585886490562033, 0.05340793202959183, 0.8055789320525842, 0.5717651340872281]


        while not rospy.is_shutdown():
            # time += 0.001

            ## waiting for stable status
            if self.legstatus.data in ["stopping", "standInit"]:
                self.log("Waiting to stable")
            
            ## Action1: move back for extra space
            elif action==1:
                self.log("Action 1: move back for extra space")
                if self.stepnum.data < 2:
                    if timer_for != 1:
                        # timer 1 start
                        timer_for = 1
                        timer = 0

                    timer += 0.001
                    fac = timer/(ACTSTEPTIME*2)
                    if fac > 1:
                        fac = 1

                    righthand_msg.command[2] = 0.8 * fac
                    righthand_msg.command[4] = 0.8 * fac
                    righthand_msg.command[6] = 0.8 * fac
                    righthand_msg.command[8] = 0.8 * fac

                    righthand_pub.publish(righthand_msg)
                    # rightlimb_pub.publish(rightlimb_msg)

                    legmotion_msg.linear.x = -0.1
                    legmotion_msg.linear.y = 0.04
                    legmotion_pub.publish(legmotion_msg)
                else:
                    self.legmotion_stop()
                    action = 2

            ## Action 2: raise right limb and hold the fridge handle
            elif action==2:
                self.log("Action 2: raise right limb and hold the fridge handle")
                if timer_for != 2:
                    # timer 2 start
                    timer_for = 2
                    timer = 0
                
                timer += 0.001
                if timer < ACTSTEPTIME*2:
                    fac = timer/(ACTSTEPTIME*2)
                    rightlimb_msg.command[0] = -0.9 * fac
                    rightlimb_msg.command[1] = -0.3 * fac
                    rightlimb_msg.command[2] = 1.57 * fac
                    rightlimb_msg.command[3] = -1.2 * fac
                    rightlimb_msg.command[4] = -1.57 * fac
                    rightlimb_msg.command[5] = 0.4 * fac
                    rightlimb_msg.command[6] = 0.1 * fac

                    righthand_pub.publish(righthand_msg)
                    rightlimb_pub.publish(rightlimb_msg)
                elif timer < ACTSTEPTIME*3:
                    fac = (timer-ACTSTEPTIME*2)/(ACTSTEPTIME*1)
                    rightlimb_msg.command[1] = -0.3 + 0.3 * fac

                    righthand_pub.publish(righthand_msg)
                    rightlimb_pub.publish(rightlimb_msg)

                elif timer < ACTSTEPTIME*4:
                    fac = (timer-ACTSTEPTIME*3)/(ACTSTEPTIME*1)
                    righthand_msg.command[0] = 1 * fac
                    righthand_msg.command[1] = 1 * fac
                    righthand_msg.command[2] = 0.8 + (1.57-0.8) * fac
                    righthand_msg.command[3] = 1.57 * fac
                    righthand_msg.command[4] = 0.8 + (1.57-0.8) * fac
                    righthand_msg.command[5] = 1.57 * fac
                    righthand_msg.command[6] = 0.8 + (1.57-0.8) * fac
                    righthand_msg.command[7] = 1.57 * fac
                    righthand_msg.command[8] = 0.8 + (1.57-0.8) * fac
                    righthand_msg.command[9] = 1.57 * fac

                    righthand_pub.publish(righthand_msg)

                else:
                    action = 3
                    self.legmotion_start()

            # Action3: hold the handle and move to open the door
            elif action==3:
                self.log("Action3: hold the handle and move to open the door")

                fx = self.rwristwrench.wrench.force.x
                fy = self.rwristwrench.wrench.force.y
                fz = self.rwristwrench.wrench.force.z

                cur_cmd = self.rightlimbstates.position
                cur_pos = self.__cmd2pos__(cur_cmd)

                # tar_pos = list(cur_pos[:7])
                tar_pos = list(cur_pos[:3]) + RHANDQUATERNION
                tar_pos[0] -= fx * self.mu if fx > -5 else 0
                tar_pos[1] -= fy * self.mu
                tar_pos[2] -= fz * self.mu 


                tar_cmd = self.__pos2cmd__(tar_pos, cur_cmd)

                Solver10.copy(tar_cmd, rightlimb_msg.command)
                rightlimb_pub.publish(rightlimb_msg)


                if self.stepnum.data < 42:
                    legmotion_msg.linear.x = -0.04
                    legmotion_msg.linear.y = -0.04
                    legmotion_msg.angular.z = 0.1
                    legmotion_pub.publish(legmotion_msg)

                else:
                    self.legmotion_stop()
                    action = 4

                
            else:
                self.log("No action is performing")
                break

            rate.sleep()
        
        

if __name__ == '__main__':
    try:
        print("\nWait 10 seconds to start all required services ...\n")
        time.sleep(10)
        solver = Solver10()
        solver.solve()
    except rospy.ROSInterruptException as e:
        print("ROS Interrupted: {}".format(e))


#   force: 
#     x: -2.13191941941
#     y: 6.56089391092
#     z: -0.613323992859
