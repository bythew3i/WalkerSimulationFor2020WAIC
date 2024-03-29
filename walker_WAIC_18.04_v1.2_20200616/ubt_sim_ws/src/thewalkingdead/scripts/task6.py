#! /usr/bin/env python

import rospy
from webots_api.srv import SceneSelection, SceneSelectionRequest
from walker_srvs.srv import leg_motion_MetaFuncCtrl, leg_motion_MetaFuncCtrlRequest
from std_msgs.msg import Int64, String
from geometry_msgs.msg import Twist, WrenchStamped
from ubt_core_msgs.msg import JointCommand
from sensor_msgs.msg import JointState
from thewalkingdead.srv import Solver
import time
from math import cos, sin, pi

class Robot():
    def __init__(self):
        self.leftLimb_cmd = None
        self.rightLimb_cmd = None
        self.leftLimb_pos = None
        self.rightLimb_pos = None
        self.leftHand_cmd = None
        self.rightHand_cmd = None
        self.tar_leftLimb_cmd = None
        self.tar_rightLimb_cmd = None
        self.tar_leftHand_cmd = None
        self.tar_rightHand_cmd = None
        self.tar_leftLimb_pos = None
        self.tar_rightLimb_pos = None
        self.step_leftLimb_cmd = None
        self.step_rightLimb_cmd = None
        self.step_leftHand_cmd = None
        self.step_rightHand_cmd = None
        self.lwrist_sensor = None
        self.rwrist_sensor = None
        self.leg_step_num = None
        self.leg_status = None

                 
        # Establish services
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
        rospy.wait_for_service("/Leg/TaskScheduler", timeout=10)
        self.legmotion_service = rospy.ServiceProxy(
            "/Leg/TaskScheduler",
            leg_motion_MetaFuncCtrl
            )
        
        
        # Subscribe topics
        rospy.Subscriber(
            "/walker/leftLimb/joint_states",
            JointState,
            self.leftLimbSubscriber
            )
        rospy.Subscriber(
            "/walker/rightLimb/joint_states",
            JointState,
            self.rightLimbSubscriber
            )
        rospy.Subscriber(
            "/walker/leftHand/joint_states",
            JointState,
            self.leftHandSubscriber
        )
        rospy.Subscriber(
            "/walker/rightHand/joint_states",
            JointState,
            self.rightHandSubscriber
        )
        rospy.Subscriber(
            "/sensor/ft/lwrist",
            WrenchStamped,
            self.lwristSensorSubscriber
        )
        rospy.Subscriber(
            "/sensor/ft/rwrist",
            WrenchStamped,
            self.rwristSensorSubscriber
        )
        rospy.Subscriber(
            "/Leg/StepNum",
            Int64,
            self.legStepNumSubscriber
        )
        rospy.Subscriber(
            "/Leg/leg_status",
            String,
            self.legStatusCallback
        )
        
        # Initialize publishers
        self.leftLimbPublisher = rospy.Publisher(
            '/walker/leftLimb/controller',
            JointCommand,
            queue_size=10
            )
        self.rightLimbPublisher = rospy.Publisher(
            '/walker/rightLimb/controller',
            JointCommand, queue_size=10
            )
        self.leftHandPublisher = rospy.Publisher(
            '/walker/leftHand/controller',
            JointCommand, queue_size=10
        )
        self.rightHandPublisher = rospy.Publisher(
            '/walker/rightHand/controller',
            JointCommand, queue_size=10
        )
        self.legmotion_publisher = rospy.Publisher(
            '/nav/cmd_vel_nav',
            Twist,
            queue_size=10
            )

        
        # self.leg
    
    # Start legmotion
    def legmotion_start(self):
        if self.legmotion_service != None:
            try:
                req = leg_motion_MetaFuncCtrlRequest()
                req.func_name = "dynamic"
                req.param_json = ""
                req.cmd = "start"
                # response
                res = self.legmotion_service(req)
                if res.success:
                    return
            except Exception as e:
                print(e)
        # raise Exception("Failed: legmotion_start")

    # Stop legmotion
    def legmotion_stop(self):
        if self.legmotion_service != None:
            req = leg_motion_MetaFuncCtrlRequest()
            req.func_name = "dynamic"
            req.param_json = ""
            req.cmd = "stop"
            # response
            res = self.legmotion_service(req)
            print(res)
            if res.success:
                return
        raise Exception("Failed: legmotion_stop")
    
    
    # Subscriber callbacks
    def leftLimbSubscriber(self, msg):
        self.leftLimb_cmd = msg.position
        
    def rightLimbSubscriber(self, msg):
        self.rightLimb_cmd = msg.position
    
    def leftHandSubscriber(self,msg):
        self.leftHand_cmd = msg.position
    
    def rightHandSubscriber(self, msg):
        self.rightHand_cmd = msg.position
    
    def lwristSensorSubscriber(self, msg):
        self.lwrist_sensor = msg.wrench
    
    def rwristSensorSubscriber(self, msg):
        self.rwrist_sensor = msg.wrench
    
    def legStepNumSubscriber(self,msg):
        self.leg_step_num = msg.data
        
    def legStatusCallback(self,msg):
        self.leg_status = msg.data
    
    # @staticmethod  
    def __cmd2pos__(self, cmd, left_right):
        if self.fk_service==None:
            raise Exception("Failed: fk_service")
        resp = self.fk_service(
            LeftRight = left_right,
            limbTwist = cmd,
            targetPos=[0, 0, 0], 
            targetOri=[0, 0, 0, 1]
        )
        return list(resp.limbPose)
    
    # @staticmethod
    def __pos2cmd__(self, pos, cmd, left_right):
        if self.ik_service==None:
            raise Exception("Failed: ik_service")
        try:
            resp = self.ik_service(
                LeftRight = left_right,
                limbTwist = cmd,
                targetPos=pos[:3], 
                targetOri=pos[3:7]
            )
            return list(resp.limbTwist)
        except Exception as e:
            print(e)
            return
    
def main():
    robot = Robot()
    # wait for subscriber to get msgs
    while (robot.leftLimb_cmd == None or robot.rightLimb_cmd == None) and not rospy.is_shutdown():
        pass
    
    # Action 1 cmds
    LShoulderPitch=1.78
    LShoulderRoll=-1.4
    LShoulderYaw=0.4
    LElbowRoll=-1.39
    LElbowYaw=0.488
    LWristRoll=0.2 
    LWristPitch=-0.3
    robot.tar_leftLimb_cmd = [LShoulderPitch,
                            LShoulderRoll,
                            LShoulderYaw,
                            LElbowRoll,
                            LElbowYaw,
                            LWristRoll,
                            LWristPitch]
    RShoulderPitch=-1.78
    RShoulderRoll=-1.4
    RShoulderYaw=-0.4
    RElbowRoll=-1.39
    RElbowYaw=-0.448
    RWristRoll=-0.2
    RWristPitch=-0.3
    robot.tar_rightLimb_cmd = [RShoulderPitch,
                               RShoulderRoll,
                               RShoulderYaw,
                               RElbowRoll,
                               RElbowYaw,
                               RWristRoll,
                               RWristPitch]
    time_elapsed = 0
    duration = 1
    rate = rospy.Rate(1000)
    initial_leftLimb_cmd = robot.leftLimb_cmd
    initial_rightLimb_cmd = robot.rightLimb_cmd
    while not rospy.is_shutdown() and time_elapsed < duration:
        # move leftLimb
        robot.step_leftLimb_cmd = [initial_leftLimb_cmd[i] + 
                                   (robot.tar_leftLimb_cmd[i] - initial_leftLimb_cmd[i])
                                   * time_elapsed / duration
                                   for i in range(len(initial_leftLimb_cmd))]
        msg = JointCommand()
        msg.mode = 5
        msg.command = robot.step_leftLimb_cmd
        robot.leftLimbPublisher.publish(msg)
        # Move rightLimb
        robot.step_rightLimb_cmd = [initial_rightLimb_cmd[i] + 
                                    (robot.tar_rightLimb_cmd[i] - initial_rightLimb_cmd[i])
                                    * time_elapsed / duration
                                    for i in range(len(initial_rightLimb_cmd))]
        msg = JointCommand()
        msg.mode = 5
        msg.command = robot.step_rightLimb_cmd
        robot.rightLimbPublisher.publish(msg)
        rate.sleep()
        time_elapsed += 0.001
    rospy.sleep(0.2)
    
    # Action 2 cmds
    LShoulderPitch=1.295
    LShoulderRoll=-0.55
    LShoulderYaw=0.4
    LElbowRoll=-1.39
    LElbowYaw=0.488
    LWristRoll=0.2 
    LWristPitch=-0.3
    robot.tar_leftLimb_cmd = [LShoulderPitch,
                            LShoulderRoll,
                            LShoulderYaw,
                            LElbowRoll,
                            LElbowYaw,
                            LWristRoll,
                            LWristPitch]
    RShoulderPitch=-1.295
    RShoulderRoll=-0.55
    RShoulderYaw=-0.4
    RElbowRoll=-1.39
    RElbowYaw=-0.448
    RWristRoll=-0.2
    RWristPitch=-0.3
    robot.tar_rightLimb_cmd = [RShoulderPitch,
                               RShoulderRoll,
                               RShoulderYaw,
                               RElbowRoll,
                               RElbowYaw,
                               RWristRoll,
                               RWristPitch]
    time_elapsed = 0
    duration = 2
    rate = rospy.Rate(1000)
    initial_leftLimb_cmd = robot.leftLimb_cmd
    initial_rightLimb_cmd = robot.rightLimb_cmd
    while not rospy.is_shutdown() and time_elapsed < duration:
        # move leftLimb
        robot.step_leftLimb_cmd = [initial_leftLimb_cmd[i] + 
                                   (robot.tar_leftLimb_cmd[i] - initial_leftLimb_cmd[i])
                                   * time_elapsed / duration
                                   for i in range(len(initial_leftLimb_cmd))]
        msg = JointCommand()
        msg.mode = 5
        msg.command = robot.step_leftLimb_cmd
        robot.leftLimbPublisher.publish(msg)
        # Move rightLimb
        robot.step_rightLimb_cmd = [initial_rightLimb_cmd[i] + 
                                    (robot.tar_rightLimb_cmd[i] - initial_rightLimb_cmd[i])
                                    * time_elapsed / duration
                                    for i in range(len(initial_rightLimb_cmd))]
        msg = JointCommand()
        msg.mode = 5
        msg.command = robot.step_rightLimb_cmd
        robot.rightLimbPublisher.publish(msg)
        rate.sleep()
        time_elapsed += 0.001
    
    # Action 3 move limb straight forward
    # robot.tar_leftLimb_pos = robot.__cmd2pos__(robot.leftLimb_cmd, "left")
    # robot.tar_leftLimb_pos[0] += 0.11
    # robot.tar_leftLimb_pos[1] += 0.023 #0.023
    # robot.tar_leftLimb_pos[2] += -0.055 #-0.05
    # robot.tar_leftLimb_cmd = robot.__pos2cmd__(robot.tar_leftLimb_pos, robot.leftLimb_cmd, "left")
    # robot.tar_leftLimb_cmd[4] = 2.447
    # robot.tar_leftLimb_cmd[5] = -0.3
    # robot.tar_leftLimb_cmd[6] = -0.3
    
    # robot.tar_rightLimb_pos = robot.__cmd2pos__(robot.rightLimb_cmd, "right")
    # robot.tar_rightLimb_pos[0] += 0.11
    # robot.tar_rightLimb_pos[1] -= 0.023
    # robot.tar_rightLimb_pos[2] += -0.055
    # robot.tar_rightLimb_cmd = robot.__pos2cmd__(robot.tar_rightLimb_pos, robot.rightLimb_cmd, "right")
    # robot.tar_rightLimb_cmd[4] = -2.447
    # robot.tar_rightLimb_cmd[5] = 0.3
    # robot.tar_rightLimb_cmd[6] = -0.3
    
    # time_elapsed = 0
    # duration = 0.5
    # rate = rospy.Rate(1000)
    # initial_leftLimb_cmd = robot.leftLimb_cmd
    # initial_rightLimb_cmd = robot.rightLimb_cmd
    # while not rospy.is_shutdown() and time_elapsed < duration:
    #     # move leftLimb
    #     robot.step_leftLimb_cmd = [initial_leftLimb_cmd[i] + 
    #                                (robot.tar_leftLimb_cmd[i] - initial_leftLimb_cmd[i])
    #                                * time_elapsed / duration
    #                                for i in range(len(initial_leftLimb_cmd))]
    #     msg = JointCommand()
    #     msg.mode = 5
    #     msg.command = robot.step_leftLimb_cmd
    #     robot.leftLimbPublisher.publish(msg)
    #     # Move rightLimb
    #     robot.step_rightLimb_cmd = [initial_rightLimb_cmd[i] + 
    #                                 (robot.tar_rightLimb_cmd[i] - initial_rightLimb_cmd[i])
    #                                 * time_elapsed / duration
    #                                 for i in range(len(initial_rightLimb_cmd))]
    #     msg = JointCommand()
    #     msg.mode = 5
    #     msg.command = robot.step_rightLimb_cmd
    #     robot.rightLimbPublisher.publish(msg)
    #     rate.sleep()
    #     time_elapsed += 0.001
    
    # Action 4 close hand
    robot.tar_leftHand_cmd = [1.5] * 10
    robot.tar_rightHand_cmd = [1.5] * 10
    
    time_elapsed = 0
    duration = 0.5
    rate = rospy.Rate(1000)
    initial_leftHand_cmd = robot.leftHand_cmd
    initial_rightHand_cmd = robot.rightHand_cmd
    while not rospy.is_shutdown() and time_elapsed < duration:
        # move leftLimb
        robot.step_leftHand_cmd = [initial_leftHand_cmd[i] + 
                                   (robot.tar_leftHand_cmd[i] - initial_leftHand_cmd[i])
                                   * time_elapsed / duration
                                   for i in range(len(initial_leftHand_cmd))]
        msg = JointCommand()
        msg.mode = 5
        msg.command = robot.step_leftHand_cmd
        robot.leftHandPublisher.publish(msg)
        # Move rightLimb
        robot.step_rightHand_cmd = [initial_rightHand_cmd[i] + 
                                    (robot.tar_rightHand_cmd[i] - initial_rightHand_cmd[i])
                                    * time_elapsed / duration
                                    for i in range(len(initial_rightHand_cmd))]
        msg = JointCommand()
        msg.mode = 5
        msg.command = robot.step_rightHand_cmd
        robot.rightHandPublisher.publish(msg)
        rate.sleep()
        time_elapsed += 0.001


    # Action 5 start walking
    robot.legmotion_start()
    time_elapsed = 0
    duration = 10
    rate = rospy.Rate(1000)
    initial_leftLimb_cmd = robot.leftLimb_cmd
    initial_rightLimb_cmd = robot.rightLimb_cmd
    initial_leftLimb_pos = robot.__cmd2pos__(robot.leftLimb_cmd, "left")
    initial_rightLimb_pos = robot.__cmd2pos__(robot.rightLimb_cmd, "right")
    robot.tar_leftLimb_pos = robot.__cmd2pos__(robot.leftLimb_cmd, "left")
    robot.tar_rightLimb_pos = robot.__cmd2pos__(robot.rightLimb_cmd, "right")
    mu = 1e-7
    beta = 0.02
    step_num = 14
    theta = pi/4
    
    while not rospy.is_shutdown() and robot.leg_step_num < step_num:
        legmotion_msg = Twist()
        legmotion_msg.linear.x = 0.2
        robot.legmotion_publisher.publish(legmotion_msg)
        
        # Get wrist sensor data 
        lfx = robot.lwrist_sensor.force.x
        lfy = robot.lwrist_sensor.force.y
        lfz = robot.rwrist_sensor.force.z
        # Sensor data constraints
        if lfx > 150: lfx = 150
        if lfx < -150: lfx = -150
        if lfy > 150: lfy = 150
        if lfy < -150: lfy = -150
        if lfz > 150: lfz = 150
        if lfz < -150: lfz = -150
        # fk
        robot.leftLimb_pos = robot.__cmd2pos__(robot.leftLimb_cmd, "left")
        robot.rightLimb_pos = robot.__cmd2pos__(robot.rightLimb_cmd, "right")
        # X axis adjustment
        robot.tar_leftLimb_pos[0] = robot.leftLimb_pos[0] - lfx*mu if lfx < -9 else robot.leftLimb_pos[0]
        # Adjust toward initial pos X
        robot.tar_leftLimb_pos[0] += (initial_leftLimb_pos[0] - robot.tar_leftLimb_pos[0]) * beta
        # Adjust pos y based on 2d force and center of pos y for two limbs 
        robot.tar_leftLimb_pos[1] = robot.leftLimb_pos[1] - (lfz*sin(theta) + lfy*cos(theta))*mu - (robot.leftLimb_pos[1]+robot.rightLimb_pos[1])/2*0.005
        robot.tar_leftLimb_pos[2] = robot.leftLimb_pos[2] - (lfz*cos(theta) + lfy*sin(theta))*mu
        # ik
        robot.step_leftLimb_cmd = robot.__pos2cmd__(robot.tar_leftLimb_pos, robot.leftLimb_cmd, "left")
        if robot.step_leftLimb_cmd == None:
            continue
        # Adjust toward initial cmds
        robot.step_leftLimb_cmd = [robot.step_leftLimb_cmd[i] 
                                   + (initial_leftLimb_cmd[i] - robot.step_leftLimb_cmd[i]) * beta 
                                   for i in range(len(robot.tar_leftLimb_cmd))]
        msg = JointCommand()
        msg.mode = 5
        msg.command = robot.step_leftLimb_cmd
        robot.leftLimbPublisher.publish(msg)
        
        rfx = robot.rwrist_sensor.force.x
        rfy = robot.rwrist_sensor.force.y
        rfz = robot.rwrist_sensor.force.z
        if rfx > 150: rfx = 150
        if rfx < -150: rfx = -150
        if rfy > 150: rfy = 150
        if rfy < -150: rfy = -150
        if rfz > 150: rfz = 150
        if rfz < -150: rfz = -150
        # robot.rightLimb_pos = robot.__cmd2pos__(robot.rightLimb_cmd, "right")
        robot.tar_rightLimb_pos[0] = robot.rightLimb_pos[0] - rfx*mu if rfx < -9 else robot.rightLimb_pos[0]
        robot.tar_rightLimb_pos[0] += (initial_rightLimb_pos[0] - robot.tar_rightLimb_pos[0]) * beta
        robot.tar_rightLimb_pos[1] = robot.rightLimb_pos[1] - (rfz*sin(theta) + rfy*cos(theta))*mu - (robot.leftLimb_pos[1]+robot.rightLimb_pos[1])/2*0.005
        robot.tar_rightLimb_pos[2] = robot.rightLimb_pos[2] - (rfz*cos(theta) + rfy*sin(theta))*mu
        robot.step_rightLimb_cmd = robot.__pos2cmd__(robot.tar_rightLimb_pos, robot.rightLimb_cmd, "right")
        if robot.step_rightLimb_cmd == None:
            continue
        robot.step_rightLimb_cmd = [robot.step_rightLimb_cmd[i] 
                                   + (initial_rightLimb_cmd[i] - robot.step_rightLimb_cmd[i]) * beta 
                                   for i in range(len(robot.tar_rightLimb_cmd))]
        msg = JointCommand()
        msg.mode = 5
        msg.command = robot.step_rightLimb_cmd
        robot.rightLimbPublisher.publish(msg)
        
        rate.sleep()
        time_elapsed += 0.001
    
    robot.legmotion_stop()

    duration = 1
    time_elapsed = 0
    # Adjust limb for smooth stop
    while time_elapsed < duration:
        legmotion_msg = Twist()
        legmotion_msg.linear.x = 0.2
        robot.legmotion_publisher.publish(legmotion_msg)
        
        # Get wrist sensor data 
        lfx = robot.lwrist_sensor.force.x
        lfy = robot.lwrist_sensor.force.y
        lfz = robot.rwrist_sensor.force.z
        # Sensor data constraints
        if lfx > 150: lfx = 150
        if lfx < -150: lfx = -150
        if lfy > 150: lfy = 150
        if lfy < -150: lfy = -150
        if lfz > 150: lfz = 150
        if lfz < -150: lfz = -150
        # fk
        robot.leftLimb_pos = robot.__cmd2pos__(robot.leftLimb_cmd, "left")
        robot.rightLimb_pos = robot.__cmd2pos__(robot.rightLimb_cmd, "right")
        # X axis adjustment
        robot.tar_leftLimb_pos[0] = robot.leftLimb_pos[0] - lfx*mu if lfx < -9 else robot.leftLimb_pos[0]
        # Adjust toward initial pos X
        robot.tar_leftLimb_pos[0] += (initial_leftLimb_pos[0] - robot.tar_leftLimb_pos[0]) * beta
        # Adjust pos y based on 2d force and center of pos y for two limbs 
        robot.tar_leftLimb_pos[1] = robot.leftLimb_pos[1] - (lfz*sin(theta) + lfy*cos(theta))*mu - (robot.leftLimb_pos[1]+robot.rightLimb_pos[1])/2*0.005
        robot.tar_leftLimb_pos[2] = robot.leftLimb_pos[2] - (lfz*cos(theta)+ lfy*sin(theta))*mu
        # ik
        robot.step_leftLimb_cmd = robot.__pos2cmd__(robot.tar_leftLimb_pos, robot.leftLimb_cmd, "left")
        # Adjust toward initial cmds
        robot.step_leftLimb_cmd = [robot.step_leftLimb_cmd[i] 
                                   + (initial_leftLimb_cmd[i] - robot.step_leftLimb_cmd[i]) * beta 
                                   for i in range(len(robot.tar_leftLimb_cmd))]
        msg = JointCommand()
        msg.mode = 5
        msg.command = robot.step_leftLimb_cmd
        robot.leftLimbPublisher.publish(msg)
        
        rfx = robot.rwrist_sensor.force.x
        rfy = robot.rwrist_sensor.force.y
        rfz = robot.rwrist_sensor.force.z
        if rfx > 150: rfx = 150
        if rfx < -150: rfx = -150
        if rfy > 150: rfy = 150
        if rfy < -150: rfy = -150
        if rfz > 150: rfz = 150
        if rfz < -150: rfz = -150
        robot.tar_rightLimb_pos[0] = robot.rightLimb_pos[0] - rfx*mu if rfx < -9 else robot.rightLimb_pos[0]
        robot.tar_rightLimb_pos[0] += (initial_rightLimb_pos[0] - robot.tar_rightLimb_pos[0]) * beta
        robot.tar_rightLimb_pos[1] = robot.rightLimb_pos[1] - (rfz*sin(theta) + rfy*cos(theta))*mu - (robot.leftLimb_pos[1]+robot.rightLimb_pos[1])/2*0.005
        robot.tar_rightLimb_pos[2] = robot.rightLimb_pos[2] - (rfz*cos(theta) + rfy*sin(theta))*mu
        robot.step_rightLimb_cmd = robot.__pos2cmd__(robot.tar_rightLimb_pos, robot.rightLimb_cmd, "right")
        robot.step_rightLimb_cmd = [robot.step_rightLimb_cmd[i] 
                                   + (initial_rightLimb_cmd[i] - robot.step_rightLimb_cmd[i]) * beta 
                                   for i in range(len(robot.tar_rightLimb_cmd))]
        msg = JointCommand()
        msg.mode = 5
        msg.command = robot.step_rightLimb_cmd
        robot.rightLimbPublisher.publish(msg)
        
        rate.sleep()
        time_elapsed += 0.001

    # Shutdown callback
    rospy.on_shutdown(robot.legmotion_stop)
    
    

if __name__ == "__main__":# Load scene
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
        
    rospy.init_node("task6")
    main()