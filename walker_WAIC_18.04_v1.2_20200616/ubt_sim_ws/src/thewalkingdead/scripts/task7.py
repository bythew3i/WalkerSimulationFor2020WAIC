#! /usr/bin/env python

import rospy
from webots_api.srv import SceneSelection, SceneSelectionRequest
from walker_srvs.srv import leg_motion_MetaFuncCtrl, leg_motion_MetaFuncCtrlRequest
from std_msgs.msg import Int64,String
from geometry_msgs.msg import Twist, WrenchStamped
from ubt_core_msgs.msg import JointCommand
from sensor_msgs.msg import JointState, Image
from thewalkingdead.srv import Solver
from cv_bridge import CvBridge
import cv2
import time

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
        self.head_depth = None

                 
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
            self.leftLimbCallback
            )
        rospy.Subscriber(
            "/walker/rightLimb/joint_states",
            JointState,
            self.rightLimbCallback
            )
        rospy.Subscriber(
            "/walker/leftHand/joint_states",
            JointState,
            self.leftHandCallback
        )
        rospy.Subscriber(
            "/walker/rightHand/joint_states",
            JointState,
            self.rightHandCallback
        )
        rospy.Subscriber(
            "/sensor/ft/lwrist",
            WrenchStamped,
            self.lwristSensorCallback
        )
        rospy.Subscriber(
            "/sensor/ft/rwrist",
            WrenchStamped,
            self.rwristSensorCallback
        )
        rospy.Subscriber(
            "/Leg/StepNum",
            Int64,
            self.legStepNumCallback
        )
        rospy.Subscriber(
            "/Leg/leg_status",
            String,
            self.legStatusCallback
        )
        rospy.Subscriber(
            "/walker/camera/headDepth",
            Image,
            self.headDepthCallback)
        
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
    def leftLimbCallback(self, msg):
        self.leftLimb_cmd = msg.position
        
    def rightLimbCallback(self, msg):
        self.rightLimb_cmd = msg.position
    
    def leftHandCallback(self,msg):
        self.leftHand_cmd = msg.position
    
    def rightHandCallback(self, msg):
        self.rightHand_cmd = msg.position
    
    def lwristSensorCallback(self, msg):
        self.lwrist_sensor = msg.wrench
    
    def rwristSensorCallback(self, msg):
        self.rwrist_sensor = msg.wrench
    
    def legStepNumCallback(self,msg):
        self.leg_step_num = msg.data
        
    def legStatusCallback(self,msg):
        self.leg_status = msg.data
        
    def headDepthCallback(self,msg):
        self.head_depth = msg
    
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
    
    def measure_depth(self):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(self.head_depth,
                                        desired_encoding="passthrough")
        return cv_image
    
    # @staticmethod
    def __pos2cmd__(self, pos, cmd, left_right):
        if self.ik_service==None:
            raise Exception("Failed: ik_service")
        resp = self.ik_service(
            LeftRight = left_right,
            limbTwist = cmd,
            targetPos=pos[:3], 
            targetOri=pos[3:7]
        )
        return list(resp.limbTwist)
    
def main():
    # Load scene
    rospy.wait_for_service("/walker/sence")
    try:
        scheduler = rospy.ServiceProxy("/walker/sence", SceneSelection)
        request = SceneSelectionRequest()
        request.scene_name = "PushCart"
        request.nav = False
        request.vision = True
        response = scheduler(request)
        print(response)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        
    rospy.init_node("task7")
    robot = Robot()
    # wait for subscriber to get msgs
    while (robot.leftLimb_cmd == None or robot.rightLimb_cmd == None or robot.head_depth == None) and not rospy.is_shutdown():
        pass
    rospy.loginfo(robot.leg_status)
    
    rospy.loginfo(robot.head_depth.height)
    rospy.loginfo(robot.head_depth.width)
    rospy.loginfo(robot.head_depth.encoding)
    rospy.loginfo(robot.measure_depth().dtype)
    rospy.loginfo(type(robot.measure_depth()))
    
    
    
    
    

if __name__ == "__main__":
    main()