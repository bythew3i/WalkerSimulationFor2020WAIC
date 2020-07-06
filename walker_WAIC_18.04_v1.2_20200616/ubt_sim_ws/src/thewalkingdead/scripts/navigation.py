import rospy
import tf
import dynamic_reconfigure.client
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int64, String

import functools
import threading
import math

PI = 3.141592654


def _dist(x, y):
    return math.sqrt(sum(map(lambda (a, b): (a-b)*(a-b), zip(x, y))))


def _dot(u, v):
    return sum([a*b for a, b in zip(u, v)])


def _todeg(rad):
    return rad * 57.2957795


def _torad(deg):
    return deg / 180.0 * PI


def _origin_symmetric_angle(rad):
    p = [-math.cos(rad), -math.sin(rad)]
    return math.atan2(p[1], p[0])


class Navigation(object):
    def __init__(self):
        # self.dcfg_client = dynamic_reconfigure.client.Client("orb_slam2_stereo",
        #     timeout=10, config_callback=self._configure_orb_slam_callback)

        self._lock = threading.Lock()
        self._lock_step = threading.Lock()
        self._lock_stat = threading.Lock()

        # TODO: rewrite these variables with @property decorator
        # and getter, setters, encapsulate locking mechanisms.
        # https://www.python-course.eu/python3_properties.php
        self.position = None
        self.yaw = None
        self.total_step = None

        self.odom_listner = rospy.Subscriber(
            "/odom", Odometry, self._odom_callback, queue_size=10)
        self.step_listener = rospy.Subscriber(
            "/Leg/StepNum", Int64, self._step_callback, queue_size=10)
        self.nav_pub = rospy.Publisher(
            "/nav/cmd_vel_nav", Twist, queue_size=10, latch=True)

        # Wait for incoming data
        self._wait_for_step_num()

    def _configure_orb_slam_callback(self, config):
        rospy.loginfo("Navigation::Current orbslam2 config: %s", str(config))

    def _odom_callback(self, d):
        with self._lock:
            self.position = [d.pose.pose.position.x, d.pose.pose.position.y]
            self._rot = d.pose.pose.orientation
            self._rot_e = tf.transformations.euler_from_quaternion([self._rot.x,
                                                                    self._rot.y,
                                                                    self._rot.z,
                                                                    self._rot.w])
            self.yaw = self._rot_e[2]

    def _step_callback(self, d):
        with self._lock_step:
            self.total_step = int(d.data)

    def _wait_for_odom_data(self):
        while self.position is None or self.yaw is None:
            pass

    def _wait_for_step_num(self):
        while self.total_step is None:
            pass

    def _nav_msg(self, lx=0.0, ly=0.0, lz=0.0, ax=0.0, ay=0.0, az=0.0):
        t = Twist()
        t.linear.x = lx
        t.linear.y = ly
        t.linear.z = lz
        t.angular.x = ax
        t.angular.y = ay
        t.angular.z = az
        return t

    def _compute_steps_angle(self, step_lengths, angle):
        step_list = []
        for l in step_lengths:
            s = math.floor(angle / l)
            angle = angle - s * l
            step_list.append(s * 2)  # When turning, 2 steps make one turn
        return step_list

    def _compute_steps_linear(self, step_lengths, x):
        step_list = []
        for l in step_lengths:
            s = math.floor(x / l)
            x = x - s * l
            step_list.append(s)
        return step_list

    # def _compute_target_yaw_dist(self, cur_pos, tar_pos):
    #     d = [b-a for a, b in zip(cur_pos, tar_pos)]
    #     norm = math.sqrt(functools.reduce(lambda a,b : a+b, [x*x for x in d]))
    #     d = [x / norm for x in d]
    #     yaw = math.acos(d[0]) if d[1] > 0 else -math.acos(d[0])
    #     return yaw, norm

    # If cur_yaw is not provided, it will only go to tar_pos with positive x.
    def _compute_target_yaw_dist(self, cur_pos, tar_pos, cur_yaw=None):

        d = [b-a for a, b in zip(cur_pos, tar_pos)]
        tnorm = norm = math.sqrt(sum([x*x for x in d]))
        d = [x / norm for x in d]
        tyaw = yaw = math.acos(d[0]) if d[1] > 0 else -math.acos(d[0])

        if cur_yaw is not None:
            e = [math.cos(cur_yaw), math.sin(cur_yaw)]
            a = math.acos(_dot(d, e))
            if a < PI / 2:
                tyaw = yaw
                tnorm = norm
            else:
                tyaw = _origin_symmetric_angle(yaw)
                tnorm = -norm

        return tyaw, tnorm

    def _wait_for_step(self):
        with self._lock_step:
            cur_step = start_step = self.total_step
        rospy.logdebug("Waiting for step...")
        while start_step == cur_step:
            with self._lock_step:
                cur_step = self.total_step

    def compute_min_turn_angle_dir(self, yaw, t_yaw):
        angle = (t_yaw - yaw + 2 * PI) % (2*PI)
        sign = 1  # ccw
        if angle > PI:
            angle = 2*PI - angle
            sign = -1  # cw
        return angle, sign

    def halt(self, t=1.0):
        r = rospy.Rate(10)
        s = 0.0
        while s < t:
            self.nav_pub.publish(self._nav_msg())
            s += 0.1
            r.sleep()

    def turn_for(self, angle):
        sign = 1 if angle > 0 else -1
        angle = abs(angle)
        step_lengths = [0.35, 0.175, 0.0875,
                        0.04375, 0.021875, 0.0109375, 0.00546875]
        steps = self._compute_steps_angle(step_lengths, abs(angle))
        rospy.loginfo("Angle: %f, direction: %s, Steps to turn: %s", angle,
                        'CCW' if sign > 0 else 'CW', str(steps))

        self._wait_for_step()
        with self._lock_step:
            cur_step = self.total_step

        r = rospy.Rate(10)
        for i in range(len(steps)):
            tar_step = cur_step + steps[i]
            while cur_step < tar_step:
                self.nav_pub.publish(self._nav_msg(az=step_lengths[i]*sign))
                with self._lock_step:
                    cur_step = self.total_step
                r.sleep()

    # TODO: rewrite turn_to using turn_for
    def turn_to(self, t_yaw):
        with self._lock:
            yaw = self.yaw

        # angle = (t_yaw - yaw + 2 * PI) % (2*PI)
        # sign = 1  # ccw
        # if angle > PI:
        #     angle = 2*PI - angle
        #     sign = -1  # cw
        angle, sign = self.compute_min_turn_angle_dir(yaw, t_yaw)

        step_lengths = [0.35, 0.175, 0.0875,
                        0.04375, 0.021875, 0.0109375, 0.00546875]
        steps = self._compute_steps_angle(step_lengths, angle)
        rospy.loginfo("Angle: %f, direction: %s, Steps to turn: %s", angle,
                      'CCW' if sign > 0 else 'CW', str(steps))

        self._wait_for_step()
        with self._lock_step:
            cur_step = self.total_step

        r = rospy.Rate(10)
        for i in range(len(steps)):
            tar_step = cur_step + steps[i]
            while cur_step < tar_step:
                self.nav_pub.publish(self._nav_msg(az=step_lengths[i]*sign))
                with self._lock_step:
                    cur_step = self.total_step
                r.sleep()

    def move_for(self, t_x):
        step_lengths = [0.32, 0.16, 0.08, 0.04, 0.02, 0.01, 0.005]
        step_list = self._compute_steps_linear(step_lengths, abs(t_x))
        sign = 1 if t_x > 0 else -1
        rospy.loginfo("t_x: %f, Steps to walk: %s, Direction: %s",
                      t_x, str(step_list), 'forward' if sign > 0 else 'backward')

        self._wait_for_step()
        with self._lock_step:
            step = cur_step = self.total_step

        r = rospy.Rate(10)
        for i in range(len(step_list)):
            tar_step = cur_step + step_list[i]
            while cur_step < tar_step:
                self.nav_pub.publish(self._nav_msg(lx=step_lengths[i]*sign))
                with self._lock_step:
                    cur_step = self.total_step
                r.sleep()

    def move_sideway_for(self, t):
        step_lengths = [0.04, 0.02, 0.01, 0.005]
        step_list = self._compute_steps_linear(step_lengths, abs(t))
        step_list = [s*2 for s in step_list] #2 steps for sideway, just like turning.
        sign = 1 if t > 0 else -1
        rospy.loginfo("t: %f, Steps to walk: %s, Direction: %s",
                      t, str(step_list), 'left' if sign > 0 else 'right')

        self._wait_for_step()
        with self._lock_step:
            step = cur_step = self.total_step

        r = rospy.Rate(10)
        for i in range(len(step_list)):
            tar_step = cur_step + step_list[i]
            while cur_step < tar_step:
                self.nav_pub.publish(self._nav_msg(ly=step_lengths[i]*sign))
                with self._lock_step:
                    cur_step = self.total_step
                r.sleep()

    def correct_x(self, tar_x):
        with self._lock:
            cur_x, _ = self.position
        d = cur_x - tar_x
        self.move_sideway_for(d)

    def go_to(self, tar_pos, head_on=False):
        with self._lock:
            cur_pos = self.position
            cur_yaw = self.yaw
        rospy.logdebug("Current position: %s, Target position: %s",
                       str(cur_pos), str(tar_pos))
        if not head_on:
            tar_yaw, tar_dist = self._compute_target_yaw_dist(
                cur_pos, tar_pos, cur_yaw)
        else:
            tar_yaw, tar_dist = self._compute_target_yaw_dist(cur_pos, tar_pos)

        rospy.logdebug("Target yaw: %s, target distance: %s",
                       str(tar_yaw), str(tar_dist))

        rospy.logdebug("GOTO::Starting turn_to...")
        self.turn_to(tar_yaw)
        rospy.logdebug("GOTO::Starting move for...")
        self.move_for(tar_dist)

    def wait_for_orb_slam(self):
        rospy.wait_for_message(
            "/orb_slam2_stereo/map_points", PointCloud2, None)

    def configure_orb_slam_localization(self):
        self.dcfg_client.update_configuration({"localize_only": True})

    def relocalize(self):
        cur_position = last_position = [0.0, 0.0]
        dist = 0.0

        r = rospy.Rate(10)
        while dist < 5.0:
            last_position = cur_position
            with self._lock:
                cur_position = self.position if self.position is not None else cur_position
            dist = _dist(last_position, cur_position)
            try:
                self.nav_pub.publish(self._nav_msg(az=-0.35))
            except rospy.ServiceException as e:
                rospy.logerror(
                    "Navigation::Failure in sending message to /nav/cmd_vel_nav")
                # raise e # try keep sending.

            r.sleep()

        rospy.loginfo(
            "Navigation::Detected large position shift, assuming relocalization happened...")
        # Once relocation finishes, orb_slam2 will start sending /map->/camera_link tf
        # which will be picked up by odom_publisher
        self._wait_for_odom_data()
