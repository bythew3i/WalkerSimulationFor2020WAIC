#!/usr/bin/env python

from math import sqrt, e, asin, acos, sin, cos, atan, pi
from scipy.interpolate import KroghInterpolator
import matplotlib.pyplot as plt 

# CONSTANTS
L1 = 0.275
L2 = 0.290
H = 0.1404
G = 9.8
Zch = 0.098
W = 0.220


# Adjustable variables
S = 0.1
T = 0.8
D = 0.22
Li = 0.52
Hm = 0.08
CARRY_TIME = 1.6
INIT_TIME = 2.0
STEPS = 15



# Calculate some useful variavles
L = sqrt(L2**2 - D**2) + Zch + L1
assert L < Zch + L1 + L2

Td = T/10
Tm = (T+Td)/2

CARRY_KNEE = acos((Li-Zch-L1)/L2)
CARRY_HIP = - CARRY_KNEE / 2
CARRY_ANKLE = CARRY_HIP

INIT_KNEE = acos((L-Zch-L1)/L2)
INIT_HIP = - INIT_KNEE
INIT_ANKLE = 0


def derivative(f, x):
    diff = 1e-8
    return (f(x+diff) - f(x))/diff


##########################################
# Forward and Back motion planning
##########################################
theta_0 = asin(S/2/L)
theta_T = asin(-S/2/L)

k = sqrt(G/L)
e1 = e**(-k*T)
e2 = e**(k*T)
c1 = (theta_0*e2 - theta_T) / (e2-e1)
c2 = (theta_T - theta_0*e1) / (e2-e1)

### Get CM trajectory on x, z
def f_theta(t):
    return c1*e**(-k*t)+c2*e**(k*t)

def f_x_cm(t):
    return L*sin(f_theta(t))

def f_z_cm(t):
    return L*cos(f_theta(t))


### Get Ankle trajectory on x, z
f_x_ank1 = KroghInterpolator(
    [Td, Td, Td, T, T, T],
    [S, 0, 0, -S, 0, 0]
)

def f_x_ank(t):
    if t < Td:
        return S
    return f_x_ank1(t)

f_z_ank1 = KroghInterpolator(
    [Td, Td, Td, Tm, Tm, Tm],
    [0, 0, 0, Hm, 0, 0]
)

f_z_ank2 = KroghInterpolator(
    [Tm, Tm, Tm, T, T, T],
    [Hm, 0, 0, 0, 0, 0]
)

def f_z_ank(t):
    if t < Td:
        return 0
    elif t < Tm:
        return f_z_ank1(t)
    return f_z_ank2(t)


### Get all the angles trajectory on x, z of Supporting Leg
def s_z(t):
    return f_z_cm(t) - Zch

def s_x(t):
    return f_x_cm(t) - D

def s_l0(t):
    return sqrt(s_z(t)**2 + s_x(t)**2)

def s_alpha(t):
    return atan(s_x(t) / s_z(t))

def f_s_ankle(t):
    L0 = s_l0(t)
    return - (s_alpha(t) + acos((L1**2+L0**2 - L2**2)/(2*L1*L0)))

def f_s_knee(t):
    L0 = s_l0(t)
    return pi - acos((L1**2+L2**2-L0**2)/(2*L1*L2))

def f_s_hip(t):
    L0 = s_l0(t)
    return - (acos((L2**2+L0**2-L1**2)/(2*L2*L0))-s_alpha(t))


### Get all the angles trajectory on x, z of Moving Leg
def m_z(t):
    return f_z_cm(t) - Zch - f_z_ank(t)

def m_x(t):
    return f_x_ank(t) - (f_x_cm(t) - D)

def m_l0(t):
    return sqrt(m_z(t)**2 + m_x(t)**2)

def m_alpha(t):
    return atan(m_x(t) / m_z(t))

def f_m_ankle(t):
    L0 = m_l0(t)
    return m_alpha(t) - acos((L1**2+L0**2-L2**2)/(2*L1*L0))

def f_m_knee(t):
    L0 = m_l0(t)
    return pi - acos((L1**2+L2**2-L0**2)/(2*L1*L2))

def f_m_hip(t):
    L0 = m_l0(t)
    return - (acos((L2**2+L0**2-L1**2)/(2*L2*L0))+m_alpha(t))




##########################################
# Left and Right motion planing
##########################################
DELTA = asin(W/2/L)
k1 = k*e**(-k*T/2)
k2 = k*e**(k*T/2)
d1 = k2*DELTA/(k1+k2)
d2 = k1*DELTA/(k1+k2)

def f_delta_cm(t):
    if t < T/2:
        return d1*e**(-k*t) + d2*e**(k*t)
    elif t < T:
        return d1*e**(-k*(T-t)) + d2*e**(k*(T-t))
    elif t < 3*T/2:
        return d1*e**(-k*(t-T)) + d2*e**(k*(t-T))
    return d1*e**(-k*(2*T-t)) + d2*e**(k*(2*T-t))

def f_delta(t):
    y_ce = W/2 - L*sin(f_delta_cm(t))
    z_ce = L*cos(f_delta_cm(t)) - Zch
    if t < T:
        return atan(y_ce/z_ce) 
    return -atan(y_ce/z_ce) 




##########################################
# Start motion planning
##########################################
gamma_0 = 0
gamma_T =  asin(-S/2/L)

b1 = (gamma_0*e2 - gamma_T) / (e2-e1)
b2 = (gamma_T - gamma_0*e1) / (e2-e1)

### Get CM trajectory on x, z
def f_theta_start(t):
    return b1*e**(-k*t)+b2*e**(k*t)

def f_x_cm_start(t):
    return L*sin(f_theta_start(t))

def f_z_cm_start(t):
    return L*cos(f_theta_start(t))


### Get Ankle trajectory on x, z
f_x_ank_start = KroghInterpolator(
    [0, 0, 0, T, T, T],
    [0, 0, 0, -S, 0, 0]
)

f_z_ank1_start = KroghInterpolator(
    [0, 0, 0, T/2, T/2, T/2,],
    [0, 0, 0, Hm/2, 0, 0,]
)

f_z_ank2_start = KroghInterpolator(
    [T/2, T/2, T/2, T, T, T],
    [Hm/2, 0, 0, 0, 0, 0]
)
def f_z_ank_start(t):
    if t < T/2:
        return f_z_ank1_start(t)
    return f_z_ank2_start(t)



### Get all the angles trajectory on x, z of Supporting Leg
def s_z_start(t):
    return f_z_cm_start(t) - Zch

def s_x_start(t):
    return f_x_cm_start(t) - D

def s_l0_start(t):
    return sqrt(s_z_start(t)**2 + s_x_start(t)**2)

def s_alpha_start(t):
    return atan(s_x_start(t) / s_z_start(t))

def f_s_ankle_start(t):
    L0 = s_l0_start(t)
    return - (s_alpha_start(t) + acos((L1**2+L0**2 - L2**2)/(2*L1*L0)))

def f_s_knee_start(t):
    L0 = s_l0_start(t)
    return pi - acos((L1**2+L2**2-L0**2)/(2*L1*L2))

def f_s_hip_start(t):
    L0 = s_l0_start(t)
    return - (acos((L2**2+L0**2-L1**2)/(2*L2*L0))-s_alpha_start(t))

### Get all the angles trajectory on x, z of Moving Leg
def m_z_start(t):
    return f_z_cm_start(t) - Zch - f_z_ank_start(t)

def m_x_start(t):
    return f_x_ank_start(t) - (f_x_cm_start(t) - D)

def m_l0_start(t):
    return sqrt(m_z_start(t)**2 + m_x_start(t)**2)

def m_alpha_start(t):
    return atan(m_x_start(t) / m_z_start(t))

def f_m_ankle_start(t):
    L0 = m_l0_start(t)
    return m_alpha_start(t) - acos((L1**2+L0**2-L2**2)/(2*L1*L0))

def f_m_knee_start(t):
    L0 = m_l0_start(t)
    return pi - acos((L1**2+L2**2-L0**2)/(2*L1*L2))

def f_m_hip_start(t):
    L0 = m_l0_start(t)
    return - (acos((L2**2+L0**2-L1**2)/(2*L2*L0))+m_alpha_start(t))



f_delta_start = KroghInterpolator(
    [0, 0, T, T],
    [-f_delta(T/2), 0, f_delta(0), derivative(f_delta, 0)]
)







##########################################
# Init motion planning
##########################################
f_ankle_init = KroghInterpolator(
    [0, 0, 0, INIT_TIME, INIT_TIME, INIT_TIME],
    [CARRY_ANKLE, 0, 0, INIT_ANKLE, 0, 0]
)

f_knee_init = KroghInterpolator(
    [0, 0, 0, INIT_TIME, INIT_TIME, INIT_TIME],
    [CARRY_KNEE, 0, 0, INIT_KNEE, 0, 0]
)

f_hip_init = KroghInterpolator(
    [0, 0, 0, INIT_TIME, INIT_TIME, INIT_TIME],
    [CARRY_HIP, 0, 0, INIT_HIP, 0, 0]
)

f_delta_init = KroghInterpolator(
    [0, 0, 0, INIT_TIME, INIT_TIME, INIT_TIME],
    [0, 0, 0, -f_delta(T/2), 0, 0]
)











##########################################
# Visualize the trajectory
##########################################
def draw_trajectory(X, Y, xa, ya, title):
    plt.figure()
    plt.plot(X, Y)
    plt.title(title)
    plt.xlabel(xa)
    plt.ylabel(ya)

def get_xy(start, end, f):
    X = []
    Y = []
    t = start
    while t < end:
        X.append(t)
        Y.append(f(t-start))
        t += 0.01
    return X, Y

def debugT(trac1, trac2, tt):
    print("{}(0)=={}(T): {} == {}  {}".format(
        trac1.__name__,
        trac2.__name__,
        trac1(0), 
        trac2(tt), 
        abs(abs(trac1(0))-abs(trac2(tt)))
    ))
    print("{}(0)=={}(T): {} == {}  {}".format(
        trac2.__name__,
        trac1.__name__,
        trac2(0), 
        trac1(tt), 
        abs(abs(trac1(0))-abs(trac2(tt)))
    ))





# X, Y = get_xy(0, T, f_z_ank_start)
# draw_trajectory(
#     X=X,
#     Y=Y,
#     xa="Time (s)",
#     ya="Angle (rad)",
#     title="f_z_ank_start Trajectory (T={}s)".format(T)
# )



# debugT(f_s_ankle, f_m_ankle, T)
# debugT(f_s_knee, f_m_knee, T)
# debugT(f_s_hip, f_m_hip, T)

# plt.show()













#################################################################################
#################################################################################
# ROS code
#################################################################################
#################################################################################
import rospy
from sensor_msgs.msg import JointState
from webots_api.srv import SceneSelection
from ubt_core_msgs.msg import JointCommand
from geometry_msgs.msg import Twist, WrenchStamped
from thewalkingdead.srv import Solver


class Logger():
    def __init__(self):
        self.msg = None 
    def log(self, msg):
        if self.msg != msg:
            self.msg = msg
            print(self.msg)

def log_msg(pmsg, msg):
    if pmsg != msg:
        print(msg)

def robot_update_all_delta(pos, delta):
    pos[1] = delta
    pos[7] = delta
    pos[5] = -delta
    pos[11] = -delta

def robot_update_l_leg(pos, ankle, knee, hip):
    pos[4] =ankle
    pos[3] = knee
    pos[2] = hip

def robot_update_r_leg(pos, ankle, knee, hip):
    pos[10] = ankle
    pos[9] = knee
    pos[8] = hip

def trans(start, end, fac):
    return start + (end - start)*fac


rospy.init_node('task14', anonymous=True)

## service
print("\nWaiting for /walker/sence service ...")
rospy.wait_for_service('/walker/sence')
scene_service = rospy.ServiceProxy(
    "/walker/sence", 
    SceneSelection
)

## publisher
leg_pub = rospy.Publisher('/Leg/DesiredJoint', JointState, queue_size=10)
r_limb_pub = rospy.Publisher('/walker/rightLimb/controller', JointCommand, queue_size=10)
l_limb_pub = rospy.Publisher('/walker/leftLimb/controller', JointCommand, queue_size=10)
r_hand_pub = rospy.Publisher('/walker/rightHand/controller', JointCommand, queue_size=10)
l_hand_pub = rospy.Publisher('/walker/leftHand/controller', JointCommand, queue_size=10)

## subscriber




## messages
leg_msg = JointState()
leg_msg.position = [0 for _ in range(12)]
r_limb_msg = JointCommand()
r_limb_msg.command = [0 for _ in range(7)]
l_limb_msg = JointCommand()
l_limb_msg.command = [0 for _ in range(7)]
r_hand_msg = JointCommand()
r_hand_msg.command = [0 for _ in range(10)]
l_hand_msg = JointCommand()
l_hand_msg.command = [0 for _ in range(10)]



## Select the scene
scene_service(scene_name="CarryBox", nav=False, vision=False)

rate = rospy.Rate(1000)
timer = 0

action = 0
logger = Logger()
step_cnt = 0

while not rospy.is_shutdown():
    timer += 0.001
    if action==0:
        logger.log("Action 0: Carrying the box")

        unit_time = CARRY_TIME/4

        if timer <= unit_time*1:
            fac = (timer + unit_time - unit_time*1)/ unit_time

            ankle = trans(0, CARRY_ANKLE, fac)
            knee = trans(0, CARRY_KNEE, fac)
            hip = trans(0, CARRY_HIP, fac)

            l_limb_msg.command[1] = trans(0, -pi/2, fac)
            l_limb_msg.command[3] = trans(0, -pi/2, fac)

            r_limb_msg.command[1] = trans(0, -pi/2, fac)
            r_limb_msg.command[3] = trans(0, -pi/2, fac)

            # Update
            l_limb_pub.publish(l_limb_msg)
            r_limb_pub.publish(r_limb_msg)
            robot_update_l_leg(leg_msg.position, ankle, knee, hip)
            robot_update_r_leg(leg_msg.position, ankle, knee, hip)
            leg_pub.publish(leg_msg)
        
        elif timer <= unit_time*2:
            fac = (timer + unit_time - unit_time*2)/ unit_time
            l_limb_msg.command[0] = trans(0, pi/2, fac)
            l_limb_msg.command[2] = trans(0, pi/2, fac)

            r_limb_msg.command[0] = trans(0, -pi/2, fac)
            r_limb_msg.command[2] = trans(0, -pi/2, fac)

            l_limb_pub.publish(l_limb_msg)
            r_limb_pub.publish(r_limb_msg)

        elif timer <= unit_time*3:
            fac = (timer + unit_time - unit_time*3)/ unit_time
            

            l_limb_msg.command[0] = trans(pi/2, 1.2, fac)
            l_limb_msg.command[1] = trans(-pi/2, -0.7, fac)
            l_limb_msg.command[2] = trans(pi/2, 0.12, fac)
            l_limb_msg.command[4] = trans(0, -1.3, fac)
            l_limb_msg.command[5] = trans(0, -0.6, fac)
            l_limb_msg.command[6] = trans(0, 0.54, fac)

            r_limb_msg.command[0] = trans(-pi/2, -1.2, fac)
            r_limb_msg.command[1] = trans(-pi/2, -0.7, fac)
            r_limb_msg.command[2] = trans(-pi/2, -0.12, fac)
            r_limb_msg.command[4] = trans(0, 1.3, fac)
            r_limb_msg.command[5] = trans(0, 0.6, fac)
            r_limb_msg.command[6] = trans(0, 0.54, fac)

            l_limb_pub.publish(l_limb_msg)
            r_limb_pub.publish(r_limb_msg)

        elif timer <= unit_time*4:
            fac = (timer + unit_time - unit_time*4)/ unit_time

            for idx in range(10):
                l_hand_msg.command[idx] = trans(0, pi/2, fac)
                r_hand_msg.command[idx] = trans(0, pi/2, fac)

            l_hand_pub.publish(l_hand_msg)
            r_hand_pub.publish(r_hand_msg)
        else:
            action += 1
            timer = 0

    elif action==1:
        logger.log("Action 1: Balancing the pos")
        if timer < INIT_TIME:
            delta =f_delta_init(timer)
            ankle = f_ankle_init(timer)
            knee = f_knee_init(timer)
            hip = f_hip_init(timer)

            robot_update_all_delta(leg_msg.position, delta)
            robot_update_l_leg(leg_msg.position, ankle, knee, hip)
            robot_update_r_leg(leg_msg.position, ankle, knee, hip)
            leg_pub.publish(leg_msg)
        else:
            action+= 1
            timer = 0
    elif action == 2:
        logger.log("Action 2: Start walking ...")
        if timer < T:
            delta =f_delta_start(timer)
            # left
            l_ankle = f_s_ankle_start(timer)
            l_knee = f_s_knee_start(timer)
            l_hip = f_s_hip_start(timer)

            #right
            r_ankle = f_m_ankle_start(timer)
            r_knee = f_m_knee_start(timer)
            r_hip = f_m_hip_start(timer)

            # Update
            robot_update_all_delta(leg_msg.position, delta)
            robot_update_l_leg(leg_msg.position, l_ankle, l_knee, l_hip)
            robot_update_r_leg(leg_msg.position, r_ankle, r_knee, r_hip)
            leg_pub.publish(leg_msg)
        else:
            action += 1
            timer = 0
    elif action==3:
        logger.log("Action 3: Walk the walk")
        if timer < T:
            delta = f_delta((step_cnt%2)*T+timer)

            # left
            l_ankle = f_m_ankle(timer) if step_cnt%2 == 0 else f_s_ankle(timer)
            l_knee = f_m_knee(timer) if step_cnt%2 == 0 else f_s_knee(timer)
            l_hip = f_m_hip(timer) if step_cnt%2 == 0 else f_s_hip(timer)
            #right
            r_ankle = f_s_ankle(timer) if step_cnt%2 == 0 else f_m_ankle(timer)
            r_knee = f_s_knee(timer) if step_cnt%2 == 0 else f_m_knee(timer)
            r_hip = f_s_hip(timer) if step_cnt%2 == 0 else f_m_hip(timer)

            # Update
            robot_update_all_delta(leg_msg.position, delta)
            robot_update_l_leg(leg_msg.position, l_ankle, l_knee, l_hip)
            robot_update_r_leg(leg_msg.position, r_ankle, r_knee, r_hip)
            leg_pub.publish(leg_msg)
        else:
            timer = 0
            step_cnt += 1
            if step_cnt == STEPS:
                action += 1

    else:
        break

    rate.sleep()

