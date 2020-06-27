#!/usr/bin/env python

from math import sqrt, e, asin, acos, sin, cos, atan, pi
# import matplotlib.pyplot as plt 
from scipy.interpolate import KroghInterpolator

# CONSTANTS
L1 = 0.275
L2 = 0.290
H = 0.1404
G = 9.8
Zch = 0.098
W = 0.220
S = 0.30
Ht = 0.1

# Adjustable variables
T = 0.8
l1 = 0.495
Hm = Ht+0.05

Hm_START = T/4
INIT_TIME = 1.0
DELTA_FAC = 1.1



# Calculate some useful variables
assert l1 <= sqrt((sqrt((L1+L2)**2-(S/2)**2)+Zch)**2+(S/2)**2)
assert l1 <= sqrt((sqrt((L1+L2)**2-(S/2)**2)+Zch-Ht)**2+(S/2)**2)

Td = T/10
Tm = (T+Td)/2


def derivative(f, x):
    diff = 1e-8
    return (f(x+diff) - f(x))/diff


##########################################
# Forward and Back motion planning
##########################################
l2 = sqrt((sqrt(l1**2-(S/2)**2)+Ht)**2+(S/2)**2)

def f_ln(t):
    return l1+(l2-l1)/T*t

def f_kn(t):
    return sqrt(G/f_ln(t))

theta_0 = asin(-S/2/l1)
theta_T = asin(S/2/l2)

eT1 = e**(-f_kn(T)*T)
eT2 = e**(f_kn(T)*T)
c1 = (theta_0*eT2 - theta_T) / (eT2-eT1)
c2 = (theta_T - theta_0*eT1) / (eT2-eT1)


### Get CM trajectory on x, z
def f_theta(t):
    k = f_kn(t)
    return c1*e**(-k*t) + c2*e**(k*t)

def f_x_cm(t):
    return f_ln(t)*sin(f_theta(t))

def f_z_cm(t):
    return f_ln(t)*cos(f_theta(t))



### Get Ankle trajectory on x, z
f_x_ank1 = KroghInterpolator(
    [Td, Td, T, T],
    [-S, 0, S, 0]
)

def f_x_ank(t):
    if t < Td:
        return -S
    return f_x_ank1(t)


f_z_ank1 = KroghInterpolator(
    [Td, Td, Tm, Tm],
    [-Ht, 0, Hm, 0]
)

f_z_ank2 = KroghInterpolator(
    [Tm, Tm, T, T],
    [Hm, 0, Ht, 0]
)

def f_z_ank(t):
    if t < Td:
        return -Ht
    elif t < Tm:
        return f_z_ank1(t)
    return f_z_ank2(t)


### Get all the angles trajectory on x, z of Supporting Leg
def s_zh(t):
    return f_z_cm(t)-Zch

def s_l0(t):
    return sqrt(f_x_cm(t)**2 + s_zh(t)**2)

def s_alpha(t):
    return atan(f_x_cm(t)/s_zh(t))

def s_alpha_k(t):
    return acos((L1**2+L2**2-s_l0(t)**2)/(2*L1*L2))

def s_alpha_h(t):
    return acos((s_l0(t)**2+L2**2-L1**2)/(2*s_l0(t)*L2))

def s_alpha_a(t):
    return pi - s_alpha_k(t) - s_alpha_h(t)

def f_s_ankle(t):
    return - (s_alpha(t) + s_alpha_a(t))

def f_s_knee(t):
    return pi - s_alpha_k(t)

def f_s_hip(t):
    return - (s_alpha_h(t) - s_alpha(t))


### Get all the angles trajectory on x, z of Moving Leg
def m_zh(t):
    return f_z_cm(t) - Zch - f_z_ank(t)

def m_alpha(t):
    return atan((f_x_ank(t) - f_x_cm(t))/m_zh(t))

def m_l0(t):
    return sqrt((f_x_cm(t)-f_x_ank(t))**2+m_zh(t)**2)

def m_alpha_k(t):
    return acos((L1**2+L2**2-m_l0(t)**2)/(2*L1*L2))

def m_alpha_h(t):
    return acos((m_l0(t)**2+L2**2-L1**2)/(2*m_l0(t)*L2))

def m_alpha_a(t):
    return pi - m_alpha_k(t) - m_alpha_h(t)

def f_m_ankle(t):
    return - (m_alpha_a(t) - m_alpha(t))

def f_m_knee(t):
    return pi - m_alpha_k(t)

def f_m_hip(t):
    return -(m_alpha(t) + m_alpha_h(t))



##########################################
# Left and Right motion planing
##########################################
L = l2
k = sqrt(G/L)
DELTA = asin(W/2/L)
k1 = k*e**(-k*T/2)
k2 = k*e**(k*T/2)
d1 = k2*DELTA/(k1+k2)
d2 = k1*DELTA/(k1+k2)

def f_delta_cm(t):
    k = sqrt(G/L)
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
        return atan(y_ce/z_ce) * DELTA_FAC
    return -atan(y_ce/z_ce) * DELTA_FAC





##########################################
# Start motion planning
##########################################
gamma_0 = 0
gamma_T2 = asin(S/2/l2)

k_gamma = sqrt(G/l2)
e1 = e**(-k_gamma*T/2)
e2 = e**(k_gamma*T/2)
b1 = (gamma_0*e2 - gamma_T2) / (e2-e1)
b2 = (gamma_T2 - gamma_0*e1) / (e2-e1)

# Get CM trajectory om x, z
def f_theta_start(t):
    return b1*e**(-k_gamma*t) + b2*e**(k_gamma*t)

def f_x_cm_start(t):
    return l2*sin(f_theta_start(t))

def f_z_cm_start(t):
    return l2*cos(f_theta_start(t))


### Get Ankle start trajectory on x, z
f_x_ank_start = KroghInterpolator(
    [0, 0, T/2, T/2],
    [0, 0, S, 0]
)

f_z_ank_start1 = KroghInterpolator(
    [0, 0, Hm_START, Hm_START],
    [0, 0, Hm, 0]
)

f_z_ank_start2 = KroghInterpolator(
    [Hm_START, Hm_START, T/2, T/2],
    [Hm, 0, Ht, 0]
)

def f_z_ank_start(t):
    if t < Hm_START:
        return f_z_ank_start1(t)
    return f_z_ank_start2(t)


### Get all the angles trajectory on x, z of Supporting Leg
def s_zh_start(t):
    return f_z_cm_start(t)-Zch

def s_l0_start(t):
    return sqrt(f_x_cm_start(t)**2 + s_zh_start(t)**2)

def s_alpha_start(t):
    return atan(f_x_cm_start(t)/s_zh_start(t))

def s_alpha_k_start(t):
    return acos((L1**2+L2**2-s_l0_start(t)**2)/(2*L1*L2))

def s_alpha_h_start(t):
    return acos((s_l0_start(t)**2+L2**2-L1**2)/(2*s_l0_start(t)*L2))

def s_alpha_a_start(t):
    return pi - s_alpha_k_start(t) - s_alpha_h_start(t)

def f_s_ankle_start(t):
    return - (s_alpha_start(t) + s_alpha_a_start(t))

def f_s_knee_start(t):
    return pi - s_alpha_k_start(t)

def f_s_hip_start(t):
    return - (s_alpha_h_start(t) - s_alpha_start(t))


### Get all the angles trajectory on x, z of Moving Leg
def m_zh_start(t):
    return f_z_cm_start(t) - Zch - f_z_ank_start(t)

def m_alpha_start(t):
    return atan((f_x_ank_start(t) - f_x_cm_start(t))/m_zh_start(t))

def m_l0_start(t):
    return sqrt((f_x_cm_start(t)-f_x_ank_start(t))**2+m_zh_start(t)**2)

def m_alpha_k_start(t):
    return acos((L1**2+L2**2-m_l0_start(t)**2)/(2*L1*L2))

def m_alpha_h_start(t):
    return acos((m_l0_start(t)**2+L2**2-L1**2)/(2*m_l0_start(t)*L2))

def m_alpha_a_start(t):
    return pi - m_alpha_k_start(t) - m_alpha_h_start(t)

def f_m_ankle_start(t):
    return - (m_alpha_a_start(t) - m_alpha_start(t))

def f_m_knee_start(t):
    return pi - m_alpha_k_start(t)

def f_m_hip_start(t):
    return -(m_alpha_start(t) + m_alpha_h_start(t))





f_delta_start = KroghInterpolator(
    [0, 0, T/2, T/2],
    [-f_delta(T/2), 0, f_delta(0), derivative(f_delta, 0)]
)
















##########################################
# Stop motion planning
##########################################
sigma_0 = asin(-S/2/l1)
sigma_T = 0

def f_ln_stop(t):
    return l1+(l2-l1)/(T)*t

def f_kn_stop(t):
    return sqrt(G/f_ln_stop(t))

es1 = e**(-f_kn_stop(T)*T)
es2 = e**(f_kn_stop(T)*T)
a1 = (sigma_0*es2 - sigma_T) / (es2-es1)
a2 = (sigma_T - sigma_0*es1) / (es2-es1)


# Get CM trajectory om x, z
def f_theta_stop(t):
    k = f_kn_stop(t)
    return a1*e**(-k*t) + a2*e**(k*t)

def f_x_cm_stop(t):
    return f_ln_stop(t)*sin(f_theta_stop(t))

def f_z_cm_stop(t):
    return f_ln_stop(t)*cos(f_theta_stop(t))


### Get Ankle stop trajectory on x, z
f_x_ank_stop = KroghInterpolator(
    [0, 0, T, T],
    [-S, 0, 0, 0]
)

f_z_ank_stop1 = KroghInterpolator(
    [0, 0, T/2, T/2],
    [-Ht, 0, Hm, 0]
)

f_z_ank_stop2 = KroghInterpolator(
    [T/2, T/2, T, T],
    [Hm, 0, 0, 0]
)

def f_z_ank_stop(t):
    if t < T/2:
        return f_z_ank_stop1(t)
    return f_z_ank_stop2(t)



### Get all the angles trajectory on x, z of Supporting Leg
def s_zh_stop(t):
    return f_z_cm_stop(t)-Zch

def s_l0_stop(t):
    return sqrt(f_x_cm_stop(t)**2 + s_zh_stop(t)**2)

def s_alpha_stop(t):
    return atan(f_x_cm_stop(t)/s_zh_stop(t))

def s_alpha_k_stop(t):
    return acos((L1**2+L2**2-s_l0_stop(t)**2)/(2*L1*L2))

def s_alpha_h_stop(t):
    return acos((s_l0_stop(t)**2+L2**2-L1**2)/(2*s_l0_stop(t)*L2))

def s_alpha_a_stop(t):
    return pi - s_alpha_k_stop(t) - s_alpha_h_stop(t)

def f_s_ankle_stop(t):
    return - (s_alpha_stop(t) + s_alpha_a_stop(t))

def f_s_knee_stop(t):
    return pi - s_alpha_k_stop(t)

def f_s_hip_stop(t):
    return - (s_alpha_h_stop(t) - s_alpha_stop(t))


### Get all the angles trajectory on x, z of Moving Leg
def m_zh_stop(t):
    return f_z_cm_stop(t) - Zch - f_z_ank_stop(t)

def m_alpha_stop(t):
    return atan((f_x_ank_stop(t) - f_x_cm_stop(t))/m_zh_stop(t))

def m_l0_stop(t):
    return sqrt((f_x_cm_stop(t)-f_x_ank_stop(t))**2+m_zh_stop(t)**2)

def m_alpha_k_stop(t):
    return acos((L1**2+L2**2-m_l0_stop(t)**2)/(2*L1*L2))

def m_alpha_h_stop(t):
    return acos((m_l0_stop(t)**2+L2**2-L1**2)/(2*m_l0_stop(t)*L2))

def m_alpha_a_stop(t):
    return pi - m_alpha_k_stop(t) - m_alpha_h_stop(t)

def f_m_ankle_stop(t):
    return - (m_alpha_a_stop(t) - m_alpha_stop(t))

def f_m_knee_stop(t):
    return pi - m_alpha_k_stop(t)

def f_m_hip_stop(t):
    return -(m_alpha_stop(t) + m_alpha_h_stop(t))



f_delta_stop_T = KroghInterpolator(
    [0, 0, T, T],
    [f_delta(T), derivative(f_delta, T), 0 ,0]
)

f_delta_stop_2T = KroghInterpolator(
    [0, 0, T, T],
    [f_delta(0), derivative(f_delta, 0), 0 ,0]
)



##########################################
# Visualize the trajectory
##########################################
# def draw_trajectory(X, Y, xa, ya, title):
#     plt.figure()
#     plt.plot(X, Y)
#     plt.title(title)
#     plt.xlabel(xa)
#     plt.ylabel(ya)

# def get_xy(start, end, f):
#     X = []
#     Y = []
#     t = start
#     while t < end:
#         X.append(t)
#         Y.append(f(t-start))
#         t += 0.01
#     return X, Y











#################################################################################
#################################################################################
# ROS code
#################################################################################
#################################################################################
import rospy
from sensor_msgs.msg import JointState
from webots_api.srv import SceneSelection


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


rospy.init_node('motion', anonymous=True)

rospy.wait_for_service('/walker/sence', timeout=10)
scene_service = rospy.ServiceProxy(
    "/walker/sence", 
    SceneSelection
)

## Select the scene
scene_service(scene_name="Upstairs", nav=False, vision=False)

leg_joints_pub = rospy.Publisher('/Leg/DesiredJoint', JointState, queue_size=10)
leg_joints_msg = JointState()
leg_joints_msg.position = [0 for _ in range(12)]



rate = rospy.Rate(1000)
nT = 5
cT = -1
timer = 0
ready = False

ankle_init = f_s_ankle_start(0)
knee_init = f_s_knee_start(0)
hip_init = f_s_hip_start(0)
delta_init = -f_delta(T/2)

time = 0
vis_t = []
vis_r_ankle = []
vis_r_knee = []
vis_r_hip = []
vis_l_ankle = []
vis_l_knee = []
vis_l_hip = []
vis_delta = []

while not rospy.is_shutdown():
    timer += 0.001
    
    if not ready:
        if timer < INIT_TIME/2:
            fac = timer / (INIT_TIME/2)
            ankle = ankle_init * fac
            knee = knee_init * fac
            hip = hip_init * fac
            robot_update_l_leg(leg_joints_msg.position, ankle, knee, hip)
            robot_update_r_leg(leg_joints_msg.position, ankle, knee, hip)
            leg_joints_pub.publish(leg_joints_msg)
        elif timer < INIT_TIME:
            fac = (timer - INIT_TIME/2) / (INIT_TIME/2)
            delta = delta_init*fac
            robot_update_all_delta(leg_joints_msg.position, delta)
            leg_joints_pub.publish(leg_joints_msg)
        else:
            ready = True
            timer = 0
    elif cT < 0:
        if timer < T/2:
            delta = f_delta_start(timer)

            # left (start -> move)
            l_ankle = f_s_ankle_start(timer)
            l_knee = f_s_knee_start(timer)
            l_hip = f_s_hip_start(timer)

            # right (start -> support)
            r_ankle = f_m_ankle_start(timer)
            r_knee = f_m_knee_start(timer)
            r_hip = f_m_hip_start(timer)

            # Update
            robot_update_all_delta(leg_joints_msg.position, delta)
            robot_update_l_leg(leg_joints_msg.position, l_ankle, l_knee, l_hip)
            robot_update_r_leg(leg_joints_msg.position, r_ankle, r_knee, r_hip)
            leg_joints_pub.publish(leg_joints_msg)
        else:
            cT += 1
            timer = 0
    elif cT < nT:
        if timer < T:
            delta = f_delta((cT%2)*T + timer)

            # left
            l_ankle = f_m_ankle(timer) if cT%2 == 0 else f_s_ankle(timer)
            l_knee = f_m_knee(timer) if cT%2 == 0 else f_s_knee(timer)
            l_hip = f_m_hip(timer) if cT%2 == 0 else f_s_hip(timer)
            #right
            r_ankle = f_s_ankle(timer) if cT%2 == 0 else f_m_ankle(timer)
            r_knee = f_s_knee(timer) if cT%2 == 0 else f_m_knee(timer)
            r_hip = f_s_hip(timer) if cT%2 == 0 else f_m_hip(timer)

            # Update
            robot_update_all_delta(leg_joints_msg.position, delta)
            robot_update_l_leg(leg_joints_msg.position, l_ankle, l_knee, l_hip)
            robot_update_r_leg(leg_joints_msg.position, r_ankle, r_knee, r_hip)
            leg_joints_pub.publish(leg_joints_msg)
        else:
            cT += 1
            timer = 0
    else:
        if timer < T:
            delta = f_delta_stop_2T(timer) if nT%2==0 else f_delta_stop_T(timer)

            # left 
            l_ankle = f_m_ankle_stop(timer) if nT%2==0 else f_s_ankle_stop(timer)
            l_knee = f_m_knee_stop(timer) if nT%2==0 else f_s_knee_stop(timer)
            l_hip = f_m_hip_stop(timer) if nT%2==0 else f_s_hip_stop(timer)
            # right
            r_ankle = f_s_ankle_stop(timer) if nT%2==0 else f_m_ankle_stop(timer)
            r_knee = f_s_knee_stop(timer) if nT%2==0 else f_m_knee_stop(timer)
            r_hip = f_s_hip_stop(timer) if nT%2==0 else f_m_hip_stop(timer)

            # Update
            robot_update_all_delta(leg_joints_msg.position, delta)
            robot_update_l_leg(leg_joints_msg.position, l_ankle, l_knee, l_hip)
            robot_update_r_leg(leg_joints_msg.position, r_ankle, r_knee, r_hip)
            leg_joints_pub.publish(leg_joints_msg)
        else:
            break

    # time += 0.001
    # vis_t.append(time)
    # vis_r_ankle.append(leg_joints_msg.position[10])
    # vis_r_knee.append(leg_joints_msg.position[9])
    # vis_r_hip.append(leg_joints_msg.position[8])
    # vis_l_ankle.append(leg_joints_msg.position[4])
    # vis_l_knee.append(leg_joints_msg.position[3])
    # vis_l_hip.append(leg_joints_msg.position[2])
    # vis_delta.append(leg_joints_msg.position[5])


    rate.sleep()



# draw_trajectory(
#     X=vis_t,
#     Y=vis_delta,
#     xa="Time (s)",
#     ya="Angle (Rad)",
#     title="delta Trajectory (T={}s)".format(T)
# )
# draw_trajectory(
#     X=vis_t,
#     Y=vis_l_ankle,
#     xa="Time (s)",
#     ya="Angle (Rad)",
#     title="l_ankle Trajectory (T={}s)".format(T)
# )
# draw_trajectory(
#     X=vis_t,
#     Y=vis_l_knee,
#     xa="Time (s)",
#     ya="Angle (Rad)",
#     title="l_knee Trajectory (T={}s)".format(T)
# )
# draw_trajectory(
#     X=vis_t,
#     Y=vis_l_hip,
#     xa="Time (s)",
#     ya="Angle (Rad)",
#     title="l_hip Trajectory (T={}s)".format(T)
# )
# draw_trajectory(
#     X=vis_t,
#     Y=vis_r_ankle,
#     xa="Time (s)",
#     ya="Angle (Rad)",
#     title="r_ankle Trajectory (T={}s)".format(T)
# )
# draw_trajectory(
#     X=vis_t,
#     Y=vis_r_knee,
#     xa="Time (s)",
#     ya="Angle (Rad)",
#     title="r_knee Trajectory (T={}s)".format(T)
# )
# draw_trajectory(
#     X=vis_t,
#     Y=vis_r_hip,
#     xa="Time (s)",
#     ya="Angle (Rad)",
#     title="r_hip Trajectory (T={}s)".format(T)
# )
# plt.show()
