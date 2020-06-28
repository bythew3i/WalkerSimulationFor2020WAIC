#!/usr/bin/env python

from math import sqrt, e, asin, acos, sin, cos, atan, pi
import matplotlib.pyplot as plt 
from scipy.interpolate import KroghInterpolator


# CONSTANTS
L1 = 0.275
L2 = 0.290
H = 0.1404
G = 9.8
Zch = 0.098
W = 0.220

# Adjustable variables
S = 0.28
T = 0.8
L = 0.52
Hm = 0.08

# Calculate some useful variables
assert L<sqrt((sqrt((L1+L2)**2-(S/2)**2)+Zch)**2+(S/2)**2)

Td = T/10
Tm = (T+Td)/2



##########################################
# Forward and Back motion planning
##########################################
theta_0 = asin(-S/2/L)
theta_T = -theta_0

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
    [-S, 0, 0, S, 0, 0]
)

def f_x_ank(t):
    if t < Td:
        return -S
    return f_x_ank1(t)

f_z_ank1 = KroghInterpolator(
    [Td, Td, Td, Tm, Tm, Tm],
    [H, 0, 0, Hm+H, 0, 0]
)

f_z_ank2 = KroghInterpolator(
    [Tm, Tm, Tm, T, T, T],
    [Hm+H, 0, 0, H, 0, 0]
)

def f_z_ank(t):
    if t < Td:
        return H
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
    return f_z_cm(t) + H - Zch - f_z_ank(t)

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
# Start and Stop motion planning
##########################################
def derivative(f, x):
    diff = 1e-8
    return (f(x+diff) - f(x))/diff


f_s_ankle_start = KroghInterpolator(
    [0, 0, T, T],
    [0, 0, f_s_ankle(0), derivative(f_s_ankle, 0)]
)
f_s_knee_start = KroghInterpolator(
    [0, 0, T, T],
    [0, 0, f_s_knee(0), derivative(f_s_knee, 0)]
)
f_s_hip_start = KroghInterpolator(
    [0, 0, T, T],
    [0, 0, f_s_hip(0), derivative(f_s_hip, 0)]
)



f_m_ankle_start = KroghInterpolator(
    [0, 0, T, T],
    [0, 0, f_m_ankle(0), derivative(f_m_ankle, 0)]
)
f_m_knee_start = KroghInterpolator(
    [0, 0, T, T],
    [0, 0, f_m_knee(0), derivative(f_m_knee, 0)]
)
f_m_hip_start = KroghInterpolator(
    [0, 0, T, T],
    [0, 0, f_m_hip(0), derivative(f_m_hip, 0)]
)



f_s_ankle_stop = KroghInterpolator(
    [0, 0, T, T],
    [f_s_ankle(0), derivative(f_s_ankle, 0), 0 , 0]
)
f_s_knee_stop = KroghInterpolator(
    [0, 0, T, T],
    [f_s_knee(0), derivative(f_s_knee, 0), 0 , 0]
)
f_s_hip_stop = KroghInterpolator(
    [0, 0, T, T],
    [f_s_hip(0), derivative(f_s_hip, 0), 0 , 0]
)



f_m_ankle_stop = KroghInterpolator(
    [0, 0, T, T],
    [f_m_ankle(0), derivative(f_m_ankle, 0), 0 , 0]
)
f_m_knee_stop = KroghInterpolator(
    [0, 0, T, T],
    [f_m_knee(0), derivative(f_m_knee, 0), 0 , 0]
)
f_m_hip_stop = KroghInterpolator(
    [0, 0, T, T],
    [f_m_hip(0), derivative(f_m_hip, 0), 0 , 0]
)



### delta start & end
f_delta_start = KroghInterpolator(
    [0 ,0, T, T],
    [0, 0, 0, derivative(f_delta, 0)]
)

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

def gen_full_trajectory(f_start, f1, f2, f_stop):
    X, Y = [], []
    X0, Y0 = get_xy(0, T, f_start)
    X1, Y1 = get_xy(T, 2*T, f1)
    X += X0 + X1
    Y += Y0 + Y1
    t = 2*T
    while t < 8*T:
        X2, Y2 = get_xy(t, t+T, f2)
        X3, Y3 = get_xy(t+T, t+2*T, f1)
        X += X2 + X3
        Y += Y2 + Y3
        t += 2*T

    X4, Y4 = get_xy(t, t+T, f_stop)
    X += X4
    Y += Y4
    return X, Y
    





# X, Y = get_xy(0, T, f_x_cm)
# draw_trajectory(
#     X=X,
#     Y=Y,
#     xa="Time (s)",
#     ya="x Position (m)",
#     title="Center of Mass X-Trajectory (T={}s)".format(T)
# )


# X, Y = get_xy(0, T, f_z_cm)
# draw_trajectory(
#     X=X,
#     Y=Y,
#     xa="Time (s)",
#     ya="z Position (m)",
#     title="Center of Mass Z-Trajectory (T={}s)".format(T)
# )


# X, Y = get_xy(0, T, f_x_ank)
# draw_trajectory(
#     X=X,
#     Y=Y,
#     xa="Time (s)",
#     ya="x Position (m)",
#     title="Ankle X-Trajectory (T={}s)".format(T)
# )

# X, Y = get_xy(0, T, f_z_ank)
# draw_trajectory(
#     X=X,
#     Y=Y,
#     xa="Time (s)",
#     ya="z Position (m)",
#     title="Ankle Z-Trajectory (T={}s)".format(T)
# )

# X, Y = get_xy(0, T, f_s_ankle)
# draw_trajectory(
#     X=X,
#     Y=Y,
#     xa="Time (s)",
#     ya="Angle (Rad)",
#     title="Supporting ankle Trajectory (T={}s)".format(T)
# )

# X, Y = get_xy(0, T, f_s_knee)
# draw_trajectory(
#     X=X,
#     Y=Y,
#     xa="Time (s)",
#     ya="Angle (Rad)",
#     title="Supporting knee Trajectory (T={}s)".format(T)
# )

# X, Y = get_xy(0, T, f_s_hip)
# draw_trajectory(
#     X=X,
#     Y=Y,
#     xa="Time (s)",
#     ya="Angle (Rad)",
#     title="Supporting hip Trajectory (T={}s)".format(T)
# )


# X, Y = get_xy(0, T, f_m_ankle)
# draw_trajectory(
#     X=X,
#     Y=Y,
#     xa="Time (s)",
#     ya="Angle (Rad)",
#     title="Moving ankle Trajectory (T={}s)".format(T)
# )

# X, Y = get_xy(0, T, f_m_knee)
# draw_trajectory(
#     X=X,
#     Y=Y,
#     xa="Time (s)",
#     ya="Angle (Rad)",
#     title="Moving knee Trajectory (T={}s)".format(T)
# )

# X, Y = get_xy(0, T, f_m_hip)
# draw_trajectory(
#     X=X,
#     Y=Y,
#     xa="Time (s)",
#     ya="Angle (Rad)",
#     title="Moving hip Trajectory (T={}s)".format(T)
# )



# X, Y = gen_full_trajectory(f_s_ankle_start, f_s_ankle, f_m_ankle, f_s_ankle_stop)
# draw_trajectory(
#     X=X,
#     Y=Y,
#     xa="Time (s)",
#     ya="Angle (Rad)",
#     title="Trajectory (T={}s)".format(T)
# )

# X, Y = gen_full_trajectory(f_m_ankle_start, f_m_ankle, f_s_ankle, f_m_ankle_stop)
# draw_trajectory(
#     X=X,
#     Y=Y,
#     xa="Time (s)",
#     ya="Angle (Rad)",
#     title="Trajectory (T={}s)".format(T)
# )

# X, Y = get_xy(0, T, f_delta_start)
# draw_trajectory(
#     X=X,
#     Y=Y,
#     xa="Time (s)",
#     ya="Angle (Rad)",
#     title="Delta Start Trajectory (T={}s)".format(T)
# )

# X, Y = get_xy(0, T, f_delta_stop_T)
# draw_trajectory(
#     X=X,
#     Y=Y,
#     xa="Time (s)",
#     ya="Angle (Rad)",
#     title="Delta Start Trajectory (T={}s)".format(T)
# )


# X, Y = get_xy(0, T, f_delta_stop_2T)
# draw_trajectory(
#     X=X,
#     Y=Y,
#     xa="Time (s)",
#     ya="Angle (Rad)",
#     title="Delta Start Trajectory (T={}s)".format(T)
# )


# X, Y = get_xy(0, 2*T, f_delta)
# draw_trajectory(
#     X=X,
#     Y=Y,
#     xa="Time (s)",
#     ya="Angle (Rad)",
#     title="Delta CE Trajectory (T={}s)".format(2*T)
# )




# plt.show()



#################################################################################
#################################################################################
# ROS code
#################################################################################
#################################################################################
import rospy
from sensor_msgs.msg import JointState

## Update all the delta
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

leg_joints_pub = rospy.Publisher('/Leg/DesiredJoint', JointState, queue_size=10)
leg_joints_msg = JointState()
leg_joints_msg.position = [0 for _ in range(12)]

rate = rospy.Rate(1000)
nT = 4
cT = -1
timer = 0

# time = 0
# vis_x = []
# vis_y = []
while not rospy.is_shutdown():
    # time += 0.001
    timer += 0.001
    if cT<0:
        if timer < T:
            delta = f_delta_start(timer)
            
            # left (start -> move)
            l_ankle = f_m_ankle_start(timer)
            l_knee = f_m_knee_start(timer)
            l_hip = f_m_hip_start(timer)

            # right (start -> support)
            r_ankle = f_s_ankle_start(timer)
            r_knee = f_s_knee_start(timer)
            r_hip = f_s_hip_start(timer)

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

            #left
            l_ankle = f_m_ankle_stop(timer) if nT%2==0 else f_s_ankle_stop(timer)
            l_knee = f_m_knee_stop(timer) if nT%2==0 else f_s_knee_stop(timer)
            l_hip = f_m_hip_stop(timer) if nT%2==0 else f_s_hip_stop(timer)
            #right
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


    rate.sleep()

# draw_trajectory(
#     X=vis_x,
#     Y=vis_y,
#     xa="time (s)",
#     ya="angle (rad)",
#     title="Delta Trajectory in {}T={}s".format(nT+2, (nT+2)*T)
# )
# plt.show()