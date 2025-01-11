#!/usr/bin/env python3

import numpy as np
from scipy.integrate import solve_ivp
from scipy import constants
from scipy.differentiate import derivative
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

mass = 0.18 # kg
I = np.array([[0.00025,   0,          2.55e-6], \
              [0,         0.000232,   0], \
              [2.55e-6,   0,          0.0003738]])
invI = np.linalg.inv(I)

sim_time = 10

# Converts roll, pitch, yaw to a body-to-world rotation matrix
def rpy_to_rot_zxy(phi, theta, psi):
    return np.array([[np.cos(psi) * np.cos(theta) - np.sin(phi) * np.sin(psi) * np.sin(theta), \
                      np.cos(theta) * np.sin(psi) + np.cos(psi) * np.sin(phi) * np.sin(theta), \
                      -np.cos(phi) * np.sin(theta)], \
                     [-np.cos(phi) * np.sin(psi), \
                       np. cos(phi) * np.cos(psi), \
                       np.sin(phi)], \
                     [np.cos(psi) * np.sin(theta) + np.cos(theta) * np.sin(phi) * np.sin(psi), \
                      np.sin(psi) * np.sin(theta) - np.cos(psi) * np.cos(theta) * np.sin(phi), \
                      np.cos(phi) * np.cos(theta)]])

def rot_to_quat(R):
    tr = np.trace(R)    
    if tr > 0:
        S = 2.0 * np.sqrt(tr + 1.0)
        q_w = 0.25 * S
        q_x = (R[2, 1] - R[1, 2]) / S
        q_y = (R[0, 2] - R[2, 0]) / S
        q_z = (R[1, 0] - R[0, 1]) / S
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        q_w = (R[2, 1] - R[1, 2]) / S
        q_x = 0.25 * S
        q_y = (R[0, 1] + R[1, 0]) / S
        q_z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        q_w = (R[0, 2] - R[2, 0]) / S
        q_x = (R[0, 1] + R[1, 0]) / S
        q_y = 0.25 * S
        q_z = (R[1, 2] + R[2, 1]) / S
    else:
        S = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        q_w = (R[1, 0] - R[0, 1]) / S
        q_x = (R[0, 2] + R[2, 0]) / S
        q_y = (R[1, 2] + R[2, 1]) / S
        q_z = 0.25 * S
    
    return np.array([q_w, q_x, q_y, q_z])

# quaternion to body-to-world rotation matrix
def quat_to_rot(quat):
    qnorm = np.linalg.norm(quat)
    if (qnorm > 0.0):
        quat = quat / qnorm

    qahat = np.zeros((3, 3))

    qahat[0][1] = -quat[3]
    qahat[0][2] =  quat[2]
    qahat[1][2] = -quat[1]
    qahat[1][0] =  quat[3]
    qahat[2][0] = -quat[2]
    qahat[2][1] =  quat[1]

    Rot = np.eye(3) + 2.0 * qahat @ qahat + 2 * quat[0] * qahat

    return Rot

def traj_gen(t):
    x_lambda = lambda t: 1.0 * np.sin(0.5 * constants.pi / sim_time * t)
    y_lambda = lambda t: 1.0 * np.sin(0.5 * constants.pi / sim_time * t)
    z_lambda = lambda t: 1.0 * np.sin(0.5 * constants.pi / sim_time * t)

    x = x_lambda(t)
    y = y_lambda(t)
    z = z_lambda(t)

    x_dot = derivative(x_lambda, t).df
    y_dot = derivative(y_lambda, t).df
    z_dot = derivative(z_lambda, t).df

    delta = 1e-6
    x_dot_ = derivative(x_lambda, t - delta).df
    y_dot_ = derivative(y_lambda, t - delta).df
    z_dot_ = derivative(z_lambda, t - delta).df

    x_dotdot = (x_dot - x_dot_) / delta
    y_dotdot = (y_dot - y_dot_) / delta
    z_dotdot = (z_dot - z_dot_) / delta

    yaw = 0.0
    yaw_dot = 0.0

    return x, y, z, x_dot, y_dot, z_dot, x_dotdot, y_dotdot, z_dotdot, yaw, yaw_dot

def controller(t, s):
    x = s[0]
    y = s[1]
    z = s[2]
    x_dot = s[3]
    y_dot = s[4]
    z_dot = s[5]
    qW = s[6]
    qX = s[7]
    qY = s[8]
    qZ = s[9]
    p  = s[10]
    q  = s[11]
    r  = s[12]

    quat = np.array([qW, qX, qY, qZ])
    bRw = quat_to_rot(quat)  # Rot from World to Body

    phi   = np.arcsin(bRw[1][2]) # roll
    the = np.arctan2(-bRw[0][2] / np.cos(phi), bRw[2][2] / np.cos(phi)) # pitch
    psi   = np.arctan2(-bRw[1][0] / np.cos(phi), bRw[1][1] / np.cos(phi)) # yaw

    s_des = traj_gen(t)

    x_des = s_des[0]
    y_des = s_des[1]
    z_des = s_des[2]
    x_dot_des = s_des[3]
    y_dot_des = s_des[4]
    z_dot_des = s_des[5]
    x_dotdot_des = s_des[6]
    y_dotdot_des = s_des[7]
    z_dotdot_des = s_des[8]
    yaw_des = s_des[9]
    yaw_dot_des = s_des[10]

    kpr3 = 50.0
    kdr3 = 11.0
    F = mass * (constants.g + z_dotdot_des \
                  + kdr3 * (z_dot_des - z_dot)) + kpr3 * (z_des - z)
    
    F = np.maximum(0.0, F)

    kpr1 = kpr3 / mass
    kdr1 = kdr3 / mass
    
    kpr2 = kpr1
    kdr2 = kdr1

    r1_dotdot_c = x_dotdot_des + kdr1 * (x_dot_des - x_dot) + kpr1 * (x_des - x)
    r2_dotdot_c = y_dotdot_des + kdr2 * (y_dot_des - y_dot) + kpr2 * (y_des - y)

    phi_des = 1.0 / constants.g * (r1_dotdot_c * np.sin(yaw_des) - r2_dotdot_c * np.cos(yaw_des))
    the_des = 1.0 / constants.g * (r1_dotdot_c * np.cos(yaw_des) + r2_dotdot_c * np.sin(yaw_des))

    kp_phi = 1.0
    kd_phi = 0.1

    kp_the = kp_phi
    kd_the = kd_phi

    kp_psi = 1.0
    kd_psi = 0.1

    phi_c = kp_phi * (phi_des - phi) + kd_phi * (0.0 - p)
    the_c = kp_the * (the_des - the) + kd_the * (0.0 - q)
    psi_c = kp_psi * (yaw_des - psi) + kd_psi * (yaw_dot_des - r)

    M = np.array([phi_c, the_c, psi_c])

    # print(x, y, z, phi, the, psi)

    # print("c: ", t, r1_dotdot_c, r2_dotdot_c, phi_des, the_des, F, M)

    return F, M

# quadrotor dynamic model dx/dt = f(x, t)
def dxdt(t, s):
    # assign states
    x = s[0]
    y = s[1]
    z = s[2]
    x_dot = s[3]
    y_dot = s[4]
    z_dot = s[5]
    qW = s[6]
    qX = s[7]
    qY = s[8]
    qZ = s[9]
    p  = s[10]
    q  = s[11]
    r  = s[12]

    quat = np.array([qW, qX, qY, qZ])

    bRw = quat_to_rot(quat)  # Rot from World to Body
    wRb = np.linalg.inv(bRw) # Rot from Body to World

    F, M = controller(t, s)

    # acceleration
    accel = 1.0 / mass * (wRb @ np.array([0.0, 0.0, F]) - np.array([0.0, 0.0, mass * constants.g]))

    # angular velocity
    K_quat = 2.0
    quaterror = 1.0 - (qW * qW + qX * qX + qY * qY + qZ * qZ)

    q_dot = -0.5 * np.array([[0, -p, -q, -r], \
                            [p,  0, -r,  q], \
                            [q,  r,  0, -p], \
                            [r, -q,  p,  0]]) @ quat + K_quat * quaterror * quat
    
    # angular acceleration
    omega = np.array([p, q, r])
    omega_dot = invI @ (M - np.cross(omega, I @ omega))

    # print("S: ", wRb, accel)

    s_dot = [0.0] * 13

    s_dot[0] = x_dot
    s_dot[1] = y_dot
    s_dot[2] = z_dot
    s_dot[3] = accel[0]
    s_dot[4] = accel[1]
    s_dot[5] = accel[2]
    s_dot[6] = q_dot[0]
    s_dot[7] = q_dot[1]
    s_dot[8] = q_dot[2]
    s_dot[9] = q_dot[3]
    s_dot[10] = omega_dot[0]
    s_dot[11] = omega_dot[1]
    s_dot[12] = omega_dot[2]

    print("s: ", t, x, y, z)

    return s_dot

def format_axes(fig):
    for i, ax in enumerate(fig.axes):
        ax.text(0.5, 0.5, "ax%d" % (i+1), va="center", ha="center")
        ax.tick_params(labelbottom=False, labelleft=False)

phi0   = 0.0
theta0 = 0.0
psi0   = 0.0

rot0   = rpy_to_rot_zxy(phi0, theta0, psi0)
quat0  = rot_to_quat(rot0)

'''
    [ x0 x1 x2 x3 x4 x5 x6        x7        x8        x9        x10 x11 x12 ]
    [ x  y  z  x' y' z' qW        qX        qY        qZ        p   q   r]
'''
x0 =  0, 0, 0, 0, 0, 0, quat0[0], quat0[1], quat0[2], quat0[3], 0,  0,  0

t0, tf = 0, sim_time
soln = solve_ivp(dxdt, (t0, tf), x0, method='RK45', t_eval = np.linspace(t0, tf, 1000))

quats = np.array([soln.y[6], soln.y[7], soln.y[8], soln.y[9]])
quats = np.transpose(quats)

phis = []
thes = []
psis = []

for quat in quats:
    bRw = quat_to_rot(quat)
    phi = np.arcsin(bRw[1][2]) # roll
    the = np.arctan2(-bRw[0][2] / np.cos(phi), bRw[2][2] / np.cos(phi)) # pitch
    psi = np.arctan2(-bRw[1][0] / np.cos(phi), bRw[1][1] / np.cos(phi)) # yaw    
    phis.append(phi)
    thes.append(the)
    psis.append(psi)

fig = plt.figure(layout="constrained")

gs = GridSpec(9, 2, figure=fig)

ax_x = fig.add_subplot(gs[0, 0])
ax_y = fig.add_subplot(gs[1, 0])
ax_z = fig.add_subplot(gs[2, 0])
ax_vx = fig.add_subplot(gs[3, 0])
ax_vy = fig.add_subplot(gs[4, 0])
ax_vz = fig.add_subplot(gs[5, 0])
ax_phi = fig.add_subplot(gs[6, 0])
ax_the = fig.add_subplot(gs[7, 0])
ax_psi = fig.add_subplot(gs[8, 0])

ax_3d = fig.add_subplot(gs[:, 1], projection = '3d')

ax_x.plot(soln.t, soln.y[0], color='k')
ax_y.plot(soln.t, soln.y[1], color='k')
ax_z.plot(soln.t, soln.y[2], color='k')

ax_vx.plot(soln.t, soln.y[3], color='k')
ax_vy.plot(soln.t, soln.y[4], color='k')
ax_vz.plot(soln.t, soln.y[5], color='k')

ax_phi.plot(soln.t, phis, color='k')
ax_the.plot(soln.t, thes, color='k')
ax_psi.plot(soln.t, psis, color='k')

ax_3d.plot(soln.y[0], soln.y[1], soln.y[2], color='k')

fig.suptitle("quadrotor flight sim")

plt.show()
