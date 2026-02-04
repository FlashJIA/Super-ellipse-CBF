#! /usr/bin/env python3
import numpy as np
from cvxopt import matrix, sparse

from scipy.sparse import identity
from cvxopt import matrix, sparse, solvers
from scipy.special import comb
from cvxopt.solvers import qp, options
from scipy.integrate import odeint
from math import *
import time

options['show_progress'] = False
options['reltol'] = 1e-2 # was e-2
options['feastol'] = 1e-2 # was e-4
options['maxiters'] = 50 # default is 100

def si_position_controller(xi, positions, x_velocity_gain=1, y_velocity_gain=1, velocity_magnitude_limit=0.5):
    _,N = np.shape(xi)
    dxi = np.zeros((2, N))

        # Calculate control input
    dxi[0][:] = np.round(x_velocity_gain*(positions[0][:]-xi[0][:]), 2)
    dxi[1][:] = np.round(y_velocity_gain*(positions[1][:]-xi[1][:]), 2)

        # Threshold magnitude
    norms = np.linalg.norm(dxi, axis=0)
    idxs = np.where(norms > velocity_magnitude_limit)
    if norms[idxs].size != 0:
        dxi[:, idxs] *= velocity_magnitude_limit/norms[idxs]

    return dxi

def si_position_controller_2(xi, positions, x_velocity_gain=1, y_velocity_gain=1, velocity_magnitude_limit=0.1):
    _,N = np.shape(xi)
    dxi = np.zeros((2, N))

        # Calculate control input
    dxi[0][:] = (x_velocity_gain*(positions[0][:]-xi[0][:]))
    dxi[1][:] = (y_velocity_gain*(positions[1][:]-xi[1][:]))

        # Threshold magnitude
    norms = np.linalg.norm(dxi, axis=0)
    idxs = np.where(norms > velocity_magnitude_limit)
    if norms[idxs].size != 0:
        dxi[:, idxs] *= velocity_magnitude_limit/norms[idxs]

    return dxi

def si_barrier_cert(dxi, x, obstacles, barrier_gain=110, safety_radius_robot=0.3, magnitude_limit=0.25):
    """
    对不同障碍物设置不同的安全半径(safety radius)的屏障证书函数。
    
    假设 obstacles 的形状为 (3, M):
    obstacles = [
        [Ox_1, Ox_2, ..., Ox_M],
        [Oy_1, Oy_2, ..., Oy_M],
        [R_1,  R_2,  ..., R_M ]  # 每个障碍物的安全半径
    ]

    参数:
    - dxi: 原始控制输入 (2xN)
    - x: 机器人位置 (2xN)
    - obstacles: 障碍物信息 (3xM)
    - barrier_gain: 屏障增益
    - safety_radius_robot: 机器人之间的安全半径
    - magnitude_limit: 最大速度幅度限制
    """

    start_time = time.time()
    N = dxi.shape[1]
    num_constraints = int(comb(N, 2))
    A = np.zeros((num_constraints, 2*N))
    b = np.zeros(num_constraints)

    # 障碍物数量
    M = obstacles.shape[1] if obstacles.size > 0 else 0

    Aob = np.zeros((N * M, 2*N))
    bob = np.zeros(N * M)

    # 构造H矩阵，使用2*I
    H = sparse(matrix(2 * np.identity(2*N)))
    count = 0
    countObstacles = 0

    # Constraints between robots
    for i in range(N-1):
        for j in range(i+1, N):
            error = x[:, i] - x[:, j]
            h = (error[0]**2 + error[1]**2) - safety_radius_robot**2

            A[count, 2*i:2*i+2] = -2 * error
            A[count, 2*j:2*j+2] =  2 * error
            b[count] = barrier_gain * (h**3)
            count += 1

    # Constraints between robots and obstacles
    # 假设obstacles为[ [Ox_1,...], [Oy_1,...], [R_1,...] ]
    if M > 0:
        for i in range(N):
            for k in range(M):
                Ox = obstacles[0, k]
                Oy = obstacles[1, k]
                safety_radius_obstacle_k = obstacles[2, k]  # 第k个障碍物的安全半径

                errorObs = x[:, i] - np.array([Ox, Oy])
                hObs = (errorObs[0]**2 + errorObs[1]**2) - safety_radius_obstacle_k**2

                Aob[countObstacles, 2*i:2*i+2] = -2 * errorObs
                bob[countObstacles] = barrier_gain * (hObs**3)

                countObstacles += 1

    # Combine robot-robot and robot-obstacle constraints
    if M > 0:
        A = np.vstack((A, Aob))
        b = np.concatenate((b, bob))

    # Threshold control inputs before QP
    norms = np.linalg.norm(dxi, axis=0)
    idxs_to_normalize = norms > magnitude_limit
    dxi[:, idxs_to_normalize] *= magnitude_limit / norms[idxs_to_normalize]

    f = -2 * dxi.flatten(order='F')

    # Solve QP
    result = solvers.qp(matrix(H), matrix(f), matrix(A), matrix(b))['x']
    
    return np.reshape(result, (2, -1), order='F')

def robotFeedbackControl(xi, positions): #P controller
    
    #Feedback control parameter for REAL ROBOT
    GOAL_DIST_THRESHOLD=0.08
    K_RO=2
    K_ALPHA=13
    V_CONST=0.1

    #Feedback control parameter for SIMULATED ROBOT
    # GOAL_DIST_THRESHOLD=0.05
    # K_RO=3
    # K_ALPHA=13
    # V_CONST=0.2

    _,N = np.shape(xi)
    dxi = np.zeros((2, N))

    norms = np.linalg.norm(xi[0:2, 0:N]-positions, axis = 0)
    lamda = np.arctan2(positions[1][:]-xi[1][:], positions[0][:]-xi[0][:])
    alpha = np.array([(lamda[:] - xi[2][:] + pi) % (2 * pi) - pi]) #-360degrees
    v = K_RO * norms
    w = K_ALPHA * alpha

    dxi[0][:] = np.round((v[:] / abs(v[:]) * V_CONST), 2)
    dxi[1][:] = np.round((w[:] / abs(v[:]) * V_CONST), 2)

    idxs = np.where(norms < GOAL_DIST_THRESHOLD)
    dxi[:, idxs] = 0
    return dxi

def robotPDFeedbackControl(xi, positions, n, a, GOAL_DIST_THRESHOLD=0.05, K_RO=3, K_ALPHA=13, Kd_RO = 5, Kd_ALPHA = 5, V_CONST=0.3): #PD controller
    _,N = np.shape(xi)
    dxi = np.zeros((2, N))

    norms = np.linalg.norm(xi[0:2, 0:N]-positions, axis = 0)
    lamda = np.arctan2(positions[1][:]-xi[1][:], positions[0][:]-xi[0][:])
    alpha = np.array([(lamda[:] - xi[2][:] + pi) % (2 * pi) - pi]) #-360degrees
    d_norms = norms - n
    d_alpha = alpha - a
    v = (K_RO * norms) + (Kd_RO * d_norms)
    w = (K_ALPHA * alpha) + (Kd_ALPHA * d_alpha)
    n = norms
    a = alpha
    dxi[0][:] = v[:] / abs(v[:]) * V_CONST
    dxi[1][:] = w[:] / abs(v[:]) * V_CONST
    idxs = np.where(norms < GOAL_DIST_THRESHOLD)
    dxi[:, idxs] = 0
    return dxi, n, a

def diff_equation_fatii(y_list,t,e,omega):
    ki = 22
    sum_fu = y_list[1] + e
    coe = 0
    for i in range(2,len(y_list)):
        if i%2 == 0:
            coe = int(i/2)
            sum_fu += (y_list[i] + e*cos(coe*omega*t)) * cos(coe*omega*t)
        else:
            coe = int((i-1)/2)
            sum_fu += (y_list[i] + e*sin(coe*omega*t)) * sin(coe*omega*t)

        result = []
        result.append(-ki*e-sum_fu + 20*cos(pi*t))
        result.append(-e)
    for i in range(2, len(y_list)):
        if i%2 == 0:
            coe = int(i/2)
            result.append(coe*e*omega*sin(coe*omega*t) + ki*e*cos(coe*omega*t))
        else:
            coe = int((i-1)/2)
            result.append((-e)*coe*omega*cos(coe*omega*t)+ki*e*sin(coe*omega*t))
    return np.array(result)
                     

def cal_tra_fatii(new_coords,new_centroids):
    T=5
    t=np.linspace(0,T,num=1)
    omega = pi*2/T
    point_lists = []
    for i in range(len(new_coords)):
        y_list_x = [new_coords[i][0],0,0,0,0,0,0,0,0,0,0]
        y_list_y = [new_coords[i][1],0,0,0,0,0,0,0,0,0,0]
        result_x = odeint(diff_equation_fatii, y_list_x, t, args=(new_coords[i][0]-new_centroids[i][0],omega))
        result_y = odeint(diff_equation_fatii, y_list_y, t, args=(new_coords[i][1]-new_centroids[i][1],omega))
        result_xt = result_x[:,0]
        result_yt = result_y[:,0]
        new_result = np.vstack((np.array(result_xt), np.array(result_yt))).T
        point_lists.append(list(new_result))
    return point_lists