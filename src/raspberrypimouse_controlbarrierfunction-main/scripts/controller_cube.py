#! /usr/bin/env python3
import numpy as np
from cvxopt import matrix, sparse
from cvxopt.solvers import qp, options
from scipy.integrate import odeint
from math import *
import time
from cvxopt import matrix, solvers, sparse
from scipy.spatial.distance import pdist, squareform
from scipy.optimize import fsolve

options['show_progress'] = False
options['reltol'] = 1e-2 # was e-2
options['feastol'] = 1e-2 # was e-4
options['maxiters'] = 50 # default is 100

# def si_position_controller(xi, positions, x_velocity_gain=1, y_velocity_gain=1, velocity_magnitude_limit=0.5):
#     _,N = np.shape(xi)
#     dxi = np.zeros((2, N))

#         # Calculate control input
#     dxi[0][:] = np.round(x_velocity_gain*(positions[0][:]-xi[0][:]), 2)
#     dxi[1][:] = np.round(y_velocity_gain*(positions[1][:]-xi[1][:]), 2)

#         # Threshold magnitude
#     norms = np.linalg.norm(dxi, axis=0)
#     idxs = np.where(norms > velocity_magnitude_limit)
#     if norms[idxs].size != 0:
#         dxi[:, idxs] *= velocity_magnitude_limit/norms[idxs]

#     return dxi
# cube 0.2 cro1
def si_position_controller(xi, positions, x_velocity_gain=1, y_velocity_gain=1, velocity_magnitude_limit=0.5):
    """
    Single-integrator position controller that calculates velocities based on current and target positions.
    Parameters:
    xi: np.array - Current positions of the robots.
    positions: np.array - Target positions for the robots.
    x_velocity_gain: float - Gain for x-axis velocity control.
    y_velocity_gain: float - Gain for y-axis velocity control.
    velocity_magnitude_limit: float - Maximum allowed magnitude of the velocity vector.
    Returns:
    dxi: np.array - Calculated velocities for robots.
    """
    _, N = np.shape(xi)
    dxi = np.zeros((2, N))

    # Calculate control input without rounding
    dxi[0][:] = x_velocity_gain * (positions[0][:] - xi[0][:])
    dxi[1][:] = y_velocity_gain * (positions[1][:] - xi[1][:])

    # Threshold magnitude smoothly
    norms = np.linalg.norm(dxi, axis=0)
    idxs = np.where(norms > velocity_magnitude_limit)
    if norms[idxs].size != 0:
        scaling_factor = velocity_magnitude_limit / norms[idxs]
        dxi[:, idxs] *= scaling_factor

    return dxi

# cros n=2
def callSolve(Ox, Oy, Px, Py, theta, a, b, n=4):
    """
    计算旋转超椭圆边界点。
    Ox, Oy: 障碍物中心坐标
    Px, Py: 机器人位置坐标
    theta:  障碍物旋转角度（弧度）
    a, b:   超椭圆主轴长度参数
    n:      超椭圆方程的指数(默认4)
    """
    # 1. 相对位置
    relative_position = np.array([Px - Ox, Py - Oy])
    
    # 2. 旋转矩阵 (将全局坐标旋转到障碍物局部坐标系)
    R = np.array([[np.cos(theta), -np.sin(theta)],
                  [np.sin(theta),  np.cos(theta)]])
    
    # 3. 转换到局部坐标系
    local_position = R.T @ relative_position

    # 4. 定义超椭圆方程
    def func(x):
        lx, ly = local_position[0]*x, local_position[1]*x
        return (abs(lx/a)**n + abs(ly/b)**n - 1)

    # 5. 求解root
    # 初值0.5可根据需要调整
    root = fsolve(func, 0.5)

    # 6. 得到局部坐标系中边界点坐标
    local_result = local_position * root[0]

    # 7. 转回全局坐标系
    global_result = (R @ local_result) + np.array([Ox, Oy])
    
    return global_result


def si_barrier_cert(dxi, x, obstacles, barrier_gain=100, safety_radius=0.35, magnitude_limit=0.5, n=4):
    """
    单积分机器人屏障证书
    dxi: 初始速度输入 (2xN)
    x:   机器人位置 (2xN)
    obstacles: 每行 [Ox, Oy, theta, a, b] 表示障碍物参数：
                - (Ox, Oy): 障碍物中心坐标
                - theta: 障碍物旋转角度（弧度）
                - a, b: 超椭圆主轴长度参数
    barrier_gain: 屏障增益
    safety_radius: 机器人之间的安全半径
    magnitude_limit: 速度大小限制
    n: 超椭圆的指数（可根据需要修改为每个障碍物单独指定）
    """
    N = dxi.shape[1]
    num_constraints = comb(N, 2)
    A = np.zeros((num_constraints, 2*N))
    b = np.zeros(num_constraints)

    # 初始化障碍物约束矩阵
    Aob = np.zeros((N * obstacles.shape[0], 2*N))
    bob = np.zeros(N * obstacles.shape[0])

    count = 0
    countObstacles = 0

    # 处理机器人-机器人之间的障碍
    for i in range(N-1):
        for j in range(i+1, N):
            error = x[:, i] - x[:, j]
            h = np.sum(error**2) - safety_radius**2
            A[count, 2*i:2*i+2] = -2 * error
            A[count, 2*j:2*j+2] = 2 * error
            b[count] = barrier_gain * (h**3)
            count += 1

    # 处理机器人-障碍物之间的障碍
    for i in range(N):
        Px, Py = x[:, i]
        for k in range(obstacles.shape[0]):
            Ox, Oy, theta, a, b_ = obstacles[k, :5]
            # 计算障碍物边界点(全局坐标)
            obstacle_boundary_point = callSolve(Ox, Oy, Px, Py, theta, a, b_, n)
            # 相对障碍物中心的向量
            errorObs = np.array([Px - Ox, Py - Oy])
            boundary_vector = obstacle_boundary_point - np.array([Ox, Oy])

            # 计算hObs (机器人与障碍物边界距离差)
            hObs = np.linalg.norm(errorObs) - np.linalg.norm(boundary_vector)

            Aob[countObstacles, 2*i:2*i+2] = -2 * errorObs
            bob[countObstacles] = barrier_gain * (hObs**3)
            countObstacles += 1

    # 合并机器人间和机器人-障碍物约束
    A_full = np.concatenate((A, Aob), axis=0)
    b_full = np.concatenate((b, bob), axis=0)

    # 对输入速度限幅
    norms = np.linalg.norm(dxi, axis=0)
    idxs_to_normalize = (norms > magnitude_limit)
    dxi[:, idxs_to_normalize] *= magnitude_limit / norms[idxs_to_normalize]

    # 构造QP参数
    # min (dxi^T dxi)
    # => H = 2I, f = -2dxi
    N_var = 2*N
    H = 2 * np.eye(N_var)
    f = -2 * dxi.flatten(order='F')

    # 转换为cvxopt格式
    H_cvx = matrix(H)
    f_cvx = matrix(f)
    A_cvx = matrix(A_full)
    b_cvx = matrix(b_full)

    # 调用qp求解
    sol = qp(H_cvx, f_cvx, A_cvx, b_cvx)
    result = np.array(sol['x']).reshape(2, -1, order='F')

    return result

# def superellipse_barrier(error, radius):
#     a = b = radius / 1.5  
#     x, y = error
#     h = ((x / a) ** 4 + (y / b) ** 4 - 1)
#     return h

    
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