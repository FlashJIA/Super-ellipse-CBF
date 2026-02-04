#! /usr/bin/env python3
import rospy
import numpy as np
from raspi import *
from transform import *
# from controller_cross import *
from controller_ellpise import *
import matplotlib.pyplot as plt
import matplotlib.patches as pat

# 超级椭圆参数
a = 0.8 # x轴半径
b = 1.2  # y轴半径
n = 2  # 控制曲线的形状，n越大越接近矩形
angle = 180  # 超级椭圆的旋转角度



N = 4
iterations = 1
# 超级椭圆公式
def super_ellipse(theta, a, b, n):
    x = np.sign(np.cos(theta)) * (np.abs(np.cos(theta)) ** (2 / n)) * a
    y = np.sign(np.sin(theta)) * (np.abs(np.sin(theta)) ** (2 / n)) * b
    return x, y

if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous=False)
        x_goal=np.array([[-2.5,0.3,-0.3,2.5],[-0.3,-3,3,0.3]])
        x_traj = np.empty((0, N), float)
        y_traj = np.empty((0, N), float)

        count = 0
        while not rospy.is_shutdown():
            pose = getposition(N)
            pose_si = uni_to_si_states(pose)

            x_traj = np.append(x_traj, pose_si[0:1], axis=0)
            y_traj = np.append(y_traj, pose_si[1:2], axis=0)

            print((np.linalg.norm(x_goal - pose_si)))
            if np.linalg.norm(x_goal - pose_si) < 0.02:
                for i in range(len(x_goal)):
                    for j in range(len(x_goal[i])):
                        if abs(x_goal[i][j]) == 2:
                            x_goal[i][j] *= -1

                count += 1
                if count == iterations:
                    rospy.signal_shutdown('End of testing')
                    pass

            dxi = si_position_controller(pose_si, x_goal)

            obstacle_data = np.array([[0.05, 0]])  # 障碍物数据
            dxi = si_barrier_cert(dxi, pose_si, obstacle_data)
            dxu = si_to_uni_dyn(dxi, pose)
            k = set_velocities(N, dxu)
            put_velocities(N, k)

        # 超级椭圆绘图
        theta = np.linspace(0, 2 * np.pi, 100)
        x_super, y_super = super_ellipse(theta, a, b, n)

        # 旋转超级椭圆
        rotation_matrix = np.array([[np.cos(np.radians(angle)), -np.sin(np.radians(angle))],
                                    [np.sin(np.radians(angle)), np.cos(np.radians(angle))]])
        rotated_super = rotation_matrix @ np.array([x_super, y_super])
        x_super_rot, y_super_rot = rotated_super

        # 画图
        fig, ax = plt.subplots()
        ax.set_xlim((-4, 4))
        ax.set_ylim((-4, 4))
        ax.grid(True)

        # 绘制超级椭圆
        ax.plot(x_super_rot, y_super_rot, color="red", label="Super-Ellipse")

        # 绘制机器人轨迹
        for i in range(N):
            ax.plot(x_traj[:, i], y_traj[:, i], label=f'Robot {i}')
            ax.scatter(x_traj[0, i], y_traj[0, i], color='red', s=50, marker='o')  # 起点
            ax.scatter(x_traj[-1, i], y_traj[-1, i], color='blue', s=50, marker='x')  # 终点

        # 添加图例
        ax.legend()
        plt.show()

    except rospy.ROSInterruptException:
        rospy.signal_shutdown('End of testing')
        pass
