#!/usr/bin/env python3
import rospy
import numpy as np
import pandas as pd  # 用于保存 CSV 文件
from raspi import *
from transform import *
from controller_cube import *
# from controller_ellpise import *

N = 12
iterations = 1

if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous=False)
        x_goal=np.array([[ 4, 0, -4, 0.05,
                           4,-1.5, -4, 1.5,
                           4,1.5,-4,-1.5],#x
                          [0.05, -3.8, 0, 3.8,
                           1.5,-3.8,-1.5,3.8,
                           -1.5,-4,1.5,4]])#y
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

            obstacle_data = np.array([
                [-0.01, -0.05,0],
                [1.42, 1.28, 0.57],    # cube_1
                [-1.45, 1.2, -0.35],  # cube_2
                [-1.7, -1.7, -1.22], # cube_3
                [1.76, -1.43, -2.11]   # cube_4  
            ])
            dxi = si_barrier_cert(dxi, pose_si, obstacle_data)
            dxu = si_to_uni_dyn(dxi, pose)
            k = set_velocities(N, dxu)
            put_velocities(N, k)

        # 存储轨迹数据到 CSV 文件
        traj_data = pd.DataFrame({
            f'Robot_{i}_x': x_traj[:, i] for i in range(N)
        })
        for i in range(N):
            traj_data[f'Robot_{i}_y'] = y_traj[:, i]

        # 保存为 CSV 文件
        csv_file = "/home/robolab/catkin_ws_jia/src/raspberrypimouse_controlbarrierfunction-main/scripts/data/12cube.csv"
        traj_data.to_csv(csv_file, index=False)
        print(f"轨迹数据已保存到 {csv_file}")

    except rospy.ROSInterruptException:
        rospy.signal_shutdown('End of testing')
        pass