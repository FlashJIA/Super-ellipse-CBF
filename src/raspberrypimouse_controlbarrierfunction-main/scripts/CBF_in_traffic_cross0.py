#!/usr/bin/env python3
import rospy
import numpy as np
import pandas as pd  # 用于保存 CSV 文件
from raspi import *
from transform import *
from controller_aim import *
# from controller_ellpise import *

N = 8
iterations = 1

if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous=False)
        x_goal = np.array([[-2.5, 0.3, -0.3, 2.5, -2.5, -0.35, 0.3, 2.5],
                           [-0.3, -3, 3, 0.3, 0.33, -3, 3, -0.3]])
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

            # obstacle_data = np.array([[0.05, 0,1.35],[]])  # 障碍物数据
            obstacle_data = np.array([[0.05,0.02,0.02,-1.65,1.65],
                          [0.0,-2.35,2.41,0.02,0.02],
                          [1.28,0.34,0.33,0.2,0.2]])
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
        csv_file = "/home/robolab/catkin_ws_jia/src/raspberrypimouse_controlbarrierfunction-main/scripts/prcbf/15.csv"
        traj_data.to_csv(csv_file, index=False)
        print(f"轨迹数据已保存到 {csv_file}")

    except rospy.ROSInterruptException:
        rospy.signal_shutdown('End of testing')
        pass