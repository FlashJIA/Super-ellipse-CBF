#! /usr/bin/env python3
import rospy
import numpy as np
from raspi import *
import pandas as pd  # 用于保存 CSV 文件
from transform import *
# from controller_cross import *
from controller_cube import *
import matplotlib.pyplot as plt
import matplotlib.patches as pat

N = 8
iterations = 1

if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous = False)
        x_goal=np.array([[-2.5,0.3,-0.3,2.5,-2.5,-0.35,0.3,2.5]
                        ,[-0.3,-3,3,0.3,0.33,-3,3,-0.3]])
        x_traj = np.empty((0, N), float)
        y_traj = np.empty((0, N), float)

        count = 0
        norms = np.zeros((1,N))
        alpha = np.zeros((1,N))
        while not rospy.is_shutdown():
            pose = getposition(N)
            pose_si = uni_to_si_states(pose)

            x_traj = np.append(x_traj, pose_si[0:1], axis=0)
            y_traj = np.append(y_traj, pose_si[1:2], axis=0)

                    

            # For plotting Trajectories
            # x_traj = np.append(x_traj, pose[0:1], axis=0)
            # y_traj = np.append(y_traj, pose[1:2], axis=0)

            # if(np.linalg.norm(x_goal - pose_md_si) > 0.02):
            #     x_traj_md = np.append(x_traj_md, pose_md_si[0:1], axis=0)
            #     y_traj_md = np.append(y_traj_md, pose_md_si[1:2], axis=0)
            # print ('X', x_traj_md)
            # print ('Y', y_traj_md)
            
            print ((np.linalg.norm(x_goal - pose_si)))
            if np.linalg.norm(x_goal - pose_si) < 0.02 :
                for i in range(len(x_goal)):
                    for j in range(len(x_goal[i])):
                        if abs(x_goal[i][j]) == 2:
                            x_goal[i][j] *= -1
                # for i in range(len(x_goal)): #row
                #     for j in range(len(x_goal[i]) - 1): #column
                #         t = x_goal[i][j]
                #         x_goal[i][j] = x_goal[i][j - 2]
                #         x_goal[i][j-2] = 

                count += 1
                if count == iterations:

                    # For plotting Trajectories
                    # np.savetxt(LOG_DIR+'/X_traj.csv', x_traj, delimiter=' , ')
                    # np.savetxt(LOG_DIR+'/Y_traj.csv', y_traj, delimiter=' , ')
                    # np.savetxt(LOG_DIR+'/X_traj_md.csv', x_traj_md, delimiter=' , ')
                    # np.savetxt(LOG_DIR+'/Y_traj_md.csv', y_traj_md, delimiter=' , ')
                    rospy.signal_shutdown('End of testing')
                    pass


            dxi = si_position_controller(pose_si, x_goal)
            # [x,y,radius]
            # obstacle_data = np.array([
            #     [0.00001, 0.000001, 0.7],
            #     [1.67, 0.025, 0.01],
    # 处理机器人之间的障碍
    # for i in range(N-1):
    #     for j in range(i+1, N):
    #         error = x[:, i] - x[:, j]
    #         h = np.sum(error**2) - safety_radius**2
    #         A[count, 2*i:2*i+2] = -2*error
    #         A[count, 2*j:2*j+2] = 2*error
    #         b[count] = barrier_gain * pow(h, 3)
    #         count += 1

    # 处理机器人与障碍物之间的障碍
    
            #     [-0.035,1.69,0.01],
            #     [-0.04, -1.6, 0.01],
            #     [-1.67,0.025,0.01]
            # ])
            obstacle_data = np.array([
               [0.1, 0.001,0,1.05,1.32],
                [0.02, -2.43, 0,0.27,0.39],    # cube_1
                [0.02, 2.45, 0,0.27,0.39],  # cube_2
                [-1.65, 0.02, 0,0.2,0.22],  # cube_2
                [1.65, 0.02, 0,0.20,0.2]  # cube_2
                
                
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
        csv_file = "/home/robolab/catkin_ws_jia/src/raspberrypimouse_controlbarrierfunction-main/scripts/cros/cross15.csv"
        traj_data.to_csv(csv_file, index=False)
        print(f"轨迹数据已保存到 {csv_file}")

    except rospy.ROSInterruptException:
        rospy.signal_shutdown('End of testing')
        pass

