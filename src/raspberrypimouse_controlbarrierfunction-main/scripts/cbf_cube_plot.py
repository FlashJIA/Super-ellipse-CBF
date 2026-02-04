#! /usr/bin/env python3
import rospy
import numpy as np
import pandas as pd  # 用于保存 CSV 文件
from raspi import *
from transform import *
from controller_cube import *


N = 8
iterations = 1

if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous = False)
        # x_goal=np.array([[4],[0.01]])
        # x_goal=np.array([[3.5,0,-3.5,0],[0.01,-3.5,0,3.5]])
        x_goal=np.array([[ 3.5, 0, -3.5, 0.05,
                           3.5,-1.5, -3.5, 1.5
                            ],#x
                          [0.05, -3.5, 0, 3.5,
                           1.5,-3.5,-1.5,3.5
                           ]])#y
        # x_goal=np.array([[ 3.5, 0, -3.5, 0.05,
        #                    3.5,-1.5, -3.5, 1.5,
        #                    3.5,1.5,-3.5,-1.5],#x
        #                   [0.05, -3.5, 0, 3.5,
        #                    1.5,-3.5,-1.5,3.5,
        #                    -1.5,-3.5,1.5,3.5]])#y
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

                    
            
            print ((np.linalg.norm(x_goal - pose_si)))
            if np.linalg.norm(x_goal - pose_si) < 0.02 :
                for i in range(len(x_goal)):
                    for j in range(len(x_goal[i])):
                        if abs(x_goal[i][j]) == 2:
                            x_goal[i][j] *= -1
              

                count += 1
                if count == iterations:

                    rospy.signal_shutdown('End of testing')
                    pass


            dxi = si_position_controller(pose_si, x_goal)
            
            # [x,y,radius]
            # obstacle_data = np.array([
            #     [-0.01, -0.05,0],
            #     [1.42, 1.28, 0.57],    # cube_1
            #     [-1.45, 1.2, -0.35],  # cube_2
            #     [-1.7, -1.7, -1.22], # cube_3
            #     [1.76, -1.43, -2.11]   # cube_4  
            # ])
            obstacle_data = np.array([
                [-0.01, -0.05,0,0.5,1.08],
                [0.46, 1.80, -2.07,0.5,1.08],    # cube_1
                [-1.45, 1.2, -0.35,0.5,1.08],  # cube_2
                [-1.7, -1.7, -1.22,0.5,1.08], # cube_3
                [1.41, -1.65, -1.96,0.47,1.0],   # cube_4  
                [1.56,0.55,0,0.37,0.37], #cylinder1
                [1.56,-0.58,0,0.35,0.35], #cylinder2
                [-0.24,-2.38,0.37,0.85,0.6], #ellipse
                # [-0.027,-2.38,0.37,0.85,0.6], #ellipse
                [-1.086,-0.51,-1.56,0.45,0.46], #square 2
                [-2.23,-0.49,-1.56,0.45,0.46] #square 1
            ])
            
            dxi = si_barrier_cert(dxi, pose_si, obstacle_data)
            dxu = si_to_uni_dyn(dxi, pose)
            k = set_velocities(N, dxu)
            put_velocities(N, k)
        
        traj_data = pd.DataFrame({
            f'Robot_{i}_x': x_traj[:, i] for i in range(N)
        })
        for i in range(N):
            traj_data[f'Robot_{i}_y'] = y_traj[:, i]

        # 保存为 CSV 文件
        csv_file = "/home/robolab/catkin_ws_jia/src/raspberrypimouse_controlbarrierfunction-main/scripts/cube2/8robots10.csv"
        traj_data.to_csv(csv_file, index=False)
        print(f"轨迹数据已保存到 {csv_file}")
        
    except rospy.ROSInterruptException:
        rospy.signal_shutdown('End of testing')
        pass

