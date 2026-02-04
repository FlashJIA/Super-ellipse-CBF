#! /usr/bin/env python3
import rospy
import numpy as np
from raspi import *
from transform import *
from controller_nc import *
import matplotlib.pyplot as plt
import matplotlib.patches as pat

N = 12
iterations = 1

if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous = False)
        # x_goal=np.array([[4],[0.01]])
        # x_goal=np.array([[4,0,-4,0],[0.01,-3.8,0,3.8]])
        x_goal=np.array([[ 3.5, 0, -3.5, 0.05,
                           3.5,-1.5, -3.5, 1.5,
                           3.5,1.5,-3.5,-1.5],#x
                          [0.05, -3.5, 0, 3.5,
                           1.5,-3.5,-1.5,3.5,
                           -1.5,-3.5,1.5,3.5]])#y
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
            #     [-0.01, -0.05,0],
            #     [1.42, 1.28, 0.57],    # cube_1
            #     [-1.45, 1.2, -0.35],  # cube_2
            #     [-1.7, -1.7, -1.22], # cube_3
            #     [1.76, -1.43, -2.11]   # cube_4  
            # ])
            obstacle_data = np.array([
                [-0.01, -0.05,0,1,1],
                [0.46, 1.80, -2.07,1,1],    # cube_1
                [-1.45, 1.2, -0.35,1,1],  # cube_2
                [-1.7, -1.7, -1.22,1,1.0], # cube_3
                [1.41, -1.65, -1.96,1,1.0],   # cube_4  
                [1.56,0.55,0,0.37,0.37], #cylinder1
                [1.56,-0.58,0,0.35,0.35], #cylinder2
                [-0.24,-2.38,0.37,0.85,0.85], #ellipse
                # [-0.027,-2.38,0.37,0.85,0.6], #ellipse
                [-1.086,-0.51,-1.56,0.45,0.46], #square 2
                [-2.23,-0.49,-1.56,0.45,0.46] #square 1
            ])
            
            dxi = si_barrier_cert(dxi, pose_si, obstacle_data)
            dxu = si_to_uni_dyn(dxi, pose)
            k = set_velocities(N, dxu)
            put_velocities(N, k)
        
        fig, ax = plt.subplots()
        ax.set_xlim((-4, 4))
        ax.set_ylim((-4, 4))
        ax.grid(True)
        ellipse=pat.Ellipse((1.5,0.0),1.2,0.75, angle=90,color="blue")
        ax.add_patch(ellipse)
        # rectangle = pat.Rectangle((1.5, -0.75), 1.5, 0.2, angle=90, color = "green")

        # ax.add_patch(rectangle)


        for i in range(N):
            ax.plot(x_traj[:, i], y_traj[:, i], label=f'Robot {i}')
            ax.scatter(x_traj[0, i], y_traj[0, i], color='red', s=50, marker='o')  # 起点
            ax.scatter(x_traj[-1, i], y_traj[-1, i], color='blue', s=50, marker='x')  # 终点
        ax.legend()
        plt.show()
        
    except rospy.ROSInterruptException:
        rospy.signal_shutdown('End of testing')
        pass

