#! /usr/bin/env python3
import rospy
import numpy as np
from raspi import *
from transform import *
from controller_test import *
import matplotlib.pyplot as plt
import matplotlib.patches as pat

N = 1
iterations = 1

if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous = False)
        x_goal=np.array([[0],[0.01]])
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
            obstacle_data = np.array([
                [1.0, 0.000001, 0.7],
                
            ])
            
            dxi = si_barrier_cert(dxi, pose_si, obstacle_data)
            dxu = si_to_uni_dyn(dxi, pose)
            k = set_velocities(N, dxu)
            put_velocities(N, k)
        
        fig, ax = plt.subplots()
        ax.set_xlim((-4, 4))
        ax.set_ylim((-4, 4))
        ax.grid(True)
        circle1 = pat.Circle((0.00001, 0.000001), 0.7, color='red', alpha=0.5)
        ax.add_patch(circle1)

        circle2 = pat.Circle((1.67, 0.025), 0.2, color='green', alpha=0.5)
        ax.add_patch(circle2)

        circle3 = pat.Circle((-0.035, 1.69), 0.2, color='green', alpha=0.5)
        ax.add_patch(circle3)

        circle4 = pat.Circle((-0.04, -1.6), 0.2, color='green', alpha=0.5)
        ax.add_patch(circle4)

        circle5 = pat.Circle((-1.67, 0.05), 0.2, color='green', alpha=0.5)
        ax.add_patch(circle5)


        for i in range(N):
            ax.plot(x_traj[:, i], y_traj[:, i], label=f'Robot {i}')
            ax.scatter(x_traj[0, i], y_traj[0, i], color='red', s=50, marker='o')  # 起点
            ax.scatter(x_traj[-1, i], y_traj[-1, i], color='blue', s=50, marker='x')  # 终点
        ax.legend()
        plt.show()
        
    except rospy.ROSInterruptException:
        rospy.signal_shutdown('End of testing')
        pass

