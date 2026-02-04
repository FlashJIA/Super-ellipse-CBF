#! /usr/bin/env python3
import rospy
import numpy as np
from raspi import *
from transform import *
from controller_aim import *
import matplotlib.pyplot as plt
import matplotlib.patches as pat

N = 1
iterations = 1

if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous = False)
        # x_goal = np.array([[-2, -2, 2, 2, 0.25, -0.25, 0.25, -0.25], [0.25, -0.25, 0.25, -0.25, -2, -2, 2, 2]])
        x_goal=np.array([[0],[0.01]])
        x_traj = np.empty((0, N), float)
        y_traj = np.empty((0, N), float)
        # {4, 5, 6, 7}    {0, 1, 2, 3}

        # For plotting Trajectories
        # LOG_DIR = '/home/robolab/raspi_ws/src/coverage_control/Data'
        # x_traj = np.empty((0, N), float)
        # y_traj = np.empty((0, N), float)
        # x_traj_md = np.empty((0, N), float)
        # y_traj_md = np.empty((0, N), float)
        # pose_md = getposition(N)
        # pose_md_si = uni_to_si_states(pose_md)

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

            #for plotting trajectories
            # dxi_md = si_position_controller_2(pose_md_si, x_goal)
            # pose_md_si = np.add(pose_md_si, dxi_md)

            # dxi = si_barrier_cert(dxi, pose_si)
            dxi = si_barrier_cert(dxi, pose_si, np.transpose(np.array([[1.5, 0.001]])))
            dxu = si_to_uni_dyn(dxi, pose)
            
            k = set_velocities(N, dxu)
            put_velocities(N, k)
         
        fig, ax = plt.subplots()
        ax.set_xlim((-4, 4))
        ax.set_ylim((-4, 4))
        ax.grid(True)
        ellipse=pat.Ellipse((1.5,0.0),1.2,0.75, angle=90,color="blue")
        ax.add_patch(ellipse)


        for i in range(N):
            ax.plot(x_traj[:, i], y_traj[:, i], label=f'Robot {i}')
            ax.scatter(x_traj[0, i], y_traj[0, i], color='red', s=50, marker='o')  # 起点
            ax.scatter(x_traj[-1, i], y_traj[-1, i], color='blue', s=50, marker='x')  # 终点
        ax.legend()
        plt.show()
        

    except rospy.ROSInterruptException:
        rospy.signal_shutdown('End of testing')
        pass
