import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.linalg import expm
from IPython.display import display, clear_output
from matplotlib.animation import FuncAnimation

def screw_matrix(w, q):
    w_hat = np.array([[0, -w[2], w[1]],
                      [w[2], 0, -w[0]],
                      [-w[1], w[0], 0]])

    screw_matrix = np.vstack([np.hstack([w_hat, np.reshape(q, (3, 1))]),
                             [0, 0, 0, 0]])

    return screw_matrix

def plot_robot(robot_frames, ax):
    ax.clear()

    for i in range(len(robot_frames) - 1):
        frame_start = robot_frames[i]
        frame_end = robot_frames[i + 1]

        # Plotting the robot joints
        ax.quiver(frame_end[0, 3], frame_end[1, 3], frame_end[2, 3],
                  frame_end[0, 0], frame_end[1, 0], frame_end[2, 0],
                  length=0.1, normalize=True, color='r')

        ax.quiver(frame_end[0, 3], frame_end[1, 3], frame_end[2, 3],
                  frame_end[0, 1], frame_end[1, 1], frame_end[2, 1],
                  length=0.1, normalize=True, color='g')

        ax.quiver(frame_end[0, 3], frame_end[1, 3], frame_end[2, 3],
                  frame_end[0, 2], frame_end[1, 2], frame_end[2, 2],
                  length=0.1, normalize=True, color='b')

        # Connect frames with lines
        ax.plot([frame_start[0, 3], frame_end[0, 3]],
                [frame_start[1, 3], frame_end[1, 3]],
                [frame_start[2, 3], frame_end[2, 3]], color=(215/255, 91/255, 91/255), linewidth=5.0)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')


def main():
    joint_axes = [
        (np.array([0, 0, 0]), np.array([0, 0, 3])),
        (np.array([0, 0, 0]), np.array([0, -2, 0])),
        (np.array([0, 0, 0]), np.array([5, 0, 2])),
        (np.array([0, 0, 0]), np.array([0, 2, 0])),
        (np.array([0, 0, 0]), np.array([5, 0, 2])),
        (np.array([0, 0, 0]), np.array([1, 0, -1])),
        (np.array([0, 0, 0]), np.array([1, 0, -1])),
    ]

    current_frame = np.eye(4)
    robot_frames = [current_frame]

    for axis in joint_axes:
        w, q = axis
        screw_matrix_i = screw_matrix(w, q)
        current_frame = np.dot(current_frame, expm(screw_matrix_i))
        robot_frames.append(current_frame)

    %matplotlib qt 
    fig = plt.figure()
    ax = fig.add_subplot((111), aspect='equal', projection='3d')
    ax.scatter((1, 2), (1, 1), (1, 2)) 

    ax.set_box_aspect([np.ptp(ax.get_xlim()), np.ptp(ax.get_ylim()), np.ptp(ax.get_zlim())])

    for i in range(len(robot_frames) - 1):
        plot_robot(robot_frames[:i + 1], ax)
        plt.pause(0.0001) 
        clear_output(wait=True)

    plt.show()

if __name__ == "__main__":
    main()

