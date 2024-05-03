import matplotlib.pyplot as plt
import numpy as np

def draw_lines(q1, q2, q3, q4, L):
    
    # Forward kinematics
    def forward_kinematics2(q1, q2, q3, q4, L):
        f1 = (q3+8.825)*np.cos(np.deg2rad(q1+q2))+-39.177*np.sin(np.deg2rad(q1+q2))+189.44*np.cos(np.deg2rad(q1))
        f2 = (q3+8.825)*np.sin(np.deg2rad(q1+q2))+189.44*np.sin(np.deg2rad(q1))+39.177*np.cos(np.deg2rad(q1+q2))
        f3 = -q4 + 183.9- L
        return f1, f2, f3

    def FKlink12(q1):
        f1 = (189.44 * np.cos(np.deg2rad(q1)))-39.177*np.sin(np.deg2rad(q1+q2))
        f2 =  189.44 * np.sin(np.deg2rad(q1)) + 39.177 * np.cos(np.deg2rad(q1+q2))
        f3 = 254.9
        return f1, f2, f3

    # Draw line from origin to FKlink1(q1, q2)
    plt.plot([0, FKlink12(q1)[0]], [0, FKlink12(q1)[1]], 'r-', label='Line 1')

    # Draw line from FKlink1(q1, q2) to forward_kinematics(q1, q2, q3, q4, L)
    plt.plot([FKlink12(q1)[0], forward_kinematics2(q1, q2, q3, q4, L)[0]], [FKlink12(q1)[1], forward_kinematics2(q1, q2, q3, q4, L)[1]], 'b-', label='Line 2')

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.xlim(-360, 360)  # Set x-axis limits
    plt.ylim(0, 360)  # Set y-axis limits
    plt.legend()
    plt.show()

    # Plot vertical position (z)
    plt.plot([forward_kinematics2(q1, q2, q3, 0, 0)[0], forward_kinematics2(q1, q2, q3, q4, L)[0]], [forward_kinematics2(q1, q2, q3, 0, 0)[2], forward_kinematics2(q1, q2, q3, q4, L)[2]], 'b-', label='Line 2')

    plt.xlabel('X')
    plt.ylabel('Z')
    plt.xlim(-360, 360)  # Set x-axis limits
    plt.ylim(0, 400)  # Set y-axis limits
    plt.legend()
    plt.show()

    print("x: ", forward_kinematics2(q1, q2, q3, q4, L)[0])
    print("y: ", forward_kinematics2(q1, q2, q3, q4, L)[1])
    print("z: ", forward_kinematics2(q1, q2, q3, q4, L)[2])

draw_lines(29.951270823208574, -5.011696547310612, -83.21071656476593, 50, 64)
