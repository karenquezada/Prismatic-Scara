import matplotlib.pyplot as plt
import numpy as np

def draw_lines(q1, q2, q3, q4):
    
    # Forward kinematics
    def forward_kinematics2(q1, q2, q3, q4):
        L=64
        f1 = (q3+8.825)*np.cos(np.deg2rad(q1+q2))+-39.177*np.sin(np.deg2rad(q1+q2))+189.44*np.cos(np.deg2rad(q1))
        f2 = (q3+8.825)*np.sin(np.deg2rad(q1+q2))+189.44*np.sin(np.deg2rad(q1))+39.177*np.cos(np.deg2rad(q1+q2))
        f3 = -q4 + 183.9- L
        return f1, f2, f3

    def FKlink12(q1):
        # f1 = (189.44 * np.cos(np.deg2rad(q1)))-39.177*np.sin(np.deg2rad(q1+q2))
        # f2 =  189.44 * np.sin(np.deg2rad(q1)) + 39.177 * np.cos(np.deg2rad(q1+q2))
        f1 = (189.44 * np.cos(np.deg2rad(q1)))
        f2 =  189.44 * np.sin(np.deg2rad(q1))
        f3 = 254.9
        return f1, f2, f3

    # Draw line from origin to FKlink1(q1, q2)
    plt.plot([0, FKlink12(q1)[0]], [0, FKlink12(q1)[1]], 'r-', label='Link 1')

    
    plt.plot([FKlink12(q1)[0], forward_kinematics2(q1, q2, q3, q4)[0]], [FKlink12(q1)[1], forward_kinematics2(q1, q2, q3, q4)[1]], 'b-', label='Link 2')
    # Calculate the direction vector of Link 2
    link2_dir_x = forward_kinematics2(q1, q2, q3, q4)[0] - FKlink12(q1)[0]
    link2_dir_y = forward_kinematics2(q1, q2, q3, q4)[1] - FKlink12(q1)[1]
    
    # Normalize the direction vector
    link2_length = np.sqrt(link2_dir_x ** 2 + link2_dir_y ** 2)
    link2_dir_x /= link2_length
    link2_dir_y /= link2_length
    
    # Calculate the direction vector of the perpendicular offset
    offset_dir_x = -link2_dir_y  # Perpendicular direction
    offset_dir_y = link2_dir_x   # Perpendicular direction
    
    # Calculate the endpoint of the offset
    offset_end_x = forward_kinematics2(q1, q2, q3, q4)[0] + 39 * offset_dir_x
    offset_end_y = forward_kinematics2(q1, q2, q3, q4)[1] + 39 * offset_dir_y
    
    # Draw line symbolizing the offset
    plt.plot([forward_kinematics2(q1, q2, q3, q4)[0], offset_end_x], 
            [forward_kinematics2(q1, q2, q3, q4)[1], offset_end_y], 'g-', label='Offset')
    # Draw line from FKlink1(q1, q2) to forward_kinematics(q1, q2, q3, q4)
        # Calculate the endpoint of the constant length yellow line
    constant_length = 190
    yellow_line_end_x = FKlink12(q1)[0] + constant_length * link2_dir_x
    yellow_line_end_y = FKlink12(q1)[1] + constant_length * link2_dir_y
    
    # Draw yellow line of constant length 215
    plt.plot([FKlink12(q1)[0], yellow_line_end_x], [FKlink12(q1)[1], yellow_line_end_y], 'y-', label='Link 2 - Constant Length')

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.xlim(-400, 400)  # Set x-axis limits
    plt.ylim(-300, 500)  # Set y-axis limits
    plt.legend()
    plt.show()

    print("x: ", forward_kinematics2(q1, q2, q3, q4)[0])
    print("y: ", forward_kinematics2(q1, q2, q3, q4)[1])
    print("z: ", forward_kinematics2(q1, q2, q3, q4)[2])

draw_lines(29.95, -5, 100,0)
