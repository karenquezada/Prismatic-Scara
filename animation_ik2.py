import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from ik2 import inverse_kinematics, forward_link2, forward_link1, offset, create_points, create_arc

qs = [] 
def lista_de_posiciones(pos1, pos2, n):
    punto_semilla = np.array([0, 0, 0, 0])
    if np.sign(pos1[1]) != np.sign(pos2[1]):
        pos = create_arc(pos1, pos2, [0, 0, 70], n)
        for punto in pos:
            q = inverse_kinematics(punto, punto_semilla)
            qs.append(q)
            punto_semilla = q  
    else:
        pos = create_points(pos1, pos2, n)
        for punto in pos:
            q = inverse_kinematics(punto, punto_semilla)
            qs.append(q)
            punto_semilla = q  
    return qs


def draw_lines(ax, q1, q2, q3, q4):
    ax.clear()
    #draw the link 1
    ax.plot([0, forward_link1(q1)[1]], [0, forward_link1(q1)[0]], 'r-', label='Link 1')
    #calculate the direction vector of link2:
    link2_dir_y = forward_link2(q1,q2,q3,q4)[0] - offset(q1)[0]
    link2_dir_x = forward_link2(q1,q2,q3,q4)[1] - offset(q1)[1]
    #Normalize the direction vector
    link2_length = np.sqrt(link2_dir_y ** 2 + link2_dir_x ** 2)
    link2_dir_y /= link2_length
    link2_dir_x /= link2_length

    #draw the offset
    ax.plot([offset(q1)[1], forward_link1(q1)[1]], [offset(q1)[0], forward_link1(q1)[0]], 'g-', label='Offset')
    #draw the link 2
    ax.plot([forward_link2(q1,q2,q3,q4)[1], offset(q1)[1]], [forward_link2(q1,q2,q3,q4)[0], offset(q1)[0]], 'b-', label='Link 2')
    # Calculate the endpoint of the constant length yellow line
    constant_length= 190
    yellow_line_y = offset(q1)[0] + constant_length * link2_dir_y
    yellow_line_x = offset(q1)[1] + constant_length * link2_dir_x 
    #draw the yellow line
    ax.plot([forward_link2(q1,q2,q3,q4)[1], yellow_line_x], [forward_link2(q1,q2,q3,q4)[0], yellow_line_y], 'y-', label='Constant link')
    
    
    #Axis limits
    ax.set_xlim(-400, 400)
    ax.set_ylim(-400, 400)

    #Axis labels
    ax.set_xlabel('Y')
    ax.set_ylabel('X')

    #Legend
    ax.legend()

params = lista_de_posiciones([20,230,250],[10,-250,250], 10)
  
#Update function for the animation
def update(frame, ax, params):
    q1, q2, q3, q4 = params[frame]
    # print("q1: ", q1)
    # print("q2: ", q2)
    draw_lines(ax, q1, q2, q3, q4)

fig, ax = plt.subplots()
animation = FuncAnimation(fig, update, frames=len(params), fargs=(ax, params), interval=500)

plt.show()
