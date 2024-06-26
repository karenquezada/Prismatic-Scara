import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from SCARA_kinematics import inverse_kinematics
from generador_ptos_semilla import generar_punto_semilla

def F(theta):
    L = 64
    f1 = (theta[2] + 8.825) * np.cos(np.deg2rad(theta[0] + theta[1])) - 39.177 * np.sin(np.deg2rad(theta[0] + theta[1])) + 189.44 * np.cos(np.deg2rad(theta[0]))
    f2 = (theta[2] + 8.825) * np.sin(np.deg2rad(theta[0] + theta[1])) + 189.44 * np.sin(np.deg2rad(theta[0])) + 39.177 * np.cos(np.deg2rad(theta[0] + theta[1]))
    f3 = -theta[3] + 183.9 - L
    return np.array([f1, f2, f3])

def create_points(start, end, n):
    x = np.linspace(start[0], end[0], n)
    y = np.linspace(start[1], end[1], n)
    z = np.linspace(start[2], end[2], n)
    points = np.column_stack((x, y, z))
    return points

qs = []  # Lista para almacenar los valores de q
def lista_de_posiciones(pos1, pos2, n=10):
    pos = create_points(pos1, pos2, n)
    punto = pos[0]
    punto_semilla = generar_punto_semilla(pos1,pos2)
    q = inverse_kinematics(F, punto_semilla, punto)
    qs.append(q)  # Agregar q a la lista
    for i in range(n-1):
        punto = pos[i+1]
        punto_semilla = np.array(q)
        q = inverse_kinematics(F, punto_semilla, punto)
        qs.append(q)  # Agregar q a la lista
    return qs  # Devolver la lista completa

def draw_lines(ax, q1, q2, q3, q4):
    # Borramos las líneas existentes para evitar superposición en cada cuadro de animación
    ax.clear()
    
    # Forward kinematics
    def forward_kinematics2(q1, q2, q3, q4):
        L = 64
        f1 = (q3 + 8.825) * np.cos(np.deg2rad(q1 + q2)) - 39.177 * np.sin(np.deg2rad(q1 + q2)) + 189.44 * np.cos(np.deg2rad(q1))
        f2 = (q3 + 8.825) * np.sin(np.deg2rad(q1 + q2)) + 189.44 * np.sin(np.deg2rad(q1)) + 39.177 * np.cos(np.deg2rad(q1 + q2))
        f3 = -q4 + 183.9 - L
        return f1, f2, f3
    
    def FKlink12(q1):
        f1 = 189.44 * np.cos(np.deg2rad(q1))
        f2 = 189.44 * np.sin(np.deg2rad(q1))
        f3 = 254.9
        return f1, f2, f3
    
    # Draw line from origin to FKlink12(q1)
    ax.plot([0, FKlink12(q1)[0]], [0, FKlink12(q1)[1]], 'r-', label='Link 1')

    # Draw line from FKlink12(q1) to forward_kinematics2(q1, q2, q3, q4, L)
    ax.plot([FKlink12(q1)[0], forward_kinematics2(q1, q2, q3, q4)[0]], [FKlink12(q1)[1], forward_kinematics2(q1, q2, q3, q4)[1]], 'b-', label='Link 2')

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
    offset_end_x = forward_kinematics2(q1, q2, q3, q4)[0] + 39.177 * offset_dir_x
    offset_end_y = forward_kinematics2(q1, q2, q3, q4)[1] + 39.177 * offset_dir_y
    
    # Draw line symbolizing the offset
    ax.plot([forward_kinematics2(q1, q2, q3, q4)[0], offset_end_x], 
            [forward_kinematics2(q1, q2, q3, q4)[1], offset_end_y], 'g-', label='Offset')

    # Calculate the endpoint of the constant length yellow line
    constant_length = 190
    yellow_line_end_x = FKlink12(q1)[0] + constant_length * link2_dir_x
    yellow_line_end_y = FKlink12(q1)[1] + constant_length * link2_dir_y
    
    # Draw yellow line of constant length 215
    ax.plot([FKlink12(q1)[0], yellow_line_end_x], [FKlink12(q1)[1], yellow_line_end_y], 'y-', label='Link 2 - Constant Length')
    
    # Configurar los límites de los ejes
    ax.set_xlim(-400, 400)
    ax.set_ylim(-300, 500)

    # Etiquetas de los ejes
    ax.set_xlabel('X')
    ax.set_ylabel('Y')

    # Añadir leyenda
    ax.legend()


# Valores de q1, q2, q3, q4, L para cada cuadro de la animación
params = lista_de_posiciones([-100, -10, 100], [200, 335, 100], n=10)
"""
punto problematico:[-100, -10, 100], [200, 335, 100]
"""
# Función de actualización para animación
def update(frame, ax, params):
    q1, q2, q3, q4 = params[frame]
    print("q1: ", q1)
    print("q2: ", q2)
    draw_lines(ax, q1, q2, q3, q4)

# Configuración del gráfico
fig, ax = plt.subplots()
animation = FuncAnimation(fig, update, frames=len(params), fargs=(ax, params), interval=500)

# Mostrar la animación
plt.show()
