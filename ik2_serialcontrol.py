import numpy as np
import scipy.optimize as opt
from SerialControl import SerialControl
import time

# Se inicia la comunicación serial
sc = SerialControl(port="COM6")
sc.open_serial()
time.sleep(1)

# Definición de constantes globales
CONST1 = 8.825
CONST2 = 39.177
CONST3 = 167.412

def f1_test(theta):
    return (theta[2] + CONST1) * np.cos(np.deg2rad(theta[0] + theta[1])) - CONST2 * np.sin(np.deg2rad(theta[0] + theta[1])) + CONST3 * np.cos(np.deg2rad(theta[0]))

def f2_test(theta):
    return (theta[2] + CONST1) * np.sin(np.deg2rad(theta[0] + theta[1])) + CONST3 * np.sin(np.deg2rad(theta[0])) + CONST2 * np.cos(np.deg2rad(theta[0] + theta[1]))

def create_points(start, end, n):
    x = np.linspace(start[0], end[0], n)
    y = np.linspace(start[1], end[1], n)
    points = np.column_stack((x, y))
    return points

def create_arc(start, end, n):
    start = np.array(start)
    end = np.array(end)
    center = (start + end) / 2
    start_angle = np.arctan2(start[1] - center[1], start[0] - center[0])
    end_angle = np.arctan2(end[1] - center[1], end[0] - center[0])
    angles = np.linspace(start_angle, end_angle, n)
    radius = np.sqrt((start[0] - center[0])**2 + (start[1] - center[1])**2)
    x = center[0] + radius * np.cos(angles)
    y = center[1] + radius * np.sin(angles)
    points = np.column_stack((x, y))
    points[-1] = end
    return points


def inverse_kinematics(posicion_deseada, theta_anterior):
    Q = np.diag([2, 1, 1])
    
    def objetivo(theta_actual):
        return np.dot((theta_actual - theta_anterior).T, np.dot(Q, (theta_actual - theta_anterior)))
    
    def f1(theta):
        return (theta[2] + CONST1) * np.cos(np.deg2rad(theta[0] + theta[1])) - CONST2 * np.sin(np.deg2rad(theta[0] + theta[1])) + CONST3 * np.cos(np.deg2rad(theta[0]))
    
    def f2(theta):
        return (theta[2] + CONST1) * np.sin(np.deg2rad(theta[0] + theta[1])) + CONST3 * np.sin(np.deg2rad(theta[0])) + CONST2 * np.cos(np.deg2rad(theta[0] + theta[1]))

    non_linear_constraint1 = opt.NonlinearConstraint(f1, posicion_deseada[0], posicion_deseada[0])
    non_linear_constraint2 = opt.NonlinearConstraint(f2, posicion_deseada[1], posicion_deseada[1])

    lower_bounds = [-180, -170, 0]
    upper_bounds = [180, 170, 160]
    bounds = opt.Bounds(lower_bounds, upper_bounds)
    
    try:
        result = opt.minimize(objetivo, theta_anterior, method="trust-constr", constraints=[non_linear_constraint1, non_linear_constraint2], bounds=bounds, options={'maxiter': 10000})
        if result.success:
            pass
        else:
            pass
    except Exception as e:
        pass
    
    return result.x

qs = []
def lista_de_posiciones(pos1, pos2, n):
    if np.sign(pos1[1]) != np.sign(pos2[1]):
        pos = create_arc(pos1, pos2, n)
        punto_semilla = [0, 0, 0]
        for punto in pos:
            q = inverse_kinematics(punto, punto_semilla)
            qs.append(q)
            punto_semilla = q
    else:
        pos = create_points(pos1, pos2, n)
        punto_semilla = [0, 0, 0]
        for punto in pos:
            q = inverse_kinematics(punto, punto_semilla)
            qs.append(q)
            punto_semilla = q
    for envio_ang in qs:
        sc.send_angles(envio_ang[0], envio_ang[1], envio_ang[2])

# Comando para realizar homing
sc.home()

lista_de_posiciones([176.24, -39.18], [250, 39.18], 10)

sc.close_serial()
