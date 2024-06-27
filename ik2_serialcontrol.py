import numpy as np
import scipy.optimize as opt
from SerialControl import SerialControl
import time

#se inicia la comunicación serial
sc = SerialControl(port="COM4")
sc.open_serial()
time.sleep(2)


# Definición de constantes globales
CONST1 = 8.825
CONST2 = 39.177
CONST3 = 167.412
CONST4 = 251.49

def create_points(start, end, n):
    x = np.linspace(start[0], end[0], n)
    y = np.linspace(start[1], end[1], n)
    z = np.linspace(start[2], end[2], n)
    points = np.column_stack((x, y, z))
    return points

def create_arc(start, end, center, n):
    start_angle = np.arctan2(start[1] - center[1], start[0] - center[0])
    end_angle = np.arctan2(end[1] - center[1], end[0] - center[0])
    angles = np.linspace(start_angle, end_angle, n)
    radius = np.sqrt((start[0] - center[0])**2 + (start[1] - center[1])**2)
    x = center[0] + radius * np.cos(angles)
    y = center[1] + radius * np.sin(angles)
    z = np.linspace(start[2], end[2], n)
    points = np.column_stack((x, y, z))
    points[-1] = end
    return points

def forward_link2(q1, q2, q3, q4):
    f1 = (q3 + CONST1) * np.cos(np.deg2rad(q1 + q2)) - CONST2 * np.sin(np.deg2rad(q1 + q2)) + CONST3 * np.cos(np.deg2rad(q1))
    f2 = (q3 + CONST1) * np.sin(np.deg2rad(q1 + q2)) + CONST3 * np.sin(np.deg2rad(q1)) + CONST2 * np.cos(np.deg2rad(q1 + q2))
    f3 = -q4 + CONST4
    return f1, f2, f3

def forward_link1(q1):
    f1 = 167.412 * np.cos(np.deg2rad(q1))
    f2 = 167.412 * np.sin(np.deg2rad(q1))
    f3 = 254.9
    return f1, f2, f3

def offset(q1): #solo para graficar
    f1 =39.177*np.cos(np.deg2rad(90-q1))+forward_link1(q1)[0]
    f2 =-39.177*np.sin(np.deg2rad(90-q1))+forward_link1(q1)[1]
    return f1, f2


def inverse_kinematics(posicion_deseada, theta_anterior):
    Q = np.diag([2, 2, 1, 1])
    
    def objetivo(theta_actual):
        return np.dot((theta_actual - theta_anterior).T, np.dot(Q, (theta_actual - theta_anterior)))
    
    def f1(theta):
        return (theta[2] + CONST1) * np.cos(np.deg2rad(theta[0] + theta[1])) - CONST2 * np.sin(np.deg2rad(theta[0] + theta[1])) + CONST3 * np.cos(np.deg2rad(theta[0]))
        #return (theta[2] + CONST1) * np.sin(np.deg2rad(theta[0] + theta[1])) - CONST2 * np.cos(np.deg2rad(theta[0] + theta[1])) + CONST3 * np.sin(np.deg2rad(theta[0]))
    def f2(theta):
        return (theta[2] + CONST1) * np.sin(np.deg2rad(theta[0] + theta[1])) + CONST3 * np.sin(np.deg2rad(theta[0])) + CONST2 * np.cos(np.deg2rad(theta[0] + theta[1]))
        #return (theta[2] + CONST1) * np.cos(np.deg2rad(theta[0] + theta[1])) + CONST3 * np.cos(np.deg2rad(theta[0])) + CONST2 * np.sin(np.deg2rad(theta[0] + theta[1]))
    
    def f3(theta):
        return -theta[3] + CONST4

    def jacobian_f1(theta):
        grad_theta0 = (
            -0.683768 * np.cos(np.deg2rad(theta[0] + theta[1])) -
            2.92189 * np.sin(np.deg2rad(theta[0])) -
            (np.pi / 180) * (theta[2] + CONST1) * np.sin(np.deg2rad(theta[0] + theta[1])))
        
        grad_theta1 = (
            -0.683768 * np.cos(np.deg2rad(theta[0] + theta[1])) -
            (np.pi / 180) * (theta[2] + CONST1) * np.sin(np.deg2rad(theta[0] + theta[1])))
        
        grad_theta2 = np.cos(np.deg2rad(theta[0] + theta[1]))
        
        return np.array([grad_theta0, grad_theta1, grad_theta2, 0])

    def jacobian_f2(theta):
        grad_theta0 = (
            2.92189 * np.cos(np.deg2rad(theta[0])) +
            (np.pi / 180) * (theta[2] + CONST1) * np.cos(np.deg2rad(theta[0] + theta[1])) -
            0.683768 * np.sin(np.deg2rad(theta[0] + theta[1])))
        
        grad_theta1 = (
            (np.pi / 180) * (theta[2] + CONST1) * np.cos(np.deg2rad(theta[0] + theta[1])) -
            0.683768 * np.sin(np.deg2rad(theta[0] + theta[1])))
        
        grad_theta2 = np.sin(np.deg2rad(theta[0] + theta[1]))
        
        return np.array([grad_theta0, grad_theta1, grad_theta2, 0])

    def jacobian_f3(theta):
        return np.array([0, 0, 0, -1])
    
    non_linear_constraint1 = opt.NonlinearConstraint(f1, posicion_deseada[0], posicion_deseada[0], jac=jacobian_f1)
    non_linear_constraint2 = opt.NonlinearConstraint(f2, posicion_deseada[1], posicion_deseada[1], jac=jacobian_f2)
    non_linear_constraint3 = opt.NonlinearConstraint(f3, posicion_deseada[2], posicion_deseada[2], jac=jacobian_f3)

    lower_bounds = [-100, -170, 0, 0]
    upper_bounds = [100, 170, 160, 200]
    bounds = opt.Bounds(lower_bounds, upper_bounds)
    
    try:
        result = opt.minimize(objetivo, theta_anterior, method='SLSQP', constraints=[non_linear_constraint1, non_linear_constraint2, non_linear_constraint3], bounds=bounds, options={'maxiter': 1000})

        if result.success:
            print("Optimización exitosa")
            print("Resultado de theta: ", result.x)
            print("f1: ", f1(result.x))
            print("f2: ", f2(result.x))
            print("f3: ", f3(result.x))
            pass
        else:
            print("Optimización fallida: ", result.message)
            pass
    except Exception as e:
        print("Error durante la optimización: ", str(e))
        pass
    
    return result.x


punto2 = [10,-250,250]
punto1 = [20, 230, 250]
origen = [0,0,70]
# [-92, -10, 68, 0]
qs = [] 
def lista_de_posiciones(pos1, pos2, n):
    if np.sign(pos1[1]) != np.sign(pos2[1]):
        pos = create_arc(pos1, pos2, [0, 0, 70], n)
        punto_semilla =[0,0,0,0]
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
    sc.send_angles(qs)