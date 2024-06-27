import numpy as np
import scipy.optimize as opt


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

def forward_link2(q1, q2, q3):
    f1 = (q3 + CONST1) * np.cos(np.deg2rad(q1 + q2)) - CONST2 * np.sin(np.deg2rad(q1 + q2)) + CONST3 * np.cos(np.deg2rad(q1))
    f2 = (q3 + CONST1) * np.sin(np.deg2rad(q1 + q2)) + CONST3 * np.sin(np.deg2rad(q1)) + CONST2 * np.cos(np.deg2rad(q1 + q2))
    return f1, f2

def forward_link1(q1):
    f1 = 167.412 * np.cos(np.deg2rad(q1))
    f2 = 167.412 * np.sin(np.deg2rad(q1))
    return f1, f2

def offset(q1): #solo para graficar
    f1 =39.177*np.cos(np.deg2rad(90-q1))+forward_link1(q1)[0]
    f2 =-39.177*np.sin(np.deg2rad(90-q1))+forward_link1(q1)[1]
    return f1, f2


def inverse_kinematics(posicion_deseada, theta_anterior):
    Q = np.diag([2, 1, 1])
    
    def objetivo(theta_actual):
        return np.dot((theta_actual - theta_anterior).T, np.dot(Q, (theta_actual - theta_anterior)))
    
    def f1(theta):
        return (theta[2] + CONST1) * np.cos(np.deg2rad(theta[0] + theta[1])) - CONST2 * np.sin(np.deg2rad(theta[0] + theta[1])) + CONST3 * np.cos(np.deg2rad(theta[0]))
    def f2(theta):
        return (theta[2] + CONST1) * np.sin(np.deg2rad(theta[0] + theta[1])) + CONST3 * np.sin(np.deg2rad(theta[0])) + CONST2 * np.cos(np.deg2rad(theta[0] + theta[1]))

    non_linear_constraint1 = opt.NonlinearConstraint(f1, posicion_deseada[0], posicion_deseada[0])#, jac=jacobian_f1)
    non_linear_constraint2 = opt.NonlinearConstraint(f2, posicion_deseada[1], posicion_deseada[1])#, jac=jacobian_f2)

    lower_bounds = [-180, -170, 0]
    upper_bounds = [180, 170, 160]
    bounds = opt.Bounds(lower_bounds, upper_bounds)
    
    try: #SLSQP #trust-constr
        result = opt.minimize(objetivo, theta_anterior,method="trust-constr", constraints=[non_linear_constraint1, non_linear_constraint2], bounds=bounds, options={'maxiter': 10000})
        if result.success:
            # print("status", result.status)
            # print("Optimización exitosa")
            # print("Resultado de theta: ", result.x)
            # print("f1: ", f1(result.x))
            # print("f2: ", f2(result.x))
            pass
        else:
            #print("Optimización fallida: ", result.message)
            pass
    except Exception as e:
        #print("Error durante la optimización: ", str(e))
        pass
    
    return result.x


qs = [] 
def lista_de_posiciones(pos1, pos2, n):
    if np.sign(pos1[1]) != np.sign(pos2[1]): #posicion y cambia de signo
        pos = create_arc(pos1, pos2, n)
        punto_semilla =[0,0,0]
        for punto in pos:
            q = inverse_kinematics(punto, punto_semilla)
            qs.append(q)
            punto_semilla = q  
    else:
        pos = create_points(pos1, pos2, n)
        punto_semilla =[0,0,0]
        for punto in pos:
            q = inverse_kinematics(punto, punto_semilla)
            qs.append(q)
            punto_semilla = q  
    return qs

# punto1=[176.24, 30.18]
# punto2= [300, 39.18]
# vector=lista_de_posiciones(punto1,punto2,10)
# print("qs," , vector)
