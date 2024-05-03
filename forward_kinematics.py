import sympy as sp
import numpy as np
import matplotlib.pyplot as plt

#symbolic variables
q1,q2,q3,q4 = sp.symbols('q1 q2 q3 q4')
L=sp.symbols('L')
#pen length
def denavit_hartenberg_matrix(alpha,a,d,theta):
    return sp.Matrix([[sp.cos(theta), -sp.sin(theta)*sp.cos(alpha), sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
                      [sp.sin(theta), sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
                      [0, sp.sin(alpha), sp.cos(alpha), d],
                      [0, 0, 0, 1]])


a1=189.44
a2=0
a3=0
a4=0
alpha1=0
alpha2=sp.pi/2
alpha3=sp.pi/2
alpha4=0
d1=75.6
d2=179.3
d3=8.825+q3
d4=71+L+q4
theta1=q1
theta2=q2+sp.pi/2
theta3=0
theta4=0
# Transformation matrices
# Calculate transformation matrices
T0_1 = denavit_hartenberg_matrix(alpha1, a1, d1, theta1)
T1_2 = denavit_hartenberg_matrix(alpha2, a2, d2, theta2)
T2_3 = denavit_hartenberg_matrix(alpha3, a3, d3, theta3)
T3_4 = denavit_hartenberg_matrix(alpha4, a4, d4, theta4)

# Calculate combined transformation matrices
T0_2 = T0_1 * T1_2
T0_3 = T0_2 * T2_3
T0_4 = T0_3 * T3_4

# print("T0_4:")
# print(T0_4[0:3,3])
# print("T0_2 ")
# print(T0_2[0:3,3])


# Forward kinematics
q=np.array([])
def forward_kinematics(q1,q2,q3,q4,L=25):
    q1,q2,q3,q4=q
    f1 = (q3+8.825)*np.cos(np.deg2rad(q1+q2))+189.44*np.cos(np.deg2rad(q1))
    f2 = (q3+8.825)*np.sin(np.deg2rad(q1+q2))+189.44*np.sin(np.deg2rad(q1))
    f3 = -q4 + 183.9- L
    return f1, f2, f3

def jacobian(f, x, epsilon=1e-1): #epsilon es el error admisible
    n = len(x)
    m = len(f(x))
    jacobian_matrix = np.zeros((m, n))
    f_x = f(x)
    
    for i in range(n):
        perturbation = np.zeros(n)
        perturbation[i] = epsilon
        f_x_plus = f(x + perturbation)
        jacobian_matrix[:, i] = (f_x_plus - f_x) / epsilon
        
    return jacobian_matrix


def inverse_kinematics(f, theta0, target_pos, tol=1e-3, max_iter=500):
    theta = theta0
    for _ in range(max_iter):
        jacobiano = jacobian(f, theta)
        try:
            jacobiano_inv = np.linalg.inv(jacobiano)
        except np.linalg.LinAlgError:
            jacobiano_inv = np.linalg.pinv(jacobiano)
        square_jacobian = np.dot(jacobiano_inv.T, jacobiano_inv)
        if np.abs(np.linalg.det(square_jacobian)) > tol:
            theta_new = theta - jacobiano_inv.dot(f(theta) - target_pos)
        else:
            theta_new = theta + np.random.rand(len(theta))          
        if np.linalg.norm(theta_new - theta) < tol:
            return theta_new
        theta = theta_new
    raise ValueError("No se encontró la solución después de {} iteraciones".format(max_iter)) 

#function that contains forward kinematics
def F(theta,L=25):
    f1 = (theta[2]+8.825)*np.cos(np.deg2rad(theta[0]+theta[1]))+189.44*np.cos(np.deg2rad(theta[0]))
    f2 = (theta[2]+8.825)*np.sin(np.deg2rad(theta[0]+theta[1]))+189.44*np.sin(np.deg2rad(theta[0]))
    f3 = -theta[3] + 183.9- L
    return np.array([f1, f2, f3])

position=[100,200,100]
initial_guess = np.array([np.pi,np.pi/2,50,0])
q=inverse_kinematics(F, initial_guess, position)
print("q1: ",q[0])
print("q2: ",q[1])
print("q3: ",q[2])
print("q4: ",q[3])
print("L: ",25)
print("evalurar puntos: ",forward_kinematics(q[0],q[1],q[2],q[3],25))
