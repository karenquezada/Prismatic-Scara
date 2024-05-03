import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button

"""
Esta función de jacobiano está lista, se modificó con respecto a la del auxiliar 5, para que ahora 
pueda recibir matrices rectangulares, como es en este caso.
Usar solo si lo estima conveniente (depende del método de resolución que elija).
"""
def jacobian(f, x, epsilon=1e-1): 
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


class SCARAPrisPoseViewer():
    def __init__(self, val_list=[0,0,0,0], L=70):
        #los ángulos se pasan en grados
        self.L = L #largo del lapiz
        self.val_list = val_list
        self.poses = np.array([])
        self.ax3 = None

    def get_pose_to_origin_scara_pris(self):
        
        # Usen np.linalg.multi_dot([Z1, X1, ...., Zn, Xn]), para rotaciones: rot_axis(np.deg2rad(self.values_list[i]))

        # TODO: Implementar la cinematica directa del SCARA

        # Eje del origen
        origen = np.identity(4) 
        Z1=np.linalg.multi_dot([self.translation_z(75.6),self.rot_z(np.deg2rad(self.val_list[0]))])
        X1=np.linalg.multi_dot([self.translation_x(189.44),self.rot_x(np.deg2rad(0))])
        Z2=np.linalg.multi_dot([self.translation_z(179.3),self.rot_z(np.deg2rad(self.val_list[1]+90))])
        X2=np.linalg.multi_dot([self.translation_x(39.177),self.rot_x(np.deg2rad(90))])
        Z3=np.linalg.multi_dot([self.translation_z(8.825+self.val_list[2]),self.rot_z(np.deg2rad(0))])
        X3=np.linalg.multi_dot([self.translation_x(0),self.rot_x(np.deg2rad(90))])
        Z4=np.linalg.multi_dot([self.translation_z(120+self.val_list[3]),self.rot_z(np.deg2rad(0))])
        X4=np.linalg.multi_dot([self.translation_x(0),self.rot_x(np.deg2rad(0))])
        #generar las poses de todos los ejes a la lista de poses
        A1=np.linalg.multi_dot([origen,Z1])
        A2=np.linalg.multi_dot([A1,X1,Z2])
        A3=np.linalg.multi_dot([A2,X2,Z3])
        A4=np.linalg.multi_dot([A3,X3,Z4,X4])

        poses = np.array([origen, A1, A2, A3, A4])
        self.poses = poses
        return self.poses

    def translation_x(self,x):
        return np.array([[1, 0, 0, x], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    
    def translation_y(self,y):
        return np.array([[1, 0, 0, 0], [0, 1, 0, y], [0, 0, 1, 0], [0, 0, 0, 1]])
    
    def translation_z(self, z):
        return np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, z], [0, 0, 0, 1]])

    def rot_x(self, qx):
        return np.array([[1, 0, 0, 0], [0, np.cos(qx), -np.sin(qx), 0], [0, np.sin(qx), np.cos(qx), 0], [0, 0, 0, 1]])

    def rot_y(self, qy):
        return np.array([[np.cos(qy), 0, np.sin(qy), 0], [0, 1, 0, 0], [-np.sin(qy), 0, np.cos(qy), 0], [0, 0, 0, 1]])

    def rot_z(self, qz):
        return np.array([[np.cos(qz), -np.sin(qz), 0, 0], [np.sin(qz), np.cos(qz), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    def draw_axes_tf(self, poses, name="", color="k"):
        for pose in poses:
            self.ax3.set_xlim(-700, 700)
            self.ax3.set_ylim(-700, 700)
            self.ax3.set_zlim(0, 600)
            self.ax3.set_xlabel('x-axis')
            self.ax3.set_ylabel('y-axis')
            self.ax3.set_zlabel('z-axis')
            self.ax3.scatter(xs=[0], ys=[0], zs=[0], marker='o', color=color)
            origin_pose = np.transpose(pose)[3, 0:3]            
            x_rot = np.linalg.multi_dot([pose, [1, 0, 0, 0]])
            y_rot = np.linalg.multi_dot([pose, [0, 1, 0, 0]])
            z_rot = np.linalg.multi_dot([pose, [0, 0, 1, 0]])
            self.ax3.quiver(origin_pose[0], origin_pose[1], origin_pose[2], x_rot[0],
                    x_rot[1], x_rot[2], length=100, normalize=True, color='r')
            self.ax3.quiver(origin_pose[0], origin_pose[1], origin_pose[2], y_rot[0],
                    y_rot[1], y_rot[2], length=100, normalize=True, color='g')
            self.ax3.quiver(origin_pose[0], origin_pose[1], origin_pose[2], z_rot[0],
                    z_rot[1], z_rot[2], length=100, normalize=True, color='b')
            self.ax3.scatter(xs=[origin_pose[0]], ys=[origin_pose[1]],
                    zs=[origin_pose[2]], marker='o')

    def slider(self):
        fig = plt.figure(figsize=(6, 8))
        self.ax3 = fig.add_subplot(111, projection='3d', position=[0.1, 0.3, 0.8, 0.8])

        self.draw_axes_tf(self.poses)

        # Creacion de sliders
        self.slider1_ax = plt.axes([0.2, 0.1, 0.65, 0.03])
        self.slider2_ax = plt.axes([0.2, 0.15, 0.65, 0.03])
        self.slider3_ax = plt.axes([0.2, 0.2, 0.65, 0.03])
        self.slider4_ax = plt.axes([0.2, 0.25, 0.65, 0.03])
        resetax = plt.axes([0.8, 0.025, 0.1, 0.04])

        self.slider1 = Slider(self.slider1_ax, 'Hombro', -90, 90, valinit=self.val_list[0])
        self.slider2 = Slider(self.slider2_ax, 'Codo', -180, 180, valinit=self.val_list[1])
        self.slider3 = Slider(self.slider3_ax, 'Prismático [mm]', 0, 100, valinit=self.val_list[2])
        self.slider4 = Slider(self.slider4_ax, 'Z [mm]', -10.0, 0, valinit=self.val_list[3])
        
        # Agregar boton de reseteo
        button = plt.Button(resetax, 'Reset', color='white', hovercolor='0.975')
        button.on_clicked(self.reset)
        self.slider1.on_changed(self.update)
        self.slider2.on_changed(self.update)
        self.slider3.on_changed(self.update)
        self.slider4.on_changed(self.update)
        plt.show()
        
    def update(self, val):
        # Limpiar el plot
        self.ax3.cla()
        slider_values = np.array([self.slider1.val, self.slider2.val, self.slider3.val, self.slider4.val])
        self.val_list = slider_values
        self.get_pose_to_origin_scara_pris()
        self.draw_axes_tf(self.poses)
        pass

    # Crear un reset para los sliders
    def reset(self, event):
        self.slider1.reset()
        self.slider2.reset()
        self.slider3.reset()
        self.slider4.reset()
        pass
    """
    Función de cinematica directa
    recibe un array de q's que representan las variables que puede tomar el SCARA prismático
    devuelve un array con las coordenadas x,y,z que representa la posición del efector final
    en el espacio cartesiano 3D
    """

    def forward_kinematics(self, parametros):
        """
        Parametros
            param_joints: parametros (angulo o distancia) de los joints en grados o mm.
                    numpy array de forma (4)

        Retorna
            pose: posicion del end effector en mm y orientacion en grados, restringido al workspace del robot.
                    numpy array de forma (4)             
        """
        self.val_list = parametros
        self.get_pose_to_origin_scara_pris()

        # TODO: Obtener la posicion del end effector desde la matriz de transformacion homogenea (self.poses)

        #XYZ = np.array([0, 0, 0], dtype=float) # Reemplazar por columna correspondiente de la matriz de transformacion homogenea (self.poses)
        XYZ = np.array([self.poses[-1][0,3], self.poses[-1][1,3], self.poses[-1][2,3]], dtype=float)
        # Pose final del end effector
        pose = np.array([XYZ[0], XYZ[1], XYZ[2]])
        return pose




    """
    Se agrega la función de cinemática inversa, puede usar cualquier método de resolución que desee.

    En el caso de resolver con Newton Raphson, considerar que a una matriz no cuadrada no se le puede calcular
    el determinante directamente, por lo que en este caso se recomendaría crear una matriz cuadrada multiplicando 
    la matriz jacobiana inversa por su transpuesta, y calcular el determinante de esta matriz cuadrada. 

    La fn. de cinematica inversa recibe como minimo la posición deseada y entrega un array de 4 elementos con las q's que permiten
    al SCARA llegar a la posición deseada.
    """

    def inverse_kinematics(self, f, target_pos, theta0, tol=1e-3, max_iter=500):
        """
            Parametros
                x, y, z: posicion del end effector en mm
                        int o float   

            Retorna
                params: parametros (angulo o distancia) de los joints en grados o mm.
                        numpy array de forma (4)

        """
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
                self.get_pose_to_origin_scara_pris()
                return np.array(theta_new, dtype = float)
            theta = theta_new
            self.values_list = theta
        
        raise ValueError("No se encontró la solución después de {} iteraciones".format(max_iter))

        # TODO: Implementar la cinematica inversa del SCARA

        
    

def F(theta,L=64):
    f1 = (theta[2]+8.825)*np.cos(np.deg2rad(theta[0]+theta[1]))-39.177*np.sin(np.deg2rad(theta[0]+theta[1]))+189.44*np.cos(np.deg2rad(theta[0]))
    f2 = (theta[2]+8.825)*np.sin(np.deg2rad(theta[0]+theta[1]))+189.44*np.sin(np.deg2rad(theta[0]))+39.177*np.cos(np.deg2rad(theta[0]+theta[1]))
    f3 = -theta[3] + 183.9- L
    return np.array([f1, f2, f3])


if __name__ == "__main__":
    scara = SCARAPrisPoseViewer()

    param_joints = [10,120,110,50]# Ejemplo de parametros de los 4 joints, modificar por cualquier otro valor

    initial_guess = np.array([np.pi,np.pi/2,50,0])
    dk = scara.forward_kinematics(param_joints)
    print('Cinematica directa:', dk)
    ik = scara.inverse_kinematics(F, dk, initial_guess, tol=1e-3, max_iter=500)
    print('Cinematica inversa:', ik)

    # param_joints debe ser igual o equivalente a ik
    scara.slider()


