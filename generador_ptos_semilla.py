import numpy as np

def generar_punto_semilla(pos_inicial, pos_final):
    # Calcular la dirección desde el punto inicial al punto final
    direccion_x = pos_final[0] - pos_inicial[0]
    direccion_y = pos_final[1] - pos_inicial[1]
    sector = 2
    if pos_inicial[0]>0:
        sector=2
    if pos_inicial[0]<0:
        sector=1
    
    # Calcular el ángulo de dirección
    angulo_direccion = np.arctan2(direccion_y, direccion_x)

    if sector == 1:  # Sector 1, debe moverse en sentido horario (decreciente)
        theta1 = -90
        theta2 = np.degrees(angulo_direccion) + 90
    else:  # Sector 2, debe moverse en sentido antihorario (creciente)
        theta1 = 90
        theta2 = np.degrees(angulo_direccion) + 90

    theta3 = 0
    theta4 = 0

    # Asegurar que los ángulos estén en el rango adecuado (-180 a 180 grados)
    theta1 %= 360
    theta2 %= 360
    
    return np.array([theta1, theta2, theta3, theta4])
