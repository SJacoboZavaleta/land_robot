# moose_controller.py
# Controlador del robot Moose usando Pure Pursuit y PID
# Author: Sergio Jacobo Zavaleta
# Date: 25/02/2025
# Version: 3.0
# Curso: Robots de campo

from controller import Robot, Motor, GPS, InertialUnit, Keyboard
import struct
import math
import csv

# Constantes
TIME_STEP = 16
MAX_SPEED = 26.0*0.5  # Velocidad máxima en rad/s
WHEEL_RADIUS = 0.57  # Radio de las ruedas (m)
# https://github.com/cyberbotics/webots/blob/released/projects/robots/clearpath/moose/protos/Moose.proto
v_base = MAX_SPEED * WHEEL_RADIUS # Velocidad base ajustada para e-puck (m/s)
L = 2.4#2.96  # Distancia entre ejes (m)
L_d = 1  # Distancia de lookahead (m)
Kp = 46.0  # Ganancia proporcional del PID
Ki = 0.01  # Ganancia integral del PID
Kd = 5.0  # Ganancia derivativa del PID

# Inicialización del robot
robot = Robot()

# Dispositivos
motors = []
motor_names = ["left motor 1", "left motor 2", "left motor 3", "left motor 4", "right motor 1", "right motor 2", "right motor 3", "right motor 4"]
for i in range(8):
    motor = robot.getDevice(motor_names[i])
    motor.setPosition(float('inf'))
    motors.append(motor)

gps = robot.getDevice("gps")
gps.enable(TIME_STEP)

inertial_unit = robot.getDevice("inertial unit")
inertial_unit.enable(TIME_STEP)

keyboard = Keyboard()
keyboard.enable(TIME_STEP)

# Función para leer la trayectoria desde un archivo CSV
def read_path_from_csv(filename):
    """Función para leer una ruta desde un archivo CSV"""
    path = []
    with open(filename, 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        next(csvreader)  # Omitimos los encabezados
        for row in csvreader:
            x, y, z = map(float, row)  # Convertimos las coordenadas a float
            path.append((x, y, z))
    return path

# Cargar la trayectoria desde un archivo CSV
waypoints = read_path_from_csv('path_combined.csv')

# Variables de estado
current_target_index = 0
integral_error = 0
previous_error = 0

# Variables auxliares
omega = 0
gamma = 0
PID = False # Para activar el control PID con teclado
start_time = robot.getTime()  # Tiempo inicial
total_alpha = 0  # Acumulador de errores (alpha)
iteration_count = 0  # Contador de iteraciones

# Función para calcular la distancia entre dos puntos
def calculate_distance(point1, point2):
    return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

# Función para encontrar el punto de referencia (lookahead point)
def find_lookahead_point(robot_pos, waypoints, L_d, current_idx):
    for i in range(current_idx, len(waypoints)):
        distance = calculate_distance(robot_pos[:2], waypoints[i][:2])  # Solo x, y
        if distance >= L_d:
            return waypoints[i], i
    return waypoints[-1], len(waypoints) - 1

# Función para calcular el ángulo de dirección
def calculate_steering_angle(robot_pos, target_point, robot_yaw):
    dx = target_point[0] - robot_pos[0]
    dy = target_point[1] - robot_pos[1]
    target_angle = math.atan2(dy, dx)
    alpha = target_angle - robot_yaw  # Error angular
    gamma = 2 * math.sin(alpha) / L_d  # Curvatura
    return gamma, alpha

# Controlador PID
def pid_control(error, dt):
    global integral_error, previous_error
    proportional = Kp * error
    integral_error += error * dt
    integral = Ki * integral_error
    derivative = Kd * (error - previous_error) / dt
    previous_error = error
    return proportional + integral + derivative

# Función para establecer la velocidad de los motores
def robot_set_speed(left, right):
    for i in range(4):
        motors[i].setVelocity(left)
        motors[i + 4].setVelocity(right)

# Bucle principal
while robot.step(TIME_STEP) != -1:
    # Obtener posición y orientación del robot
    position_3d = gps.getValues()
    orientation = inertial_unit.getRollPitchYaw()  # Obtener roll, pitch, yaw
    robot_yaw = orientation[2]  # Usamos el ángulo de yaw para la orientación
    robot_pos = (position_3d[0], position_3d[1], robot_yaw)

    # Obtener comandos del teclado
    keys = keyboard.getKey()
    if (keys==Keyboard.ALT+ord('P')):
        print('Pure Pursuit + PID activado !!!')
        PID = True
    if (keys==Keyboard.ALT+ord('L')):
        print('Solo Pure Pursuit activado !!!')
        PID = False
    
    # Verificar si se llegó al objetivo final
    if calculate_distance(robot_pos[:2], waypoints[-1][:2]) < 0.1:
        robot_set_speed(0, 0)
        print("¡Trayectoria completada!")

        # Calcular tiempo total y error promedio
        end_time = robot.getTime()
        total_time = end_time - start_time
        average_alpha = total_alpha / iteration_count if iteration_count > 0 else 0

        # Guardar resultados en un archivo
        with open('simulation_results.txt', 'w') as f:
            f.write(f"Tiempo total de simulación: {total_time:.2f} segundos\n")
            f.write(f"Error promedio (alpha): {average_alpha:.4f} radianes\n")

        print(f"Tiempo total: {total_time:.2f} segundos")
        print(f"Error promedio (alpha): {average_alpha:.4f} radianes")
        break

    # Encontrar el punto de referencia
    lookahead_point, current_target_index = find_lookahead_point(robot_pos, waypoints, L_d, current_target_index)

    # Calcular el ángulo de dirección y la curvatura
    gamma, alpha = calculate_steering_angle(robot_pos, lookahead_point, robot_yaw)

     # Acumular el error (alpha) y contar iteraciones
    total_alpha += abs(alpha)
    iteration_count += 1

    # Controlador PID para ajustar la velocidad angular
    if PID:
        # Calcular velocidad angular usando PID
        omega = pid_control(alpha, TIME_STEP / 1000.0)
    else:
        # Calcular la velocidad angular de referencia
        omega = gamma*v_base
    

    # Calcular velocidades de las ruedas (m/s )
    v_left = v_base - omega * (L / 2)
    v_right = v_base + omega * (L / 2)

    # Ajustar velocidad en pendientes (opcional)
    # Podemos usar la altura (z) para ajustar la velocidad si es necesario
    slope = abs(lookahead_point[2] - robot_pos[2])  # Pendiente aproximada
    if slope > 0.1:  # Si la pendiente es significativa
        v_left *= 0.8  # Reducir velocidad en pendientes
        v_right *= 0.8

    # Limitar las velocidades al máximo permitido (Convirtiendo a rad/s)
    v_left = max(min(v_left/WHEEL_RADIUS, v_base), -v_base)
    v_right = max(min(v_right/WHEEL_RADIUS, v_base), -v_base)

    # Establecer velocidades de los motores
    robot_set_speed(v_left, v_right)

    # Salidas en terminal
    if iteration_count%50 == 0:
        print(f"Modo: {'PID' if PID else 'Pure Pursuit'}, Pos: ({robot_pos[0]:.2f}, {robot_pos[1]:.2f}), " +
          f"Look: ({lookahead_point[0]:.2f}, {lookahead_point[1]:.2f}), " +
          f"Err: {alpha:.2f}, V_L: {v_left:.2f}, V_R: {v_right:.2f}")