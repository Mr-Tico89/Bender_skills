#!/usr/bin/env python3

import rclpy
import math
from navegation.basicNav import RobotSkills

def main():
    # Inicializar ROS2
    rclpy.init()
    
    # Crear el robot
    robot = RobotSkills()
    
    print("Robot iniciado!")
    
    # 1. Ver dónde estoy
    print("Viendo mi posición actual...")
    robot.where_am_i()
    
    # 2. Ir a un punto
    print("Yendo al punto (2.0, 1.0)...")
    robot.go_to_pose(2.0, 1.0, yaw=math.pi/2)
    
    # 3. Esperar hasta llegar
    print("Esperando a llegar...")
    while rclpy.ok() and robot.is_moving():
        rclpy.spin_once(robot, timeout_sec=0.1)
    
    # 4. Comprobar si llegué
    if robot.reached():
        print("¡Llegué!")
        
        # 5. Rotar un poco
        print("Rotando...")
        robot.rotate(angular_speed=0.5, duration=2.0)
        
        print("¡Todo listo!")
    else:
        print("No pude llegar")
    
    # Limpiar
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
