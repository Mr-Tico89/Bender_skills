"""
Funciones avanzadas de navegación para RobotSkills
Complementa las funciones básicas con capacidades más sofisticadas
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import math
import json
import os
from typing import Dict, Optional, List
from .skills import RobotSkills


class AdvancedNavigation:
    """
    Clase que proporciona funciones avanzadas de navegación
    que extienden las capacidades básicas de RobotSkills.
    """
    
    def __init__(self, robot_skills: RobotSkills):
        """
        Inicializa la navegación avanzada.
        
        Args:
            robot_skills: Instancia de RobotSkills para usar las funciones básicas
        """
        self.robot = robot_skills
 


def main():
    """Ejemplo de uso de navegación avanzada"""
    rclpy.init()
    
    try:
        # Crear robot básico
        robot = RobotSkills()
        
        # Crear navegación avanzada
        advanced_nav = AdvancedNavigation(robot)
        
        print("Demo de Navegación Avanzada")
        
        # Esperar inicialización
        robot.navigator.waitUntilNav2Active()
        
        # Demo de mirar hacia un punto
        print("\n1. Mirando hacia punto (2, 2):")
        advanced_nav.look_to_pose(2.0, 2.0)
        robot.wait_for_result()
        
        # Demo de navegación por waypoints
        print("\n2. Navegación por waypoints:")
        waypoints = [
            {'x': 1.0, 'y': 0.0, 'yaw': 0.0},
            {'x': 1.0, 'y': 1.0, 'yaw': math.pi/2},
            {'x': 0.0, 'y': 1.0, 'yaw': math.pi},
            {'x': 0.0, 'y': 0.0, 'yaw': -math.pi/2}
        ]
        advanced_nav.navigate_with_intermediate_goals(waypoints)
        
        # Demo de órbita
        print("\n3. Órbita circular:")
        advanced_nav.orbit_around_point(0.0, 0.0, radius=1.5)
        
        print("\nDemo de navegación avanzada completado!")
        
    except KeyboardInterrupt:
        print("\nInterrumpido por el usuario")
        
    finally:
        if 'robot' in locals():
            robot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()