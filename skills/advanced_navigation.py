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
        self.rotation_timer = None
    
    def go_to_pose_stamped(self, pose_stamped: PoseStamped) -> bool:
        """
        Navega a una pose usando un mensaje PoseStamped completo.
        
        Args:
            pose_stamped: Mensaje PoseStamped con la pose objetivo
            
        Returns:
            True si inició la navegación, False en caso contrario
        """
        try:
            self.robot.navigator.goToPose(pose_stamped)
            self.robot.is_executing_goal = True
            
            pos = pose_stamped.pose.position
            self.robot.get_logger().info(f"Navegando a pose stamped: ({pos.x:.2f}, {pos.y:.2f})")
            return True
            
        except Exception as e:
            self.robot.get_logger().error(f"Error navegando a pose stamped: {e}")
            return False

    def rotate_improved(self, angular_speed: float = 0.5, duration: float = 3.0) -> bool:
        """
        Rota el robot en el lugar (versión mejorada y no bloqueante).
        
        Args:
            angular_speed: Velocidad angular en rad/s (positiva = izquierda)
            duration: Duración en segundos
            
        Returns:
            True si inició la rotación, False en caso contrario
        """
        try:
            msg = Twist()
            msg.angular.z = angular_speed
            
            # Publicar comando de rotación por la duración especificada
            self.robot.cmd_pub.publish(msg)
            
            # Programar parada después de la duración (usando timer)
            self.rotation_timer = self.robot.create_timer(duration, self.stop_rotation_callback)
            
            self.robot.get_logger().info(
                f"Iniciando rotación: {math.degrees(angular_speed * duration):.1f}° en {duration:.1f}s"
            )
            return True
            
        except Exception as e:
            self.robot.get_logger().error(f"Error en rotación: {e}")
            self.robot.stop()
            return False

    def stop_rotation_callback(self):
        """Callback para detener rotación automáticamente"""
        self.robot.stop()
        if self.rotation_timer:
            self.rotation_timer.destroy()
            self.rotation_timer = None


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