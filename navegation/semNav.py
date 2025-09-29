#!/usr/bin/env python3
import rclpy
import math
from .basicNav import RobotSkills


class SemanticNavigation:
    """
    Clase para manejar navegación semántica del robot.
    Permite navegar usando nombres de ubicaciones en lugar de coordenadas.
    """
    
    def __init__(self, robot_skills: RobotSkills):
        """
        Inicializa la navegación semántica.
        
        Args:
            robot_skills: Instancia de RobotSkills para usar las funciones básicas
        """
        self.robot = robot_skills
        

    def go(self, location_name: str) -> bool:
        """
        Navegación semántica - ir a una ubicación por nombre.
        
        Args:
            location_name: Nombre de la ubicación (ej: "cocina", "salon")
            
        Returns:
            True si inició la navegación, False en caso contrario
        """
        if location_name in self.robot.saved_poses:
            pose_data = self.robot.saved_poses[location_name]
            success = self.robot.go_to_pose(pose_data["x"], pose_data["y"], pose_data["yaw"])
            
            if success:
                self.robot.get_logger().info(f"Navegando semánticamente a '{location_name}'")
            
            return success
        else:
            self.robot.get_logger().warn(f"Ubicación '{location_name}' no encontrada en poses guardadas")
            return False


    def approach(self, location_name: str, approach_distance: float = 1.0) -> bool:
        """
        Se acerca a una ubicación manteniendo cierta distancia.
        
        Args:
            location_name: Nombre de la ubicación objetivo
            approach_distance: Distancia a mantener del objetivo (metros)
            
        Returns:
            True si inició el approach, False en caso contrario
        """
        if location_name not in self.robot.saved_poses:
            self.robot.get_logger().warn(f"Ubicación '{location_name}' no encontrada")
            return False
            
        target_pose = self.robot.saved_poses[location_name]
        current_pose = self.robot.where_am_i()
        
        if current_pose is None:
            return False
            
        # Calcular punto de approach
        dx = target_pose["x"] - current_pose["x"]
        dy = target_pose["y"] - current_pose["y"]
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance <= approach_distance:
            self.robot.get_logger().info(f"Ya estoy cerca de '{location_name}' (distancia actual: {distance:.2f}m)")
            return True
            
        # Calcular posición de approach
        ratio = (distance - approach_distance) / distance
        approach_x = current_pose["x"] + dx * ratio
        approach_y = current_pose["y"] + dy * ratio
        
        # Orientarse hacia el objetivo
        approach_yaw = math.atan2(dy, dx)
        
        self.robot.get_logger().info(
            f"Acercándose a '{location_name}' manteniendo {approach_distance}m de distancia"
        )
        return self.robot.go_to_pose(approach_x, approach_y, approach_yaw)


    def look(self, location_name: str) -> bool:
        """
        Mira hacia una habitación o ubicación específica.
        
        Args:
            location_name: Nombre de la ubicación hacia la cual mirar
            
        Returns:
            True si inició la rotación, False en caso contrario
        """
        if location_name not in self.robot.saved_poses:
            self.robot.get_logger().warn(f"Ubicación '{location_name}' no encontrada")
            return False
            
        target_pose = self.robot.saved_poses[location_name]
        return self.look_to_pose(target_pose["x"], target_pose["y"])


    def look_to_pose(self, target_x: float, target_y: float, angular_speed: float = 0.5) -> bool:
        """
        Rota para mirar hacia una pose específica.
        
        Args:
            target_x, target_y: Coordenadas del punto a mirar
            angular_speed: Velocidad angular en rad/s
            
        Returns:
            True si inició la rotación, False en caso contrario
        """
        current_pose = self.robot.where_am_i()
        if current_pose is None:
            return False
            
        # Calcular ángulo hacia el objetivo
        dx = target_x - current_pose["x"]
        dy = target_y - current_pose["y"]
        target_yaw = math.atan2(dy, dx)
        
        # Calcular diferencia angular
        current_yaw = current_pose["yaw"]
        angle_diff = target_yaw - current_yaw
        
        # Normalizar ángulo a [-π, π]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
            
        # Determinar dirección de rotación
        rotation_speed = angular_speed if angle_diff > 0 else -angular_speed
        
        # Calcular duración de rotación
        duration = abs(angle_diff) / angular_speed
        
        self.robot.get_logger().info(
            f"Rotando {math.degrees(angle_diff):.1f}° hacia ({target_x:.2f}, {target_y:.2f})"
        )
        
        # Usar la rotación mejorada del robot
        return self.robot.rotate_improved(rotation_speed, duration)


    def get_available_locations(self) -> list:
        """Retorna lista de ubicaciones disponibles para navegación semántica"""
        return self.robot.list_available_locations()


def main():
    """Ejemplo de uso de navegación semántica"""
    rclpy.init()
    
    try:
        # Crear robot básico
        robot = RobotSkills()
        
        # Crear navegación semántica
        semantic_nav = SemanticNavigation(robot)
        
        print("Demo de Navegación Semántica")
        
        # Guardar algunas ubicaciones de ejemplo
        robot.save_current_pose("base", "Punto base del robot")
        
        # Navegar a coordenadas y guardar
        robot.go_to_pose(2.0, 0.0, 0.0)
        robot.wait_for_result()
        robot.save_current_pose("punto_a", "Primer punto de interés")
        
        robot.go_to_pose(0.0, 2.0, math.pi/2)
        robot.wait_for_result()
        robot.save_current_pose("punto_b", "Segundo punto de interés")
        
        # Demo de navegación semántica
        print("\n1. Ubicaciones disponibles:", semantic_nav.get_available_locations())
        
        print("\n2. Navegación semántica:")
        semantic_nav.go("base")
        semantic_nav.robot.wait_for_result()
        
        print("\n3. Approach con distancia:")
        semantic_nav.approach("punto_a", approach_distance=0.8)
        semantic_nav.robot.wait_for_result()
        
        print("\n4. Mirar hacia ubicación:")
        semantic_nav.look("punto_b")
        
        print("\n5. Patrullaje:")
        semantic_nav.patrol(["base", "punto_a", "punto_b"], cycles=2)
        
        print("\nDemo de navegación semántica completado!")
        
    except KeyboardInterrupt:
        print("\nInterrumpido por el usuario")
        
    finally:
        if 'robot' in locals():
            robot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()