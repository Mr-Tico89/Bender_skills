#!/usr/bin/env python3
import rclpy
from nav2_msgs.srv import ClearEntireCostmap
import math
import os
from typing import Dict, Optional, List
from .basicNav import RobotSkills


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
        # Nota: rotate_improved ahora está integrado en robot.rotate()


    def clear_costmaps(self) -> bool:
        """
        Resetea los costmaps local y global del stack de navegación en paralelo.
        
        Returns:
            True si se limpiaron exitosamente, False en caso contrario
        """
        
        # Verificar que ambos servicios estén disponibles
        global_available = self.robot.clear_costmap_client.wait_for_service(timeout_sec=5.0)
        local_available = self.robot.clear_local_costmap_client.wait_for_service(timeout_sec=5.0)
        
        if not global_available:
            self.robot.get_logger().warn("Servicio de costmap global no disponible")

        if not local_available:
            self.robot.get_logger().warn("Servicio de costmap local no disponible")
            
        if not (global_available or local_available):
            return False
        
        # Iniciar ambas solicitudes en paralelo
        futures = []
        
        try:
            # Lanzar solicitud para costmap global
            if global_available:
                global_request = ClearEntireCostmap.Request()
                global_future = self.robot.clear_costmap_client.call_async(global_request)
                futures.append(('global', global_future))
            
            # Lanzar solicitud para costmap local
            if local_available:
                local_request = ClearEntireCostmap.Request()
                local_future = self.robot.clear_local_costmap_client.call_async(local_request)
                futures.append(('local', local_future))
            
            # Esperar que ambos terminen
            success = True
            for costmap_type, future in futures:
                rclpy.spin_until_future_complete(self.robot, future, timeout_sec=5.0)
                
                if future.result() is not None:
                    self.robot.get_logger().info(f"Costmap {costmap_type} limpiado")
                else:
                    self.robot.get_logger().warn(f"Error limpiando costmap {costmap_type}")
                    success = False
            
            return success
            
        except Exception as e:
            self.robot.get_logger().error(f"Error limpiando costmaps: {e}")
            return False
    

    def save_current_pose(self, name: str, description: str = "") -> bool:
        """
        Guarda la pose actual del robot con un nombre para uso posterior.
        
        Args:
            name: Nombre para identificar la pose guardada
            description: Descripción opcional de la pose
            
        Returns:
            True si se guardó exitosamente, False en caso contrario
        """
        import numpy as np
        import time

        current_pose = self.robot.where_am_i()
        if current_pose is None:
            return False
            
        pose_data = {
            "x": current_pose["x"],
            "y": current_pose["y"],
            "yaw": current_pose["yaw"],
            "description": description,
            "timestamp": time.time()
        }
        
        # Cargar poses existentes
        saved_poses = self.load_saved_poses()
        saved_poses[name] = pose_data
        
        # Guardar en archivo
        try:
            np.save(self.robot.saved_poses_file, saved_poses)
            
            self.robot.get_logger().info(f"Pose guardada como '{name}': {pose_data}")
            return True
        
        except Exception as e:
            self.robot.get_logger().error(f"Error guardando pose: {e}")
            return False
    

    def load_saved_poses(self) -> Dict:
        """Carga las poses guardadas desde archivo NPY"""
        import numpy as np
        
        try:
            if os.path.exists(self.robot.saved_poses_file):
                saved_poses = np.load(self.robot.saved_poses_file, allow_pickle=True).item()
                self.robot.get_logger().info(f"Poses guardadas cargadas desde {self.robot.saved_poses_file}")
                # Actualizar en robot también
                self.robot.saved_poses = saved_poses
                return saved_poses
            else:
                saved_poses = {}
                # Crear archivo vacío
                np.save(self.robot.saved_poses_file, saved_poses)
                self.robot.get_logger().info("Archivo de poses NPY creado (vacío)")
                self.robot.saved_poses = saved_poses
                return saved_poses
        except Exception as e:
            self.robot.get_logger().error(f"Error cargando poses: {e}")
            saved_poses = {}
            self.robot.saved_poses = saved_poses
            return saved_poses


    def list_available_locations(self) -> List[str]:
        """Retorna lista de ubicaciones disponibles"""
        saved_poses = self.load_saved_poses()
        return list(saved_poses.keys())


    def get_pose_info(self, location_name: str) -> Optional[Dict]:
        """Obtiene información detallada de una pose guardada"""
        saved_poses = self.load_saved_poses()
        return saved_poses.get(location_name)
 


def main():
    """Ejemplo de uso de navegación avanzada - gestión de costmaps"""
    rclpy.init()
    
    try:
        # Crear robot básico
        robot = RobotSkills()
        
        # Crear navegación avanzada
        advanced_nav = AdvancedNavigation(robot)
        
        print("Demo de Gestión Avanzada")
        
        # Esperar inicialización
        robot.navigator.waitUntilNav2Active()
        
        # Demo de gestión de poses
        print("\n1. Guardando poses:")
        advanced_nav.save_current_pose("punto_inicial", "Punto de partida del demo")
        
        # Navegar y guardar más poses
        robot.go_to_pose(1.0, 0.0, 0.0)
        robot.wait_for_result()
        advanced_nav.save_current_pose("punto_a", "Primera ubicación")
        
        robot.go_to_pose(0.0, 1.0, math.pi/2)
        robot.wait_for_result()
        advanced_nav.save_current_pose("punto_b", "Segunda ubicación")
        
        # Demo de listado de ubicaciones
        print("\n2. Ubicaciones disponibles:")
        locations = advanced_nav.list_available_locations()
        for loc in locations:
            info = advanced_nav.get_pose_info(loc)
            print(f"   {loc}: ({info['x']:.2f}, {info['y']:.2f}) - {info['description']}")
        
        # Demo de limpieza de costmaps
        print("\n3. Limpiando costmaps:")
        if advanced_nav.clear_costmaps():
            print("   Costmaps limpiados exitosamente")
        else:
            print("   Error limpiando costmaps")
        
        print("\nDemo de gestión avanzada completado!")
        
    except KeyboardInterrupt:
        print("\nInterrumpido por el usuario")
        
    finally:
        if 'robot' in locals():
            robot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()