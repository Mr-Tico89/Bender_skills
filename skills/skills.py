import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from nav2_msgs.srv import ClearEntireCostmap
import math
import json
import numpy as np
import os
import time
from typing import Dict, Optional, List

class RobotSkills(Node):
    def __init__(self):
        super().__init__('robot_skills')

        # API de navegación de Nav2
        self.navigator = BasicNavigator()

        # Publisher directo a cmd_vel (para rotar manualmente)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber para obtener velocidad actual
        self.velocity_sub = self.create_subscription(
            Twist, '/cmd_vel', self.velocity_callback, 10
        )

        # Cliente de servicio para limpiar costmaps
        self.clear_costmap_client = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap'
        )

        self.clear_local_costmap_client = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap'
        )

        self.goal_handle = None
        self.is_executing_goal = False
        self.current_velocity = Twist()
        self.saved_poses_file = "saved_poses.npy"
        
        # Inicializar poses guardadas (el mapeo lo maneja otro componente)
        self.load_saved_poses()

    def velocity_callback(self, msg):
        """Callback para monitorear la velocidad actual del robot"""
        self.current_velocity = msg


    # ===== FUNCIONES BÁSICAS MEJORADAS =====

    def where_am_i(self) -> Optional[Dict]:
        """
        Obtiene la posición actual del robot.
        
        Returns:
            Dict con x, y, yaw si es exitoso, None si hay error
        """
        pose = self.navigator.getCurrentPose()

        if pose:
            x = pose.pose.position.x
            y = pose.pose.position.y
            
            # Convertir quaternion a euler para obtener yaw
            orientation = pose.pose.orientation
            _, _, yaw = euler_from_quaternion([
                orientation.x, orientation.y, orientation.z, orientation.w
            ])
            
            result = {
                "x": x,
                "y": y, 
                "yaw": yaw,
                "yaw_degrees": math.degrees(yaw)
            }
            
            self.get_logger().info(
                f"Posición actual: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f}°"
            )

            return result
        
        else:
            self.get_logger().warn("No se pudo obtener la pose actual")
            return None


    
    def go_to_pose(self, x=None, y=None, yaw=0.0, pose_stamped=None, frame_id='map'):
        """
        Navega a una pose usando coordenadas individuales O un PoseStamped completo.
        
        Args:
            x, y, yaw: Coordenadas individuales (si no se usa pose_stamped)
            pose_stamped: PoseStamped completo (alternativa a x,y,yaw)
            frame_id: Marco de referencia ('map', 'odom', 'base_link')
            
        Returns:
            True si inició la navegación, False en caso contrario
        """
        try:
            # Caso 1: Se proporciona un PoseStamped completo
            if pose_stamped is not None:
                self.navigator.goToPose(pose_stamped)
                pos = pose_stamped.pose.position
                self.get_logger().info(
                    f"Navegando a pose stamped: ({pos.x:.2f}, {pos.y:.2f}) "
                    f"en frame '{pose_stamped.header.frame_id}'"
                )
                
            # Caso 2: Se proporcionan coordenadas individuales
            elif x is not None and y is not None:
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = frame_id
                goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()

                goal_pose.pose.position.x = float(x)
                goal_pose.pose.position.y = float(y)
                goal_pose.pose.position.z = 0.0

                q = quaternion_from_euler(0, 0, yaw)
                goal_pose.pose.orientation.x = q[0]
                goal_pose.pose.orientation.y = q[1]
                goal_pose.pose.orientation.z = q[2]
                goal_pose.pose.orientation.w = q[3]

                self.navigator.goToPose(goal_pose)
                self.get_logger().info(
                    f"Navegando hacia: ({x:.2f}, {y:.2f}) con yaw={yaw:.2f} "
                    f"en frame '{frame_id}'"
                )
            else:
                self.get_logger().error("Debe proporcionar coordenadas (x,y) O un pose_stamped")
                return False
                
            self.is_executing_goal = True
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error en navegación: {e}")
            return False


    # Cancelar navegación (versión mejorada)
    def cancel(self) -> bool:
        """
        Cancela el último objetivo de navegación entregado.
        
        Returns:
            True si se canceló exitosamente, False en caso contrario
        """
        try:
            if self.is_executing_goal:
                self.navigator.cancelTask()
                self.is_executing_goal = False
                self.get_logger().info("Navegación cancelada")
            
            # También enviar comando de parada por seguridad
            self.stop()
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error cancelando navegación: {e}")
            return False


    # ¿Se está moviendo?
    def is_moving(self, linear_threshold: float = 0.01, angular_threshold: float = 0.01) -> bool:
        """
        Determina si el robot se está moviendo actualmente.
        
        Args:
            linear_threshold: Umbral mínimo de velocidad linear (m/s)
            angular_threshold: Umbral mínimo de velocidad angular (rad/s)
            
        Returns:
            True si se está moviendo, False si está parado
        """

        linear_speed = abs(self.current_velocity.linear.x)
        angular_speed = abs(self.current_velocity.angular.z)
        is_moving = (linear_speed > linear_threshold or angular_speed > angular_threshold)
        
        # También verificar si hay una tarea de navegación en curso
        nav_in_progress = self.is_executing_goal and not self.navigator.isTaskComplete()
        
        return is_moving or nav_in_progress


    # ¿Llegó a la meta?
    def reached(self, target_x: float = None, target_y: float = None, 
                position_tolerance: float = 0.3, angle_tolerance: float = 0.2) -> bool:
        """
        Verifica si el robot llegó a la pose objetivo.
        
        Args:
            target_x, target_y: Coordenadas objetivo (opcional, usa la última meta si no se especifica)
            position_tolerance: Tolerancia en posición (metros)
            angle_tolerance: Tolerancia en ángulo (radianes)
            
        Returns:
            True si llegó al objetivo, False en caso contrario
        """

        # Si hay una tarea de navegación en curso, verificar su estado
        if self.is_executing_goal:
            if self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                self.is_executing_goal = False
                
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info("Meta alcanzada por Nav2")
                    return True
                
                else:
                    self.get_logger().warn("No se alcanzó la meta según Nav2")
                    return False
            else:
                # Tarea aún en progreso
                return False
        
        # Si se proporcionaron coordenadas específicas, verificar distancia
        if target_x is not None and target_y is not None:
            current_pose = self.where_am_i()
            if current_pose is None:
                return False
                
            distance = math.sqrt(
                (current_pose["x"] - target_x)**2 + (current_pose["y"] - target_y)**2
            )
            
            return distance <= position_tolerance
            
        # No hay tarea en curso ni target específico
        return True


    # Rotar en el lugar
    def rotate(self, angular_speed=0.5, duration=3.0):
        msg = Twist()
        msg.angular.z = angular_speed
        self.cmd_pub.publish(msg)

        # Esperar durante la duración especificada
        time.sleep(duration)
        self.stop()
        self.get_logger().info(f"Rotación completada durante {duration} s")


    # Función auxiliar para detener el robot
    def stop(self):
        msg = Twist()
        self.cmd_pub.publish(msg)


    # ===== NUEVAS FUNCIONES DE NAVEGACIÓN AVANZADA =====

    def save_current_pose(self, name: str, description: str = "") -> bool:
        """
        Guarda la pose actual del robot con un nombre para uso posterior.
        
        Args:
            name: Nombre para identificar la pose guardada
            description: Descripción opcional de la pose
            
        Returns:
            True si se guardó exitosamente, False en caso contrario
        """

        current_pose = self.where_am_i()
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
            np.save(self.saved_poses_file, saved_poses)
            
            self.get_logger().info(f"Pose guardada como '{name}': {pose_data}")
            return True
        
        except Exception as e:
            self.get_logger().error(f"Error guardando pose: {e}")
            return False

    # Nota: is_in_room() e is_in_map() son manejadas por el componente de mapeo


    def clear_costmaps(self) -> bool:
        """
        Resetea los costmaps local y global del stack de navegación en paralelo.
        
        Returns:
            True si se limpiaron exitosamente, False en caso contrario
        """
        
        # Verificar que ambos servicios estén disponibles
        global_available = self.clear_costmap_client.wait_for_service(timeout_sec=5.0)
        local_available = self.clear_local_costmap_client.wait_for_service(timeout_sec=5.0)
        
        if not global_available:
            self.get_logger().warn("Servicio de costmap global no disponible")

        if not local_available:
            self.get_logger().warn("Servicio de costmap local no disponible")
            
        if not (global_available or local_available):
            return False
        
        # Iniciar ambas solicitudes en paralelo
        futures = []
        
        try:
            # Lanzar solicitud para costmap global
            if global_available:
                global_request = ClearEntireCostmap.Request()
                global_future = self.clear_costmap_client.call_async(global_request)
                futures.append(('global', global_future))
            
            # Lanzar solicitud para costmap local
            if local_available:
                local_request = ClearEntireCostmap.Request()
                local_future = self.clear_local_costmap_client.call_async(local_request)
                futures.append(('local', local_future))
            
            # Esperar que ambos terminen
            success = True
            for costmap_type, future in futures:
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                
                if future.result() is not None:
                    self.get_logger().info(f"Costmap {costmap_type} limpiado")
                else:
                    self.get_logger().warn(f"Error limpiando costmap {costmap_type}")
                    success = False
            
            return success
            
        except Exception as e:
            self.get_logger().error(f"Error limpiando costmaps: {e}")
            return False


    def wait_for_result(self, timeout: float = 60.0) -> bool:
        """
        Espera de manera bloqueante que el robot complete la tarea actual.
        
        Args:
            timeout: Tiempo máximo a esperar en segundos
            
        Returns:
            True si completó exitosamente, False si falló o timeout
        """

        if not self.is_executing_goal:
            self.get_logger().warn("No hay ninguna tarea en ejecución para esperar")
            return True
            
        self.get_logger().info(f"Esperando resultado de la tarea (timeout: {timeout}s)")
        
        start_time = time.time()
        
        while rclpy.ok() and time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            
            if self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                self.is_executing_goal = False
                
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info("Tarea completada exitosamente")
                    return True
                
                else:
                    self.get_logger().warn(f"Tarea falló con resultado: {result}")
                    return False
                    
        # Timeout
        self.get_logger().warn("Timeout esperando resultado de la tarea")
        self.cancel()
        return False


    # ===== FUNCIONES AUXILIARES =====


    def load_saved_poses(self) -> Dict:
        """Carga las poses guardadas desde archivo"""
        try:
            if os.path.exists(self.saved_poses_file):
                self.saved_poses = np.load(self.saved_poses_file, allow_pickle=True).item()
                self.get_logger().info(f"Poses guardadas cargadas desde {self.saved_poses_file}")
            else:
                self.saved_poses = {}
                # Crear archivo vacío
                np.save(self.saved_poses_file, self.saved_poses)
                self.get_logger().info("Archivo de poses creado (vacío)")
        except Exception as e:
            self.get_logger().error(f"Error cargando poses: {e}")
            self.saved_poses = {}
            
        return self.saved_poses


    def list_available_locations(self) -> List[str]:
        """Retorna lista de ubicaciones disponibles"""
        return list(self.saved_poses.keys())


    def get_pose_info(self, location_name: str) -> Optional[Dict]:
        """Obtiene información detallada de una pose guardada"""
        return self.saved_poses.get(location_name)

def main():
    """Ejemplo básico de uso de las skills"""
    rclpy.init()
    
    try:
        robot = RobotSkills()
        print("Robot Skills iniciado!")
        
        # Ejemplo básico
        print("\n ¿Dónde estoy?")
        current_pose = robot.where_am_i()
        
        if current_pose:
            print(f" Posición: ({current_pose['x']:.2f}, {current_pose['y']:.2f})")
            
            print("\n Guardando posición actual...")
            robot.save_current_pose("inicio", "Punto de inicio")
            
            print("\n Navegando a (2.0, 1.0)...")
            robot.go_to_pose(2.0, 1.0, yaw=math.pi/2)
            
            print("\n Esperando a llegar...")
            while robot.is_moving() and rclpy.ok():
                rclpy.spin_once(robot, timeout_sec=0.5)
                
            if robot.reached():
                print(" ¡Llegué!")
                robot.rotate(angular_speed=0.5, duration=2.0)
                
        print("\n Ejemplo completado!")
        
    except KeyboardInterrupt:
        print("\n Interrumpido por el usuario")
        
    finally:
        if 'robot' in locals():
            robot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()