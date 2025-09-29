#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from nav2_msgs.srv import ClearEntireCostmap
import math
import time
from typing import Dict, Optional


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
        self.rotation_timer = None
        
        # Inicializar poses guardadas (el mapeo lo maneja otro componente)
        self.load_saved_poses()


    def velocity_callback(self, msg):
        """Callback para monitorear la velocidad actual del robot"""
        self.current_velocity = msg


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


    def rotate(self, angular_speed: float = 0.5, duration: float = 3.0) -> bool:
        """
        Rota el robot en el lugar de manera no bloqueante.
        
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
            self.cmd_pub.publish(msg)
            
            # Programar parada después de la duración (usando timer)
            self.rotation_timer = self.create_timer(duration, self.stop_rotation_callback)
            
            self.get_logger().info(
                f"Iniciando rotación: {math.degrees(angular_speed * duration):.1f}° en {duration:.1f}s"
            )

            return True
            
        except Exception as e:
            self.get_logger().error(f"Error en rotación: {e}")
            self.stop()
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

    
    def stop(self):
        """CFunción auxiliar para detener el robot"""
        msg = Twist()
        self.cmd_pub.publish(msg)
    

    def stop_rotation_callback(self):
        """Callback para detener rotación automáticamente"""
        self.stop()
        if self.rotation_timer:
            self.rotation_timer.destroy()
            self.rotation_timer = None


if __name__ == '__main__':
    print("RobotSkills - Navegación Básica")
    print("Para ejemplos completos, usar: navegation/demo/demo_skills_complete.py")