#!/usr/bin/env python3

import time
import math
import rclpy
from typing import Dict, Any
from geometry_msgs.msg import Twist
from .base_skill import BaseSkill


class SimpleMoveSkill(BaseSkill):
    """
    Skill 1: Mover hacia adelante
    Lo más básico - solo avanzar una distancia
    """
    
    def __init__(self, skill_name: str = "simple_move"):
        super().__init__(skill_name)
        
        # Publisher para enviar comandos de velocidad
        self.velocity_publisher = self.create_publisher(
            Twist, '/cmd_vel', 10
        )
        
        # Velocidad fija para simplicidad
        self.default_speed = 0.2  # m/s
        
        self.get_logger().info("SimpleMoveSkill listo - mover hacia adelante")
    
    def execute(self, parameters: Dict[str, Any] = None) -> bool:
        """Mover hacia adelante una distancia determinada."""
        
        if not parameters or 'distance' not in parameters:
            self.get_logger().error("Necesito 'distance' en metros")
            return False
        
        distance = parameters['distance']
        speed = parameters.get('speed', self.default_speed)
        
        if distance <= 0:
            self.get_logger().warn("Distancia debe ser positiva")
            return False
        
        # Calcular tiempo necesario
        duration = distance / speed
        
        self.get_logger().info(f"🚶 Moviendo {distance}m a {speed}m/s por {duration:.1f}s")
        
        # Crear mensaje de velocidad
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = 0.0
        
        # Enviar comandos mientras nos movemos
        start_time = time.time()
        rate = self.create_rate(10)  # 10 Hz
        
        while (time.time() - start_time) < duration:
            self.velocity_publisher.publish(twist)
            rate.sleep()
        
        # Parar al final
        twist.linear.x = 0.0
        self.velocity_publisher.publish(twist)
        
        self.result = {
            'distance_moved': distance,
            'time_taken': duration,
            'final_speed': 0.0
        }
        
        self.get_logger().info(f"Movimiento completado: {distance}m")
        return True
    
    def stop(self) -> bool:
        """Parar inmediatamente."""
        twist = Twist()  # Todas las velocidades en 0
        self.velocity_publisher.publish(twist)
        self.get_logger().info("Movimiento detenido")
        return True


class SimpleRotateSkill(BaseSkill):
    """
    Skill 2: Girar izquierda o derecha
    Ángulos positivos = izquierda, negativos = derecha
    """
    
    def __init__(self, skill_name: str = "simple_rotate"):
        super().__init__(skill_name)
        
        # Publisher para comandos de velocidad
        self.velocity_publisher = self.create_publisher(
            Twist, '/cmd_vel', 10
        )
        
        # Velocidad angular fija para simplicidad
        self.default_angular_speed = 0.5  # rad/s
        
        self.get_logger().info("SimpleRotateSkill listo - girar izq/der")
    
    def execute(self, parameters: Dict[str, Any] = None) -> bool:
        """Girar un ángulo determinado."""
        
        if not parameters or 'angle_degrees' not in parameters:
            self.get_logger().error("Necesito 'angle_degrees'")
            return False
        
        angle_degrees = parameters['angle_degrees']
        angular_speed = parameters.get('angular_speed', self.default_angular_speed)
        
        # Convertir a radianes
        angle_radians = math.radians(abs(angle_degrees))
        
        # Determinar dirección
        direction = 1.0 if angle_degrees > 0 else -1.0
        direction_text = "izquierda ↺" if angle_degrees > 0 else "derecha ↻"
        
        # Calcular tiempo necesario
        duration = angle_radians / angular_speed
        
        self.get_logger().info(
            f"Girando {abs(angle_degrees)}° hacia la {direction_text} "
            f"por {duration:.1f}s"
        )
        
        # Crear mensaje de velocidad
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = direction * angular_speed
        
        # Enviar comandos mientras giramos
        start_time = time.time()
        rate = self.create_rate(10)  # 10 Hz
        
        while (time.time() - start_time) < duration:
            self.velocity_publisher.publish(twist)
            rate.sleep()
        
        # Parar al final
        twist.angular.z = 0.0
        self.velocity_publisher.publish(twist)
        
        self.result = {
            'angle_rotated_degrees': angle_degrees,
            'direction': direction_text,
            'time_taken': duration
        }
        
        self.get_logger().info(f"Giro completado: {angle_degrees}°")
        return True
    
    def stop(self) -> bool:
        """Parar giro inmediatamente."""
        twist = Twist()
        self.velocity_publisher.publish(twist)
        self.get_logger().info("Giro detenido")
        return True


class SimpleStopSkill(BaseSkill):
    """
    Skill 3: Parar inmediatamente
    Para emergencias o fin de secuencias
    """
    
    def __init__(self, skill_name: str = "simple_stop"):
        super().__init__(skill_name)
        
        # Publisher para comandos de velocidad
        self.velocity_publisher = self.create_publisher(
            Twist, '/cmd_vel', 10
        )
        
        self.get_logger().info("SimpleStopSkill listo - parar ahora")
    
    def execute(self, parameters: Dict[str, Any] = None) -> bool:
        """Parar todo movimiento inmediatamente."""
        
        # Crear mensaje de velocidad cero
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        
        # Enviar varias veces para asegurar
        for _ in range(5):
            self.velocity_publisher.publish(twist)
            time.sleep(0.1)
        
        self.result = {
            'status': 'stopped',
            'all_velocities': 'zero'
        }
        
        self.get_logger().info("ROBOT DETENIDO - Todas las velocidades en cero")
        return True
    
    def stop(self) -> bool:
        """Ya estamos parados, pero por si acaso..."""
        return self.execute()


class SimpleWaitSkill(BaseSkill):
    """
    Skill 4: Esperar un tiempo determinado
    Para pausas entre acciones
    """
    
    def __init__(self, skill_name: str = "simple_wait"):
        super().__init__(skill_name)
        self.get_logger().info("SimpleWaitSkill listo - esperar tiempo")
    
    def execute(self, parameters: Dict[str, Any] = None) -> bool:
        """Esperar un número de segundos."""
        
        if not parameters or 'seconds' not in parameters:
            self.get_logger().error(" Necesito 'seconds'")
            return False
        
        wait_time = parameters['seconds']
        
        if wait_time <= 0:
            self.get_logger().warn(" Tiempo debe ser positivo")
            return False
        
        self.get_logger().info(f" Esperando {wait_time} segundos...")
        
        # Esperar mostrando progreso
        start_time = time.time()
        while (time.time() - start_time) < wait_time:
            elapsed = time.time() - start_time
            remaining = wait_time - elapsed
            
            if remaining > 1.0:  # Solo mostrar si queda más de 1 segundo
                if int(elapsed) % 1 == 0:  # Cada segundo
                    self.get_logger().info(f"Faltan {remaining:.0f} segundos...")
            
            time.sleep(0.1)
        
        self.result = {
            'waited_seconds': wait_time,
            'actual_time': time.time() - start_time
        }
        
        self.get_logger().info(f"Espera completada: {wait_time}s")
        return True
    
    def stop(self) -> bool:
        """No se puede 'parar' una espera, solo terminar."""
        self.get_logger().info("Espera interrumpida")
        return True


# Funciones de conveniencia para usar desde scripts
def move_forward(distance: float, speed: float = 0.2) -> bool:
    """Función simple para mover hacia adelante."""
    skill = SimpleMoveSkill()
    result = skill.execute({'distance': distance, 'speed': speed})
    skill.destroy_node()
    return result

def rotate_degrees(angle: float, angular_speed: float = 0.5) -> bool:
    """Función simple para girar."""
    skill = SimpleRotateSkill()
    result = skill.execute({'angle_degrees': angle, 'angular_speed': angular_speed})
    skill.destroy_node()
    return result

def stop_robot() -> bool:
    """Función simple para parar."""
    skill = SimpleStopSkill()
    result = skill.execute()
    skill.destroy_node()
    return result

def wait_seconds(seconds: float) -> bool:
    """Función simple para esperar."""
    skill = SimpleWaitSkill()
    result = skill.execute({'seconds': seconds})
    skill.destroy_node()
    return result