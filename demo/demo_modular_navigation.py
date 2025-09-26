#!/usr/bin/env python3

"""
Ejemplo de uso del sistema modular de navegación.
Demuestra cómo usar RobotSkills (básico) + SemanticNavigation (semántico) por separado.
"""

import rclpy
import math
from skills import RobotSkills, SemanticNavigation


def demo_basic_navigation():
    """Demo usando solo las skills básicas"""
    print("\n=== DEMO NAVEGACIÓN BÁSICA ===")
    
    robot = RobotSkills()
    
    # 1. Funciones básicas
    print("1. Posición actual:")
    pose = robot.where_am_i()
    if pose:
        print(f"   Posición: ({pose['x']:.2f}, {pose['y']:.2f}) - {pose['yaw_degrees']:.1f}°")
    
    print("\n2. Navegación básica a coordenadas:")
    robot.go_to_pose(1.0, 1.0, math.pi/4)
    robot.wait_for_result(timeout=20.0)
    
    print("\n3. Guardando esta ubicación:")
    robot.save_current_pose("punto_demo", "Ubicación del demo básico")
    
    print("\n4. Rotación básica:")
    robot.rotate(angular_speed=0.5, duration=2.0)
    
    return robot


def demo_semantic_navigation(robot):
    """Demo usando navegación semántica"""
    print("\n=== DEMO NAVEGACIÓN SEMÁNTICA ===")
    
    # Crear instancia de navegación semántica
    semantic = SemanticNavigation(robot)
    
    print("1. Creando algunas ubicaciones de ejemplo...")
    
    # Crear ubicaciones de ejemplo
    locations_to_create = [
        ("base", "Centro de operaciones", 0.0, 0.0, 0.0),
        ("punto_a", "Primer punto de interés", 2.0, 0.0, math.pi/2),
        ("punto_b", "Segundo punto de interés", 2.0, 2.0, math.pi),
        ("punto_c", "Tercer punto de interés", 0.0, 2.0, -math.pi/2)
    ]
    
    for name, description, x, y, yaw in locations_to_create:
        robot.go_to_pose(x, y, yaw)
        robot.wait_for_result(timeout=15.0)
        robot.save_current_pose(name, description)
        print(f"   Creado: {name}")
    
    print(f"\n2. Ubicaciones disponibles: {semantic.get_available_locations()}")
    
    print("\n3. Navegación semántica:")
    semantic.go("base")
    robot.wait_for_result()
    print("   Fui a 'base'")
    
    print("\n4. Approach - acercarse manteniendo distancia:")
    semantic.approach("punto_a", approach_distance=0.8)
    robot.wait_for_result()
    print("   Me acerqué a 'punto_a' manteniendo 0.8m")
    
    print("\n5. Look - mirar hacia ubicación:")
    semantic.look("punto_b")
    print("   Mirando hacia 'punto_b'")
    
    print("\n6. Función combinada - ir y mirar:")
    semantic.go_and_look("punto_c", "base")
    print("   Fui a 'punto_c' y miré hacia 'base'")
    
    print("\n7. Patrullaje automático:")
    patrol_points = ["base", "punto_a", "punto_b", "punto_c"]
    semantic.patrol(patrol_points, cycles=2)
    print("   Patrullaje completado")
    
    print("\n8. Ruta personalizada:")
    route = ["punto_c", "punto_b", "punto_a", "base"]
    semantic.create_route(route, name="ruta_circular")
    print("   Ruta circular completada")
    
    return semantic


def demo_advanced_features(robot, semantic):
    """Demo de funciones avanzadas combinadas"""
    print("\n=== DEMO FUNCIONES AVANZADAS ===")
    
    print("1. Encontrar ubicación más cercana:")
    nearest = semantic.find_nearest_location()
    if nearest:
        distance = semantic.get_distance_to_location(nearest)
        print(f"   Más cercana: '{nearest}' a {distance:.2f}m")
    
    print("\n2. Distancias a todas las ubicaciones:")
    for location in semantic.get_available_locations():
        distance = semantic.get_distance_to_location(location)
        if distance:
            print(f"   {location}: {distance:.2f}m")
    
    print("\n3. Limpiar mapas de navegación:")
    robot.clear_costmaps()
    
    print("\n4. Look to pose específica:")
    robot.look_to_pose(0.0, 0.0)  # Mirar al origen
    
    print("\n5. Navegación con PoseStamped:")
    from geometry_msgs.msg import PoseStamped
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = robot.navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 1.0
    goal_pose.pose.position.y = 1.0
    goal_pose.pose.position.z = 0.0
    
    robot.go_to_pose_stamped(goal_pose)
    robot.wait_for_result()
    print("   Navegación con PoseStamped completada")


def main():
    """Función principal del demo modular"""
    rclpy.init()
    
    try:
        print("Demo del Sistema Modular de Navegación")
        print("=====================================")
        
        # Demo paso a paso
        robot = demo_basic_navigation()
        
        input("\nPresiona ENTER para continuar con navegación semántica...")
        semantic = demo_semantic_navigation(robot)
        
        input("\nPresiona ENTER para continuar con funciones avanzadas...")
        demo_advanced_features(robot, semantic)
        
        print("\nDemo modular completado!")
        
        # Resumen del sistema modular
        print("\nArquitectura del Sistema:")
        print("├── RobotSkills (skills.py)")
        print("│   ├── where_am_i() - Posición actual")
        print("│   ├── go_to_pose() - Navegación básica")
        print("│   ├── save_current_pose() - Guardar ubicaciones")
        print("│   ├── is_moving() - Detectar movimiento")
        print("│   ├── reached() - Verificar llegada")
        print("│   ├── cancel() - Cancelar navegación")
        print("│   ├── rotate() - Rotación básica")
        print("│   ├── clear_costmaps() - Limpiar mapas")
        print("│   └── wait_for_result() - Espera bloqueante")
        print("│")
        print("└── SemanticNavigation (semantic_navigation.py)")
        print("    ├── go() - Navegación por nombre")
        print("    ├── approach() - Acercarse con distancia")
        print("    ├── look() - Mirar hacia ubicación")
        print("    ├── patrol() - Patrullaje automático")
        print("    ├── create_route() - Rutas personalizadas")
        print("    └── Funciones de utilidad (distancias, etc.)")
        
        print("\nVentajas del sistema modular:")
        print("   - Separación clara de responsabilidades")
        print("   - RobotSkills maneja navegación básica")
        print("   - SemanticNavigation maneja lógica compleja") 
        print("   - Fácil de mantener y extender")
        print("   - Reutilizable en diferentes contextos")
        
    except KeyboardInterrupt:
        print("\nDemo interrumpido por el usuario")
        
    except Exception as e:
        print(f"\nError en el demo: {e}")
        
    finally:
        if 'robot' in locals():
            robot.cancel()
            robot.destroy_node()
        rclpy.shutdown()
        print("\nDemo modular finalizado")


if __name__ == '__main__':
    main()