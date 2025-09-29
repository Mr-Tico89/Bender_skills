#!/usr/bin/env python3
"""
Demo completo del sistema modular de navegación robótica
Muestra el uso de RobotSkills, SemanticNavigation y AdvancedNavigation por separado
"""

import rclpy
import math
import time
from navegation import RobotSkills, SemanticNavigation, AdvancedNavigation


def demo_basic_navigation(robot):
    """Demo de funciones básicas de navegación"""
    print("\n=== DEMO SKILLS BÁSICAS ===")
    
    # Obtener posición actual
    pose_info = robot.where_am_i()
    if pose_info:
        print(f"   Posición: ({pose_info['x']:.2f}, {pose_info['y']:.2f})")
        print(f"   Orientación: {pose_info['yaw_degrees']:.1f}°")
    
    # Guardar posición actual
    if robot.save_current_pose("demo_start", "Punto de inicio del demo"):
        print("   Posición guardada exitosamente")
    
    print("   Navegando a coordenadas (1.5, 0.5, 45°)...")
    if robot.go_to_pose(1.5, 0.5, math.radians(45)):
        print("   Comando de navegación enviado")
        
        # Esperar resultado
        if robot.wait_for_result():
            print("   Navegación completada!")
            
            # Verificar llegada
            if robot.reached(1.5, 0.5, tolerance=0.2):
                print("   Confirmado: llegué al destino")
        else:
            print("   Navegación falló o cancelada")
    
    # Demo de rotación
    print("   Realizando rotación...")
    robot.rotate(angular_speed=0.5, duration=3.14)
    print("   Rotación de ~90° completada")
    
    time.sleep(1)
    robot.stop()


def demo_semantic_navigation(robot, semantic_nav):
    """Demo de navegación semántica"""
    print("\n=== DEMO NAVEGACIÓN SEMÁNTICA ===")
    
    # Primero crear algunas ubicaciones para el demo
    if len(robot.saved_poses) < 2:
        print("   Agregando algunas ubicaciones de ejemplo...")
        
        # Navegar y guardar varios puntos
        robot.go_to_pose(1.0, 0.0, 0.0)
        robot.wait_for_result()
        robot.save_current_pose("punto_a", "Primer punto de referencia")
        
        robot.go_to_pose(0.0, 1.0, math.pi/2)
        robot.wait_for_result()
        robot.save_current_pose("punto_b", "Segundo punto de referencia")
        
        robot.go_to_pose(-1.0, 0.0, math.pi)
        robot.wait_for_result()
        robot.save_current_pose("punto_c", "Tercer punto de referencia")
        
        ubicaciones = semantic_nav.get_available_locations()
        print(f"   Ubicaciones creadas: {ubicaciones}")
    
    # Demo de navegación semántica
    ubicaciones = semantic_nav.get_available_locations()
    if len(ubicaciones) >= 2:
        primera_ubicacion = ubicaciones[0]
        print(f"   Navegando semánticamente a '{primera_ubicacion}'...")
        if semantic_nav.go(primera_ubicacion):
            robot.wait_for_result()
            print(f"   Llegué a '{primera_ubicacion}'")
        
        # Demo de approach
        if len(ubicaciones) >= 2:
            segunda_ubicacion = ubicaciones[1]
            print(f"   Acercándome a '{segunda_ubicacion}' (manteniendo distancia)...")
            if semantic_nav.approach(segunda_ubicacion, approach_distance=0.8):
                robot.wait_for_result()
                print(f"   Me acerqué a '{segunda_ubicacion}' (0.8m de distancia)")
    else:
        print("   Necesito al menos 2 ubicaciones para demo completo")


def demo_advanced_navigation(robot, advanced_nav):
    """Demo de funciones avanzadas de navegación"""
    print("\n=== DEMO NAVEGACIÓN AVANZADA ===")
    
    # Demo de limpiar costmaps
    print("   Limpiando costmaps...")
    if robot.clear_costmaps():
        print("   Costmaps limpiados")
    else:
        print("   Algunos costmaps no se pudieron limpiar")
    
    # Demo de look_to_pose
    print("   Mirando hacia punto (2.0, 2.0)...")
    if advanced_nav.look_to_pose(2.0, 2.0):
        time.sleep(2)  # Esperar rotación
        print("   Rotación hacia objetivo completada")
    
    # Demo de navegación por waypoints
    print("   Navegación por waypoints (cuadrado)...")
    waypoints = [
        {'x': 0.5, 'y': 0.0, 'yaw': 0.0},
        {'x': 0.5, 'y': 0.5, 'yaw': math.pi/2},
        {'x': 0.0, 'y': 0.5, 'yaw': math.pi},
        {'x': 0.0, 'y': 0.0, 'yaw': -math.pi/2}
    ]
    if advanced_nav.navigate_with_intermediate_goals(waypoints):
        print("   Navegación por waypoints completada")


def demo_pose_management(robot):
    """Demo de gestión de poses guardadas"""
    print("\n=== DEMO GESTIÓN DE POSES ===")
    
    # Guardar pose actual con nombre específico
    timestamp = int(time.time())
    pose_name = f"demo_pose_{timestamp}"
    
    if robot.save_current_pose(pose_name, f"Pose de demo guardada a las {timestamp}"):
        print(f"   {pose_name} guardado")
    
    # Listar todas las ubicaciones
    ubicaciones = robot.list_available_locations()
    if ubicaciones:
        print("   Ubicaciones guardadas:")
        for ubicacion in ubicaciones:
            info = robot.get_pose_info(ubicacion)
            if info:
                print(f"   {ubicacion}: ({info['x']:.1f}, {info['y']:.1f}) - {info['description']}")


def show_system_architecture():
    """Muestra información sobre la arquitectura modular"""
    print("\n=== ARQUITECTURA DEL SISTEMA ===")
    print("1. RobotSkills (Básico)")
    print("   - where_am_i(): Obtener posición actual")
    print("   - go_to_pose(): Navegación a coordenadas")
    print("   - save_current_pose(): Guardar posición")
    print("   - is_moving(): Verificar movimiento")
    print("   - reached(): Verificar llegada")
    print("   - cancel(): Cancelar navegación")
    print("   - rotate(): Rotación básica")
    print("   - clear_costmaps(): Limpiar mapas de costos")
    print("   - wait_for_result(): Esperar resultado")
    
    print("\n2. SemanticNavigation (Semántico)")
    print("   - go(): Navegación por nombres")
    print("   - approach(): Acercarse manteniendo distancia")
    print("   - look(): Mirar hacia ubicación")
    print("   - patrol(): Patrullaje automático")
    print("   - create_route(): Crear rutas personalizadas")
    print("   - find_nearest_location(): Encontrar ubicación cercana")
    
    print("\n3. AdvancedNavigation (Avanzado)")
    print("   - go_to_pose_stamped(): Navegación con PoseStamped")
    print("   - look_to_pose(): Mirar hacia coordenadas")
    print("   - rotate_improved(): Rotación mejorada no bloqueante")
    print("   - navigate_with_intermediate_goals(): Navegación por waypoints")
    print("   - orbit_around_point(): Órbita circular")
    print("   - spiral_navigation(): Navegación en espiral")
    print("   - figure_eight_pattern(): Patrón figura 8")


def main():
    """Función principal del demo"""
    rclpy.init()
    
    try:
        print("Sistema Modular de Navegación Robótica")
        print("=====================================")
        
        # Crear instancias de los módulos
        robot = RobotSkills()
        semantic_nav = SemanticNavigation(robot)
        advanced_nav = AdvancedNavigation(robot)
        
        # Esperar a que Nav2 esté activo
        print("Esperando a que Nav2 esté activo...")
        robot.navigator.waitUntilNav2Active()
        print("Nav2 activo. Comenzando demo...")
        
        # Mostrar arquitectura
        show_system_architecture()
        
        # Ejecutar demos
        demo_basic_navigation(robot)
        input("\nPresiona ENTER para continuar con navegación semántica...")
        
        demo_semantic_navigation(robot, semantic_nav)
        input("\nPresiona ENTER para continuar con funciones avanzadas...")
        
        demo_advanced_navigation(robot, advanced_nav)
        input("\nPresiona ENTER para continuar con gestión de poses...")
        
        demo_pose_management(robot)
        
        # Resumen final
        print("\n=== RESUMEN DEL SISTEMA ===")
        skills_count = 0
        
        # Contar skills básicas
        basic_skills = [
            "where_am_i", "go_to_pose", "save_current_pose", "is_moving",
            "reached", "cancel", "rotate", "clear_costmaps", "wait_for_result"
        ]
        
        # Contar skills semánticas
        semantic_skills = [
            "go", "approach", "look", "patrol", "create_route", "find_nearest_location"
        ]
        
        # Contar skills avanzadas
        advanced_skills = [
            "go_to_pose_stamped", "look_to_pose", "rotate_improved",
            "navigate_with_intermediate_goals", "orbit_around_point", 
            "spiral_navigation", "figure_eight_pattern"
        ]
        
        print(f"\nSkills implementadas:")
        for i, skill in enumerate(basic_skills + semantic_skills + advanced_skills, 1):
            print(f"   {i:2d}. {skill}")
        
        print(f"\nTotal: {len(basic_skills + semantic_skills + advanced_skills)} funciones de navegación")
        print("\nPara usar estas skills en tu código:")
        print("   from skills import RobotSkills, SemanticNavigation, AdvancedNavigation")
        print("   robot = RobotSkills()")
        print("   semantic = SemanticNavigation(robot)")
        print("   advanced = AdvancedNavigation(robot)")
        
    except KeyboardInterrupt:
        print("\nDemo interrumpido por el usuario")
        
    except Exception as e:
        print(f"\nError en el demo: {e}")
        
    finally:
        # Limpiar recursos
        if 'robot' in locals():
            robot.destroy_node()
        rclpy.shutdown()
        print("Demo finalizado. Recursos liberados.")


if __name__ == '__main__':
    main()