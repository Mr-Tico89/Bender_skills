#!/usr/bin/env python3

"""
Demo completa de todas las skills de navegación implementadas.
Este archivo está separado del código principal para mantener skills.py limpio.
"""

import rclpy
import math
import time
from navegation.basicNav import RobotSkills


def demo_basic_skills(robot):
    """Demo de skills básicas"""
    print("\n === DEMO SKILLS BÁSICAS ===")
    
    print("1. where_am_i() - ¿Dónde estoy?")
    pose_info = robot.where_am_i()
    if pose_info:
        print(f"    Posición: ({pose_info['x']:.2f}, {pose_info['y']:.2f})")
        print(f"    Orientación: {pose_info['yaw_degrees']:.1f}°")
    
    print("\n2. save_current_pose() - Guardar ubicación")
    if robot.save_current_pose("demo_start", "Punto de inicio del demo"):
        print("    Posición guardada exitosamente")
    
    print("\n3. is_moving() - ¿Me estoy moviendo?")
    moving = robot.is_moving()
    print(f"   🚶 En movimiento: {moving}")
    
    print("\n4. go_to_pose() - Navegación básica")
    if robot.go_to_pose(1.5, 1.0, math.pi/4):
        print("   🚀 Navegación iniciada a (1.5, 1.0)")
        
        print("\n5. wait_for_result() - Esperar resultado")
        if robot.wait_for_result(timeout=20.0):
            print("    ¡Navegación completada!")
            
            print("\n6. reached() - Verificar llegada")
            if robot.reached():
                print("    Confirmado: llegué al destino")
        else:
            print("   ⏰ Timeout en navegación")
    
    print("\n7. rotate() - Rotación mejorada")
    robot.rotate(angular_speed=0.8, duration=2.0)  # Rotar ~90°
    print("    Rotación de ~90° completada")


def demo_semantic_navigation(robot):
    """Demo de navegación semántica"""
    print("\n🧠 === DEMO NAVEGACIÓN SEMÁNTICA ===")
    
    ubicaciones = robot.list_available_locations()
    print(f"1. Ubicaciones disponibles: {ubicaciones}")
    
    if len(ubicaciones) == 0:
        print("     Agregando algunas ubicaciones de ejemplo...")
        
        # Agregar ubicaciones de ejemplo
        robot.go_to_pose(0.0, 0.0, 0.0)
        robot.wait_for_result(timeout=10.0)
        robot.save_current_pose("centro", "Centro del área")
        
        robot.go_to_pose(2.0, 0.0, math.pi/2)
        robot.wait_for_result(timeout=10.0)  
        robot.save_current_pose("este", "Punto al este")
        
        robot.go_to_pose(0.0, 2.0, math.pi)
        robot.wait_for_result(timeout=10.0)
        robot.save_current_pose("norte", "Punto al norte")
        
        ubicaciones = robot.list_available_locations()
        print(f"    Ubicaciones creadas: {ubicaciones}")
    
    if len(ubicaciones) >= 2:
        primera_ubicacion = ubicaciones[0]
        print(f"\n2. go() - Ir a '{primera_ubicacion}'")
        
        if robot.go(primera_ubicacion):
            robot.wait_for_result(timeout=15.0)
            print(f"    Llegué a '{primera_ubicacion}'")
            
            if len(ubicaciones) > 1:
                segunda_ubicacion = ubicaciones[1]
                
                print(f"\n3. look() - Mirar hacia '{segunda_ubicacion}'")
                robot.look(segunda_ubicacion)
                time.sleep(2.0)
                print(f"   👀 Mirando hacia '{segunda_ubicacion}'")
                
                print(f"\n4. approach() - Acercarse a '{segunda_ubicacion}'")
                robot.approach(segunda_ubicacion, approach_distance=0.8)
                robot.wait_for_result(timeout=15.0)
                print(f"    Me acerqué a '{segunda_ubicacion}' (0.8m de distancia)")
        
    else:
        print("     Necesito al menos 2 ubicaciones para demo completo")


def demo_advanced_functions(robot):
    """Demo de funciones avanzadas"""
    print("\n⚡ === DEMO FUNCIONES AVANZADAS ===")
    
    print("1. clear_costmaps() - Limpiar mapas de navegación")
    if robot.clear_costmaps():
        print("    Costmaps limpiados")
    else:
        print("     Algunos costmaps no se pudieron limpiar")
    
    print("\n2. look_to_pose() - Mirar coordenada específica")
    robot.look_to_pose(0.0, 0.0)
    time.sleep(2.0)
    print("   👀 Mirando hacia el origen (0, 0)")
    
    print("\n3. cancel() - Cancelar navegación")
    robot.go_to_pose(10.0, 10.0)  # Punto lejano
    time.sleep(1.0)
    if robot.cancel():
        print("   🛑 Navegación cancelada exitosamente")
    
    print("\n4. Rotación completa (360°)")
    robot.rotate(angular_speed=1.0, duration=2*math.pi)  # 360 grados
    print("    Vuelta completa realizada")


def demo_pose_management(robot):
    """Demo de gestión de poses"""
    print("\n💾 === DEMO GESTIÓN DE POSES ===")
    
    print("1. Creando ruta de waypoints...")
    waypoints = [
        ("waypoint_1", "Primer punto", 1.0, 0.0, 0.0),
        ("waypoint_2", "Segundo punto", 1.0, 1.0, math.pi/2),
        ("waypoint_3", "Tercer punto", 0.0, 1.0, math.pi),
    ]
    
    for name, desc, x, y, yaw in waypoints:
        print(f"   Navegando a {name}...")
        robot.go_to_pose(x, y, yaw)
        robot.wait_for_result(timeout=15.0)
        robot.save_current_pose(name, desc)
        print(f"    {name} guardado")
    
    print(f"\n2. Ubicaciones guardadas: {robot.list_available_locations()}")
    
    print("\n3. Información detallada de poses:")
    for ubicacion in robot.list_available_locations():
        info = robot.get_pose_info(ubicacion)
        if info:
            print(f"    {ubicacion}: ({info['x']:.1f}, {info['y']:.1f}) - {info['description']}")
    
    print("\n4. Recorriendo waypoints guardados...")
    for waypoint in ["waypoint_1", "waypoint_2", "waypoint_3"]:
        if waypoint in robot.list_available_locations():
            print(f"   Visitando {waypoint}...")
            robot.go(waypoint)
            robot.wait_for_result(timeout=10.0)


def main():
    """Función principal del demo"""
    rclpy.init()
    
    try:
        print("🚀 Demo Completa de Robot Skills")
        print("================================")
        
        robot = RobotSkills()
        
        print(f"\n📊 Estado inicial:")
        print(f"   Ubicaciones guardadas: {robot.list_available_locations()}")
        
        # Ejecutar demos secuencialmente
        demo_basic_skills(robot)
        
        input("\n  Presiona ENTER para continuar con navegación semántica...")
        demo_semantic_navigation(robot)
        
        input("\n  Presiona ENTER para continuar con funciones avanzadas...")
        demo_advanced_functions(robot)
        
        input("\n  Presiona ENTER para continuar con gestión de poses...")
        demo_pose_management(robot)
        
        print("\n🎉 ¡Demo completado exitosamente!")
        
        # Resumen de skills
        print("\n📋 Skills implementadas:")
        skills_implemented = [
            "where_am_i() - Obtener posición actual",
            "save_current_pose() - Guardar ubicaciones con nombre",
            "go_to_pose() - Navegación a coordenadas",
            "go() - Navegación semántica por nombre",
            "approach() - Acercarse manteniendo distancia", 
            "look() - Mirar hacia ubicación nombrada",
            "look_to_pose() - Mirar hacia coordenadas",
            "rotate() - Rotación mejorada",
            "is_moving() - Detectar movimiento",
            "reached() - Verificar llegada",
            "cancel() - Cancelar navegación",
            "clear_costmaps() - Limpiar mapas de navegación",
            "wait_for_result() - Espera bloqueante",
            "go_to_pose_stamped() - Navegación con PoseStamped"
        ]
        
        for i, skill in enumerate(skills_implemented, 1):
            print(f"   {i:2d}.  {skill}")
        
        print("\nPara usar estas skills en tu código:")
        print("   from skills.skills import RobotSkills")
        print("   robot = RobotSkills()")
        print("   robot.where_am_i()")
        print("   robot.go_to_pose(2.0, 1.0)")
        print("   robot.save_current_pose('mi_lugar')")
        
    except KeyboardInterrupt:
        print("\n  Demo interrumpido por el usuario")
        
    except Exception as e:
        print(f"\nError en el demo: {e}")
        
    finally:
        if 'robot' in locals():
            robot.cancel()  # Por seguridad
            robot.destroy_node()
        rclpy.shutdown()
        print("\n👋 Demo finalizado")


if __name__ == '__main__':
    main()