#!/usr/bin/env python3
"""
Test de SemanticNavigation - Navegación Semántica
================================================

Prueba sistemática de las 6 funciones semánticas de navegación.
"""

import rclpy
import math
import time
from navegation.basicNav import RobotSkills
from navegation.advNav import AdvancedNavigation
from navegation.semNav import SemanticNavigation


class SemanticNavigationTest:
    """Test especializado para SemanticNavigation (funciones semánticas)"""
    
    def __init__(self):
        self.robot = None
        self.advanced = None
        self.semantic = None
    
    def initialize(self):
        """Inicializar SemanticNavigation"""
        print(" Inicializando SemanticNavigation (Navegación Semántica)")
        print("=" * 55)
        
        rclpy.init()
        
        # Crear instancias de los 3 módulos (semántico necesita los otros)
        self.robot = RobotSkills()
        self.advanced = AdvancedNavigation(self.robot)
        self.semantic = SemanticNavigation(self.robot)
        
        print(" RobotSkills (base) inicializado")
        print(" AdvancedNavigation (soporte) inicializado")
        print(" SemanticNavigation inicializado")
        
        # Esperar a Nav2
        print("\n Esperando que Nav2 esté activo...")
        self.robot.navigator.waitUntilNav2Active()
        print(" Nav2 activo y listo")
    
    def setup_test_locations(self):
        """Crear ubicaciones de prueba para navegación semántica"""
        print("\n Preparando ubicaciones de prueba...")
        
        # Ubicaciones de prueba
        test_locations = [
            {"name": "entrada", "x": 0.0, "y": 0.0, "yaw": 0.0, "desc": "Punto de entrada"},
            {"name": "sala", "x": 2.0, "y": 1.0, "yaw": math.pi/2, "desc": "Área de sala"},
            {"name": "cocina", "x": 1.0, "y": 2.0, "yaw": math.pi, "desc": "Área de cocina"},
            {"name": "oficina", "x": 3.0, "y": 0.5, "yaw": -math.pi/2, "desc": "Espacio de oficina"},
            {"name": "balcon", "x": 2.5, "y": 2.5, "yaw": 3*math.pi/4, "desc": "Área del balcón"}
        ]
        
        created_count = 0
        for location in test_locations:
            print(f" Creando '{location['name']}'...")
            
            # Navegar a la posición
            if self.robot.go_to_pose(location["x"], location["y"], location["yaw"]):
                if self.robot.wait_for_result(timeout=15.0):
                    # Guardar la ubicación
                    if self.advanced.save_current_pose(location["name"], location["desc"]):
                        print(f" '{location['name']}' creada exitosamente")
                        created_count += 1
                    else:
                        print(f" Error guardando '{location['name']}'")
                else:
                    print(f" Timeout navegando a '{location['name']}'")
            else:
                print(f" Error iniciando navegación a '{location['name']}'")
        
        print(f" {created_count}/{len(test_locations)} ubicaciones creadas")
        return created_count > 0
    
    def run_tests(self):
        """Ejecutar todas las pruebas de navegación semántica"""
        print("\n" + "=" * 55)
        print(" PROBANDO SEMANTICNAVIGATION - FUNCIONES SEMÁNTICAS")
        print("=" * 55)
        
        # Preparar ubicaciones de prueba
        if not self.setup_test_locations():
            print(" No se pudieron crear ubicaciones de prueba")
            return
        
        # 1. get_available_locations()
        print("\n Probando: get_available_locations()")
        print(" Obteniendo ubicaciones disponibles...")
        
        locations = self.semantic.get_available_locations()
        if locations:
            print(f" Ubicaciones disponibles ({len(locations)}):")
            for loc in locations:
                print(f"      - {loc}")
        else:
            print(" No hay ubicaciones disponibles")
            return
        
        # 2. go()
        print("\n Probando: go()")
        target_location = "sala"
        if target_location in locations:
            print(f" Navegando semánticamente a '{target_location}'")
            
            if self.semantic.go(target_location):
                print(" Navegación semántica iniciada")
                if self.robot.wait_for_result(timeout=20.0):
                    print(f" Llegué exitosamente a '{target_location}'")
                else:
                    print(" Timeout en navegación semántica")
            else:
                print(f" Error navegando a '{target_location}'")
        else:
            print(f" '{target_location}' no está disponible")
        
        # 3. approach()
        print("\n Probando: approach()")
        approach_target = "cocina"
        approach_distance = 0.8
        
        if approach_target in locations:
            print(f" Acercándome a '{approach_target}' (distancia: {approach_distance}m)")
            
            if self.semantic.approach(approach_target, approach_distance=approach_distance):
                print(" Approach iniciado")
                if self.robot.wait_for_result(timeout=20.0):
                    print(f" Me acerqué manteniendo {approach_distance}m de distancia")
                else:
                    print(" Timeout en approach")
            else:
                print(f" Error en approach a '{approach_target}'")
        else:
            print(f" '{approach_target}' no está disponible")
        
        # 4. look()
        print("\n Probando: look()")
        look_target = "oficina"
        
        if look_target in locations:
            print(f" Mirando hacia '{look_target}'")
            
            if self.semantic.look(look_target):
                print(" Rotación hacia objetivo iniciada")
                time.sleep(3)  # Esperar rotación
                print(f" Ahora miro hacia '{look_target}'")
            else:
                print(f" Error mirando hacia '{look_target}'")
        else:
            print(f" '{look_target}' no está disponible")
        
        # 5. patrol()
        print("\n Probando: patrol()")
        patrol_route = ["entrada", "sala", "cocina"] if all(loc in locations for loc in ["entrada", "sala", "cocina"]) else locations[:3]
        cycles = 1
        
        print(f" Patrullando ruta: {patrol_route} ({cycles} ciclo)")
        
        if self.semantic.patrol(patrol_route, cycles=cycles):
            print(" Patrullaje completado exitosamente")
        else:
            print(" Error en patrullaje")
        
        # 6. create_route()
        print("\n Probando: create_route()")
        custom_route = ["cocina", "entrada", "oficina", "balcon"] if all(loc in locations for loc in ["cocina", "entrada", "oficina", "balcon"]) else locations[:4]
        route_name = "ruta_completa"
        
        print(f" Creando ruta '{route_name}': {custom_route}")
        
        if self.semantic.create_route(custom_route, name=route_name):
            print(" Ruta personalizada ejecutada exitosamente")
        else:
            print(" Error ejecutando ruta personalizada")
        
        # Test adicional: find_nearest_location() 
        print("\n Test adicional: find_nearest_location()")
        print(" Buscando ubicación más cercana...")
        
        nearest = self.semantic.find_nearest_location()
        if nearest:
            print(f" Ubicación más cercana: '{nearest}'")
        else:
            print(" No se pudo determinar la ubicación más cercana")
        
        # Resumen
        print("\n" + "=" * 55)
        print("RESUMEN - TEST DE NAVEGACIÓN SEMÁNTICA")
        print("=" * 55)
        print("Funciones probadas: 6 funciones semánticas")
        print("   1. get_available_locations() - Obtener ubicaciones")
        print("   2. go() - Navegación por nombres")
        print("   3. approach() - Acercarse con distancia")
        print("   4. look() - Mirar hacia ubicación")
        print("   5. patrol() - Patrullaje automático")
        print("   6. create_route() - Rutas personalizadas")
        print(f"\n Ubicaciones procesadas: {len(locations)}")
        print(f" Ubicaciones: {', '.join(locations)}")
        print(" Test de SemanticNavigation completado!")
    
    def run_full_test(self):
        """Ejecutar test completo"""
        try:
            self.initialize()
            
            print(f"\n INICIANDO TEST DE SEMANTICNAVIGATION")
            print(f" {time.strftime('%Y-%m-%d %H:%M:%S')}")
            
            self.run_tests()
            
        except KeyboardInterrupt:
            print("\n Test interrumpido por el usuario")
        except Exception as e:
            print(f"\n Error en test: {e}")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Limpiar recursos"""
        if self.robot:
            self.robot.destroy_node()
        rclpy.shutdown()
        print("\n Recursos liberados correctamente")


def main():
    """Punto de entrada del test semántico"""
    print(" Test de SemanticNavigation - Navegación Semántica")
    print("=" * 60)
    print("Prueba sistemática de las 6 funciones semánticas:")
    print("- Navegación por nombres de ubicaciones")
    print("- Acercamiento con control de distancia")
    print("- Orientación hacia objetivos")
    print("- Patrullaje automático de rutas")
    print("- Creación de rutas personalizadas")
    print("=" * 60)
    
    test = SemanticNavigationTest()
    test.run_full_test()


if __name__ == '__main__':
    main()