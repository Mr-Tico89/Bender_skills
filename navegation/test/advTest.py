#!/usr/bin/env python3
"""
Test de AdvancedNavigation - Gestión Avanzada
============================================

Prueba sistemática de las 5 funciones avanzadas de navegación.
"""

import rclpy
import time
from navegation.basicNav import RobotSkills
from navegation.advNav import AdvancedNavigation


class AdvancedNavigationTest:
    """Test especializado para AdvancedNavigation (funciones avanzadas)"""
    
    def __init__(self):
        self.robot = None
        self.advanced = None
    
    def initialize(self):
        """Inicializar AdvancedNavigation"""
        print(" Inicializando AdvancedNavigation (Gestión Avanzada)")
        print("=" * 50)
        
        rclpy.init()
        
        # Crear instancias
        self.robot = RobotSkills()
        self.advanced = AdvancedNavigation(self.robot)
        
        print(" RobotSkills (base) inicializado")
        print(" AdvancedNavigation inicializado")
        
        # Esperar a Nav2
        print("\n Esperando que Nav2 esté activo...")
        self.robot.navigator.waitUntilNav2Active()
        print(" Nav2 activo y listo")
    
    def run_tests(self):
        """Ejecutar todas las pruebas de navegación avanzada"""
        print("\n" + "=" * 50)
        print(" PROBANDO ADVANCEDNAVIGATION - FUNCIONES AVANZADAS")
        print("=" * 50)
        
        # 1. save_current_pose()
        print("\n Probando: save_current_pose()")
        pose_name = "test_pose_advanced"
        description = "Pose de prueba para test avanzado"
        print(f" Guardando pose '{pose_name}'")
        
        if self.advanced.save_current_pose(pose_name, description):
            print(f" Pose '{pose_name}' guardada exitosamente")
        else:
            print(f" Error guardando pose '{pose_name}'")
        
        # 2. load_saved_poses()
        print("\n Probando: load_saved_poses()")
        print(" Cargando todas las poses guardadas...")
        
        poses = self.advanced.load_saved_poses()
        if poses:
            print(f" Cargadas {len(poses)} poses guardadas")
            for name in list(poses.keys())[:3]:  # Mostrar solo 3 primeras
                print(f"      - {name}")
            if len(poses) > 3:
                print(f"      ... y {len(poses)-3} más")
        else:
            print(" No hay poses guardadas o error al cargar")
        
        # 3. list_available_locations()
        print("\n Probando: list_available_locations()")
        print(" Listando ubicaciones disponibles...")
        
        locations = self.advanced.list_available_locations()
        if locations:
            print(f" Ubicaciones disponibles ({len(locations)}):")
            for loc in locations[:5]:  # Mostrar solo 5 primeras
                print(f"      - {loc}")
            if len(locations) > 5:
                print(f"      ... y {len(locations)-5} más")
        else:
            print(" No hay ubicaciones disponibles")
        
        # 4. get_pose_info()
        print("\n Probando: get_pose_info()")
        if pose_name in locations:
            print(f" Obteniendo información de '{pose_name}'")
            
            info = self.advanced.get_pose_info(pose_name)
            if info:
                print(f" Información de '{pose_name}':")
                print(f" - Posición: ({info['x']:.2f}, {info['y']:.2f})")
                print(f" - Orientación: {info['yaw_degrees']:.1f}°")
                print(f" - Descripción: {info['description']}")
                print(f" - Creada: {info['timestamp']}")
            else:
                print(f"No se pudo obtener información de '{pose_name}'")
        else:
            print(f" '{pose_name}' no está en las ubicaciones disponibles")
        
        # 5. clear_costmaps()
        print("\n Probando: clear_costmaps()")
        print(" Limpiando costmaps (local y global)...")
        
        if self.advanced.clear_costmaps():
            print(" Costmaps limpiados exitosamente")
            print(" Procesamiento paralelo completado")
        else:
            print(" Error limpiando algunos costmaps")
        
        # Test adicional: Guardar otra pose para demostrar funcionalidad
        print("\n Test adicional: Guardar pose en nueva ubicación")
        print(" Navegando a nueva posición...")
        
        # Navegar a nueva posición
        if self.robot.go_to_pose(2.0, 1.5, 0.0):
            if self.robot.wait_for_result(timeout=20.0):
                # Guardar nueva pose
                new_pose_name = "test_pose_2"
                if self.advanced.save_current_pose(new_pose_name, "Segunda pose de prueba"):
                    print(f" Nueva pose '{new_pose_name}' guardada")
                    
                    # Verificar que ahora hay más ubicaciones
                    updated_locations = self.advanced.list_available_locations()
                    print(f" Total de ubicaciones ahora: {len(updated_locations)}")
                else:
                    print(f" Error guardando '{new_pose_name}'")
            else:
                print(" Timeout navegando a nueva posición")
        
        # Resumen
        print("\n" + "=" * 50)
        print(" RESUMEN - TEST DE NAVEGACIÓN AVANZADA")
        print("=" * 50)
        print("Funciones probadas: 5 funciones avanzadas")
        print("   1. save_current_pose() - Guardar posición actual")
        print("   2. load_saved_poses() - Cargar poses guardadas")
        print("   3. list_available_locations() - Listar ubicaciones")
        print("   4. get_pose_info() - Información detallada de pose")
        print("   5. clear_costmaps() - Limpiar mapas de costo")
        print(f"\n Ubicaciones totales gestionadas: {len(self.advanced.list_available_locations())}")
        print(" Test de AdvancedNavigation completado!")
    
    def run_full_test(self):
        """Ejecutar test completo"""
        try:
            self.initialize()
            
            print(f"\n INICIANDO TEST DE ADVANCEDNAVIGATION")
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
    """Punto de entrada del test avanzado"""
    print(" Test de AdvancedNavigation - Gestión Avanzada")
    print("=" * 55)
    print("Prueba sistemática de las 5 funciones avanzadas:")
    print("- Gestión de poses guardadas")
    print("- Información de ubicaciones")
    print("- Mantenimiento del sistema (costmaps)")
    print("- Persistencia de datos en archivos")
    print("=" * 55)
    
    test = AdvancedNavigationTest()
    test.run_full_test()


if __name__ == '__main__':
    main()