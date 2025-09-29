#!/usr/bin/env python3
"""
Test de RobotSkills - Navegación Básica
======================================

Prueba sistemática de las 8 funciones básicas de navegación.
"""

import rclpy
import math
import time
from navegation.basicNav import RobotSkills


class BasicNavigationTest:
    """Test especializado para RobotSkills (funciones básicas)"""
    
    def __init__(self):
        self.robot = None
    

    def initialize(self):
        """Inicializar RobotSkills"""
        print(" Inicializando RobotSkills (Navegación Básica)")
        print("=" * 50)
        
        rclpy.init()
        self.robot = RobotSkills()
        
        print(" RobotSkills inicializado")
        
        # Esperar a Nav2
        print("\n Esperando que Nav2 esté activo...")
        self.robot.navigator.waitUntilNav2Active()
        print(" Nav2 activo y listo")
    

    def run_tests(self):
        """Ejecutar todas las pruebas de navegación básica"""
        print("\n" + "=" * 50)
        print(" PROBANDO ROBOTSKILLS - FUNCIONES BÁSICAS")
        print("=" * 50)
        
        # 1. where_am_i()
        print("\n Probando: where_am_i()")
        pose = self.robot.where_am_i()
        if pose:
            print(f"Posición: ({pose['x']:.2f}, {pose['y']:.2f})")
            print(f"Orientación: {pose['yaw_degrees']:.1f}°")

        else:
            print("Error obteniendo posición")
        
        # 2. is_moving()
        print("\n Probando: is_moving()")
        moving = self.robot.is_moving()
        print(f" Robot en movimiento: {moving}")
        
        # 3. go_to_pose()
        print("\n Probando: go_to_pose()")
        target_x, target_y = 1.5, 1.0
        print(f"Navegando a ({target_x}, {target_y})")
        
        if self.robot.go_to_pose(target_x, target_y, math.pi/4):
            print("Comando de navegación enviado")
            
            # 4. wait_for_result()
            print("\nProbando: wait_for_result()")
            if self.robot.wait_for_result(timeout=30.0):
                print("Navegación completada")
                
                # 5. reached()
                print("\nProbando: reached()")
                if self.robot.reached(target_x, target_y):
                    print(" Confirmado: llegué al destino")
                else:
                    print(" No llegué exactamente al destino")
            else:
                print(" Timeout en navegación")
        else:
            print(" Error enviando comando de navegación")
        
        # 6. rotate()
        print("\n Probando: rotate()")
        print(" Rotando 90° (no bloqueante)")
        if self.robot.rotate(angular_speed=0.5, duration=3.14):
            print(" Rotación iniciada")
            time.sleep(4)  # Esperar que termine
            print(" Rotación completada")
        else:
            print(" Error en rotación")
        
        # 7. stop()
        print("\n Probando: stop()")
        self.robot.stop()
        print(" Robot detenido")
        
        # 8. cancel()
        print("\n Probando: cancel()")
        # Iniciar navegación y cancelar
        print(" Iniciando navegación para cancelar...")
        self.robot.go_to_pose(5.0, 5.0, 0.0)
        time.sleep(1)
        if self.robot.cancel():
            print(" Navegación cancelada exitosamente")
        else:
            print(" No había navegación que cancelar")
        
        # Resumen
        print("\n" + "=" * 50)
        print(" RESUMEN - TEST DE NAVEGACIÓN BÁSICA")
        print("=" * 50)
        print("Funciones probadas: 8 funciones básicas")
        print("   1. where_am_i() - Obtener posición actual")
        print("   2. is_moving() - Verificar movimiento")
        print("   3. go_to_pose() - Navegar a coordenadas")
        print("   4. wait_for_result() - Esperar resultado")
        print("   5. reached() - Verificar llegada")
        print("   6. rotate() - Rotación no bloqueante")
        print("   7. stop() - Detener robot")
        print("   8. cancel() - Cancelar navegación")
        print("\n Test de RobotSkills completado!")
    
    def run_full_test(self):
        """Ejecutar test completo"""
        try:
            self.initialize()
            
            print(f"\n INICIANDO TEST DE ROBOTSKILLS")
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
    """Punto de entrada del test básico"""
    print(" Test de RobotSkills - Navegación Básica")
    print("=" * 50)
    print("Prueba sistemática de las 8 funciones básicas:")
    print("- Obtener posición y estado")
    print("- Navegación por coordenadas")
    print("- Control de movimiento y rotación")
    print("- Gestión de tareas de navegación")
    print("=" * 50)
    
    test = BasicNavigationTest()
    test.run_full_test()


if __name__ == '__main__':
    main()