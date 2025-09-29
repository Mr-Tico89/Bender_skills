# Bender Skills - Guía Completa del Sistema de Navegación

## 🎯 Introducción

**Bender Skills** es un sistema de navegación robótica modular para ROS2 con **arquitectura de 3 capas** que te permite programar un robot de manera profesional y escalable.

### ¿Qué son las Skills?
Las **skills** son habilidades específicas que tu robot puede realizar:
- **Navegar** a ubicaciones específicas
- **Guardar** posiciones importantes  
- **Gestionar** mapas de navegación
- **Patrullar** rutas automáticamente

### ¿Por qué Sistema Modular?
- **Separación clara** de responsabilidades
- **Fácil mantenimiento** y debugging
- **Escalabilidad** para agregar nuevas funciones
- **Reutilización** de componentes
- **Desarrollo profesional** estructurado

---

## 🏗️ Arquitectura del Sistema

```
┌─────────────────────┐    ┌─────────────────────┐    ┌─────────────────────┐
│   RobotSkills       │    │  AdvancedNavigation │    │ SemanticNavigation  │
│   (Básico)          │───►│   (Avanzado)        │───►│   (Semántico)       │
│                     │    │                     │    │                     │
│ • Navegación coords │    │ • Gestión poses     │    │ • Navegación nombres│
│ • Control movimiento│    │ • Mantenimiento     │    │ • Rutas inteligentes│
│ • Operaciones básicas│    │ • Persistencia      │    │ • Comportamientos   │
└─────────────────────┘    └─────────────────────┘    └─────────────────────┘
         8 funciones              5 funciones               8 funciones
```

---

## 🔧 Nivel 1: RobotSkills (Navegación Básica)

**Archivo:** `navegation/basicNav.py`  
**Funciones:** 8 básicas para control fundamental

### 1. `where_am_i()` - Obtener Posición Actual

**¿Qué hace?** Obtiene la posición y orientación actual del robot.

**Conceptos ROS2:**
- **TF2**: Sistema de transformaciones
- **Frames**: `map` (mapa global) ↔ `base_link` (robot)
- **Coordenadas**: x, y, yaw (orientación)

```python
import rclpy
from navegation.basicNav import RobotSkills

rclpy.init()
robot = RobotSkills()
robot.navigator.waitUntilNav2Active()

# Obtener posición actual
pose = robot.where_am_i()
print(f"Posición: ({pose['x']:.2f}, {pose['y']:.2f})")
print(f"Orientación: {pose['yaw_degrees']:.1f}°")
```

**Cuándo usarla:**
- Antes de planificar movimientos
- Para logging y debugging
- Como base para otras funciones

---

### 2. `go_to_pose(x, y, yaw)` - Navegación por Coordenadas

**¿Qué hace?** Navega a coordenadas específicas en el mapa.

**Conceptos ROS2:**
- **Nav2**: Stack de navegación de ROS2
- **Path Planning**: Planificación de rutas
- **PoseStamped**: Mensaje con posición + orientación + timestamp

```python
# Navegar a coordenadas específicas
success = robot.go_to_pose(2.0, 1.0, math.pi/2)  # x, y, orientación

if success:
    print("Navegación iniciada")
    # Esperar resultado
    robot.wait_for_result()
    
    # Verificar llegada
    if robot.reached(2.0, 1.0):
        print("¡Llegué al destino!")
```

**Parámetros:**
- `x, y`: Coordenadas en metros (frame "map")
- `yaw`: Orientación en radianes (0 = este, π/2 = norte)

---

### 3. `is_moving()` - Detectar Movimiento

**¿Qué hace?** Verifica si el robot está actualmente en movimiento.

```python
if robot.is_moving():
    print("Robot en movimiento")
else:
    print("Robot detenido")
```

---

### 4. `reached(x, y)` - Verificar Llegada

**¿Qué hace?** Confirma si el robot llegó cerca del objetivo.

```python
target_x, target_y = 2.0, 1.0
robot.go_to_pose(target_x, target_y, 0.0)
robot.wait_for_result()

if robot.reached(target_x, target_y, tolerance=0.3):
    print("Objetivo alcanzado")
```

---

### 5. `rotate(angular_speed, duration)` - Rotación Precisa

**¿Qué hace?** Rota el robot con velocidad y tiempo específicos (no bloqueante).

**Conceptos ROS2:**
- **Twist**: Mensaje de velocidad (linear + angular)
- **Timer**: Control temporal no bloqueante
- **cmd_vel**: Tópico estándar de control

```python
# Rotar 90° aproximadamente
robot.rotate(angular_speed=0.5, duration=3.14)  # rad/s, segundos

# El código continúa inmediatamente (no bloqueante)
print("Rotación iniciada, haciendo otras cosas...")
```

---

### 6. `stop()` - Parada Inmediata

**¿Qué hace?** Detiene inmediatamente el movimiento del robot.

```python
# En caso de emergencia
robot.stop()
print("Robot detenido")
```

---

### 7. `cancel()` - Cancelar Navegación

**¿Qué hace?** Cancela la navegación actual en curso.

```python
# Iniciar navegación
robot.go_to_pose(10.0, 10.0, 0.0)

# Cancelar si es necesario
if robot.cancel():
    print("Navegación cancelada")
```

---

### 8. `wait_for_result(timeout)` - Esperar Navegación

**¿Qué hace?** Espera que termine la navegación actual.

```python
robot.go_to_pose(3.0, 2.0, 1.57)

# Esperar máximo 30 segundos
if robot.wait_for_result(timeout=30.0):
    print("Navegación completada")
else:
    print("Timeout - navegación tomó demasiado tiempo")
```

---

## ⚙️ Nivel 2: AdvancedNavigation (Gestión Avanzada)

**Archivo:** `navegation/advNav.py`  
**Funciones:** 5 avanzadas para gestión del sistema

### 1. `save_current_pose(name, description)` - Guardar Posición

**¿Qué hace?** Guarda la posición actual con un nombre para uso posterior.

**Conceptos ROS2:**
- **Persistencia**: Guardar datos entre ejecuciones
- **NumPy**: Formato binario eficiente (.npy)
- **Timestamps**: Registro temporal

```python
from navegation.basicNav import RobotSkills
from navegation.advNav import AdvancedNavigation

robot = RobotSkills()
advanced = AdvancedNavigation(robot)

# Navegar a ubicación importante
robot.go_to_pose(2.5, 1.0, 1.57)
robot.wait_for_result()

# Guardar la ubicación
advanced.save_current_pose("cocina", "Centro de la cocina")
```

**Formato de datos:**
```python
{
  "cocina": {
    "x": 2.5,
    "y": 1.0,
    "yaw": 1.57,
    "yaw_degrees": 89.9,
    "description": "Centro de la cocina", 
    "timestamp": "2025-09-29 14:30:15"
  }
}
```

---

### 2. `load_saved_poses()` - Cargar Poses Guardadas

**¿Qué hace?** Carga todas las poses guardadas desde archivo.

```python
poses = advanced.load_saved_poses()

for name, data in poses.items():
    print(f"{name}: ({data['x']}, {data['y']}) - {data['description']}")
```

---

### 3. `list_available_locations()` - Listar Ubicaciones

**¿Qué hace?** Obtiene lista de nombres de ubicaciones disponibles.

```python
locations = advanced.list_available_locations()
print(f"Ubicaciones disponibles: {locations}")
# Output: ['cocina', 'sala', 'entrada', 'oficina']
```

---

### 4. `get_pose_info(location_name)` - Información Detallada

**¿Qué hace?** Obtiene información completa de una ubicación específica.

```python
info = advanced.get_pose_info("cocina")
if info:
    print(f"Cocina está en: ({info['x']}, {info['y']})")
    print(f"Descripción: {info['description']}")
    print(f"Guardada: {info['timestamp']}")
```

---

### 5. `clear_costmaps()` - Limpiar Mapas de Costo

**¿Qué hace?** Resetea los costmaps local y global (procesamiento paralelo).

**Conceptos ROS2:**
- **Costmaps**: Mapas de obstáculos para navegación  
- **Services**: Comunicación request/response
- **Async**: Procesamiento paralelo

```python
# Limpiar costmaps si hay problemas de navegación
if advanced.clear_costmaps():
    print("Costmaps limpiados - navegación optimizada")
```

**Cuándo usarla:**
- Robot "atascado" en navegación
- Después de mover obstáculos manualmente
- Mantenimiento periódico del sistema

---

## 🧠 Nivel 3: SemanticNavigation (Navegación Semántica)

**Archivo:** `navegation/semNav.py`  
**Funciones:** 8 semánticas para navegación inteligente

### 1. `get_available_locations()` - Ubicaciones Disponibles

**¿Qué hace?** Obtiene lista de ubicaciones para navegación semántica.

```python
from navegation.semNav import SemanticNavigation

semantic = SemanticNavigation(robot)
locations = semantic.get_available_locations()
print(f"Puedo ir a: {locations}")
```

---

### 2. `go(location_name)` - Navegación por Nombre

**¿Qué hace?** Navega a una ubicación usando su nombre en lugar de coordenadas.

```python
# Navegación semántica - ¡sin coordenadas!
semantic.go("cocina")
robot.wait_for_result()

# Equivalente a:
# robot.go_to_pose(2.5, 1.0, 1.57)  # Coordenadas de "cocina"
```

**Ventajas:**
- **Más intuitivo**: "Ve a la cocina" vs "Ve a (2.5, 1.0)"
- **Mantenible**: Si mueves muebles, solo actualizas la ubicación
- **Legible**: Código autodocumentado

---

### 3. `approach(location, distance)` - Acercarse con Distancia

**¿Qué hace?** Se acerca a una ubicación manteniendo cierta distancia.

**Conceptos:**
- **Approach behavior**: Comportamiento común en robótica
- **Distance control**: Control preciso de distancia
- **Vector math**: Cálculos geométricos

```python
# Acercarse a la mesa sin chocar
semantic.approach("mesa", approach_distance=0.8)  # Mantener 80cm
robot.wait_for_result()

print("Cerca de la mesa, listo para manipular objetos")
```

**Usos típicos:**
- Manipulación de objetos
- Interacción segura con personas
- Posicionamiento para sensores

---

### 4. `look(location_name)` - Mirar hacia Ubicación

**¿Qué hace?** Rota para mirar hacia una ubicación específica (sin moverse).

```python
# Mirar hacia la ventana sin moverse
semantic.look("ventana")
print("Ahora miro hacia la ventana")

# Útil para:
# - Orientar cámara hacia objetivo
# - Preparar maniobra
# - Interacción visual
```

---

### 5. `look_to_pose(x, y)` - Mirar hacia Coordenadas

**¿Qué hace?** Rota para mirar hacia coordenadas específicas.

```python
# Mirar hacia coordenadas exactas
semantic.look_to_pose(5.0, 3.0)

# Diferencia con look():
# look("cocina") - usa ubicación guardada
# look_to_pose(x, y) - usa coordenadas directas
```

---

### 6. `patrol(route, cycles)` - Patrullaje Automático

**¿Qué hace?** Patrulla automáticamente una ruta de ubicaciones.

**Conceptos:**
- **Autonomous behavior**: Comportamiento autónomo
- **Route planning**: Planificación de rutas
- **Loop control**: Control de ciclos

```python
# Patrullaje de seguridad
route = ["entrada", "sala", "cocina", "oficina", "entrada"]
semantic.patrol(route, cycles=3)

print("Patrullaje completado - 3 rondas de seguridad")
```

**Aplicaciones:**
- Seguridad y vigilancia
- Inspección de instalaciones  
- Recorridos guiados
- Monitoreo ambiental

---

### 7. `create_route(waypoints, name)` - Rutas Personalizadas

**¿Qué hace?** Ejecuta una ruta personalizada con acciones específicas.

```python
# Ruta de limpieza
waypoints = ["cocina", "sala", "oficina", "cocina"]
semantic.create_route(waypoints, name="ruta_limpieza")

# El robot ejecuta la ruta completa automáticamente
```

---

### 8. `find_nearest_location()` - Ubicación Más Cercana

**¿Qué hace?** Encuentra la ubicación guardada más cercana a la posición actual.

```python
nearest = semantic.find_nearest_location()
if nearest:
    print(f"La ubicación más cercana es: {nearest}")
    semantic.go(nearest)
```

---

## 🛠️ Cómo Usar el Sistema

### Desarrollo y Testing

#### Tests Especializados por Módulo:
```bash
cd navegation/demo/

# Probar solo navegación básica (8 funciones)
python3 basicTest.py

# Probar solo gestión avanzada (5 funciones)  
python3 advTest.py

# Probar solo navegación semántica (8 funciones)
python3 semTest.py

# Verificar importaciones
python3 test_modules.py
```

#### Tests con ROS2:
```bash
# Después de compilar el paquete
colcon build --packages-select bender_skills

# Tests disponibles
ros2 run bender_skills basic_test
ros2 run bender_skills adv_test  
ros2 run bender_skills sem_test
ros2 run bender_skills test_modules
```

### Ejemplo Completo de Uso

```python
#!/usr/bin/env python3
"""
Ejemplo completo usando los 3 módulos del sistema
"""
import rclpy
import math
from navegation.basicNav import RobotSkills
from navegation.advNav import AdvancedNavigation  
from navegation.semNav import SemanticNavigation

def main():
    # Inicializar ROS2
    rclpy.init()
    
    # Crear instancias de los 3 módulos
    robot = RobotSkills()
    advanced = AdvancedNavigation(robot)
    semantic = SemanticNavigation(robot)
    
    # Esperar Nav2
    robot.navigator.waitUntilNav2Active()
    print("Sistema iniciado")
    
    # === NAVEGACIÓN BÁSICA ===
    print("\\n1. Navegación básica por coordenadas")
    pose = robot.where_am_i()
    print(f"Posición inicial: ({pose['x']:.2f}, {pose['y']:.2f})")
    
    # Navegar a punto específico
    robot.go_to_pose(2.0, 1.0, math.pi/2)
    robot.wait_for_result()
    
    # === GESTIÓN AVANZADA ===
    print("\\n2. Guardar ubicación importante")
    advanced.save_current_pose("punto_trabajo", "Área de trabajo principal")
    
    locations = advanced.list_available_locations()
    print(f"Ubicaciones guardadas: {locations}")
    
    # === NAVEGACIÓN SEMÁNTICA ===
    print("\\n3. Navegación semántica")
    
    # Crear más ubicaciones para demostrar
    robot.go_to_pose(0.0, 2.0, 0.0)
    robot.wait_for_result()
    advanced.save_current_pose("base", "Punto base")
    
    robot.go_to_pose(3.0, 0.0, -math.pi/2)
    robot.wait_for_result()
    advanced.save_current_pose("estacion", "Estación de carga")
    
    # Navegación por nombres
    semantic.go("punto_trabajo")
    robot.wait_for_result()
    print("Llegué al punto de trabajo")
    
    # Approach inteligente
    semantic.approach("base", approach_distance=1.0)
    robot.wait_for_result()
    print("Me acerqué a la base manteniendo distancia")
    
    # Mirar hacia objetivo
    semantic.look("estacion")
    print("Mirando hacia la estación")
    
    # Patrullaje automático
    print("\\n4. Patrullaje automático")
    route = ["base", "punto_trabajo", "estacion"]
    semantic.patrol(route, cycles=2)
    print("Patrullaje completado")
    
    # Limpiar y salir
    robot.destroy_node()
    rclpy.shutdown()
    print("\\nSistema terminado correctamente")

if __name__ == '__main__':
    main()
```

---

## 📊 Casos de Uso por Industria

### 🏭 **Manufactura e Industria**
```python
# Robot de inspección industrial
semantic.create_route([
    "estacion_1", "estacion_2", "estacion_3", 
    "control_calidad", "almacen"
], name="inspeccion_diaria")

# Mantenimiento preventivo
advanced.clear_costmaps()  # Optimizar navegación
```

### 🏥 **Salud y Hospitales**
```python
# Robot de entrega médica
semantic.go("farmacia")
semantic.approach("cama_paciente_204", approach_distance=0.5)
semantic.look("enfermera")  # Orientarse para interacción
```

### 🏪 **Retail y Comercio**
```python
# Robot guía en tienda
route = ["entrada", "electronica", "ropa", "supermercado", "caja"]
semantic.patrol(route, cycles=10)  # Patrullaje continuo

# Inventario automático
semantic.go("pasillo_A")
semantic.look_to_pose(x_shelf, y_shelf)  # Orientar cámara a estante
```

### 🏠 **Domótica y Hogar**
```python
# Robot doméstico
semantic.go("cocina")
semantic.approach("refrigerador", approach_distance=0.3)

# Ronda nocturna
semantic.patrol(["sala", "cocina", "entrada"], cycles=1)
```

---

## 🎯 Mejores Prácticas

### ✅ **Recomendaciones de Uso**

1. **Siempre esperar Nav2:**
```python
robot.navigator.waitUntilNav2Active()  # ANTES de cualquier navegación
```

2. **Manejar timeouts:**
```python
if robot.wait_for_result(timeout=30.0):
    print("Éxito")
else:
    print("Timeout - revisar obstáculos")
```

3. **Verificar llegada:**
```python
robot.go_to_pose(x, y, yaw)
robot.wait_for_result()
if robot.reached(x, y):
    print("Confirmado en destino")
```

4. **Guardar ubicaciones importantes:**
```python
# Después de navegar manualmente a ubicación importante
advanced.save_current_pose("ubicacion_critica", "Descripción clara")
```

### ❌ **Errores Comunes**

1. **No inicializar ROS2:**
```python
# MAL
robot = RobotSkills()  # Error: ROS2 no iniciado

# BIEN  
rclpy.init()
robot = RobotSkills()
```

2. **No esperar resultados:**
```python
# MAL
robot.go_to_pose(2.0, 1.0, 0.0)
robot.go_to_pose(3.0, 2.0, 0.0)  # Cancela navegación anterior

# BIEN
robot.go_to_pose(2.0, 1.0, 0.0)
robot.wait_for_result()  # Esperar que termine
robot.go_to_pose(3.0, 2.0, 0.0)
```

3. **Olvidar limpiar recursos:**
```python
# MAL
robot = RobotSkills()
# ... código ...
# Programa termina sin cleanup

# BIEN
try:
    robot = RobotSkills()
    # ... código ...
finally:
    robot.destroy_node()
    rclpy.shutdown()
```

---

## 🔧 Configuración y Requisitos

### **Dependencias del Sistema:**
- **ROS2 Humble** (o superior)
- **Nav2** stack completo
- **Python 3.8+**
- **NumPy** para persistencia

### **Instalación:**
```bash
# Clonar repositorio
git clone https://github.com/Mr-Tico89/Bender_skills.git
cd Bender_skills

# Instalar dependencias
rosdep install --from-paths . --ignore-src -r -y

# Compilar
colcon build --packages-select bender_skills

# Sourcing
source install/setup.bash
```

### **Verificación:**
```bash
# Verificar que todo funciona
ros2 run bender_skills test_modules
```

---

## 📈 Roadmap y Extensiones Futuras

### **Funciones Planeadas:**
- **Multi-robot coordination**: Coordinación entre múltiples robots
- **Dynamic replanning**: Re-planificación dinámica de rutas  
- **Object interaction**: Interacción con objetos específicos
- **Human following**: Seguimiento de personas
- **Voice commands**: Comandos de voz integrados

### **Integración con otros Sistemas:**
- **MoveIt**: Para manipulación de brazos robóticos
- **Perception**: Para detección y reconocimiento de objetos
- **SLAM**: Para creación dinámica de mapas
- **Fleet management**: Para gestión de flotas

---

## 📚 Referencias y Recursos Adicionales

### **ROS2 y Nav2:**
- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [Nav2 Documentation](https://navigation.ros.org/)  
- [TF2 Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)

### **Arquitectura del Sistema:**
- **Básico (RobotSkills)**: 8 funciones fundamentales
- **Avanzado (AdvancedNavigation)**: 5 funciones de gestión  
- **Semántico (SemanticNavigation)**: 8 funciones inteligentes
- **Total**: 21 funciones especializadas

---

**Desarrollado para ROS2 Humble con Nav2**  
*Sistema de navegación robótica modular y profesional*

🤖 **¡Listo para crear robots inteligentes!** 🚀