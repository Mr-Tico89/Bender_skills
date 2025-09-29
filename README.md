# Bender Skills - Sistema de Navegación Robótica Modular para ROS2

## Arquitectura del Sistema
Sistema de navegación **modular de 3 capas** con funciones especializadas por nivel de complejidad:

```
RobotSkills (Básico) → AdvancedNavigation (Avanzado) → SemanticNavigation (Semántico)
```

## 🔧 RobotSkills (Navegación Básica) - 8 funciones
**Módulo base:** `navegation/basicNav.py`

| Función | Descripción | Uso |
|---------|-------------|-----|
| `where_am_i()` | Obtener posición y orientación actual | `pose = robot.where_am_i()` |
| `go_to_pose(x, y, yaw)` | Navegación a coordenadas específicas | `robot.go_to_pose(2.0, 1.0, 0.0)` |
| `is_moving()` | Detectar si el robot está en movimiento | `if robot.is_moving():` |
| `reached(x, y)` | Verificar si llegó al objetivo | `if robot.reached(2.0, 1.0):` |
| `rotate(speed, duration)` | Rotación no bloqueante con tiempo | `robot.rotate(0.5, 3.14)` |
| `stop()` | Detener movimiento inmediatamente | `robot.stop()` |
| `cancel()` | Cancelar navegación actual | `robot.cancel()` |
| `wait_for_result(timeout)` | Esperar resultado de navegación | `robot.wait_for_result(30.0)` |

## ⚙️ AdvancedNavigation (Gestión Avanzada) - 5 funciones
**Módulo avanzado:** `navegation/advNav.py`

| Función | Descripción | Uso |
|---------|-------------|-----|
| `save_current_pose(name, desc)` | Guardar posición actual con nombre | `adv.save_current_pose("cocina", "Centro cocina")` |
| `load_saved_poses()` | Cargar poses guardadas desde archivo | `poses = adv.load_saved_poses()` |
| `list_available_locations()` | Lista de ubicaciones disponibles | `locations = adv.list_available_locations()` |
| `get_pose_info(location)` | Información detallada de una pose | `info = adv.get_pose_info("cocina")` |
| `clear_costmaps()` | Limpiar mapas de costo (paralelo) | `adv.clear_costmaps()` |

## 🧠 SemanticNavigation (Navegación Semántica) - 8 funciones
**Módulo semántico:** `navegation/semNav.py`

| Función | Descripción | Uso |
|---------|-------------|-----|
| `get_available_locations()` | Obtener ubicaciones disponibles | `locations = sem.get_available_locations()` |
| `go(location_name)` | Ir a ubicación por nombre | `sem.go("cocina")` |
| `approach(location, distance)` | Acercarse manteniendo distancia | `sem.approach("mesa", 0.8)` |
| `look(location_name)` | Mirar hacia ubicación específica | `sem.look("ventana")` |
| `look_to_pose(x, y)` | Rotar para mirar hacia coordenadas | `sem.look_to_pose(3.0, 2.0)` |
| `patrol(route, cycles)` | Patrullaje automático de ruta | `sem.patrol(["cocina", "sala"], 3)` |
| `create_route(waypoints, name)` | Crear ruta personalizada | `sem.create_route(waypoints, "ruta_1")` |
| `find_nearest_location()` | Encontrar ubicación más cercana | `nearest = sem.find_nearest_location()` |




## 🚀 Uso del Sistema Modular

### 1. Navegación Básica (RobotSkills)
```python
import rclpy
from navegation.basicNav import RobotSkills

rclpy.init()
robot = RobotSkills()
robot.navigator.waitUntilNav2Active()

# Obtener posición actual
pose = robot.where_am_i()
print(f"Estoy en: ({pose['x']:.2f}, {pose['y']:.2f})")

# Navegar a coordenadas
robot.go_to_pose(2.0, 1.0, 0.0)
robot.wait_for_result()

# Control de movimiento
if robot.reached(2.0, 1.0):
    print("¡Llegué al destino!")

# Rotación no bloqueante
robot.rotate(angular_speed=0.5, duration=3.14)

robot.destroy_node()
rclpy.shutdown()
```

### 2. Gestión Avanzada (AdvancedNavigation)
```python
import rclpy
from navegation.basicNav import RobotSkills
from navegation.advNav import AdvancedNavigation

rclpy.init()
robot = RobotSkills()
advanced = AdvancedNavigation(robot)
robot.navigator.waitUntilNav2Active()

# Navegar y guardar posición
robot.go_to_pose(3.0, 2.0, 1.57)
robot.wait_for_result()
advanced.save_current_pose("cocina", "Centro de la cocina")

# Gestionar ubicaciones
locations = advanced.list_available_locations()
print(f"Ubicaciones: {locations}")

# Obtener información detallada
info = advanced.get_pose_info("cocina")
print(f"Cocina: {info}")

# Limpiar costmaps si hay problemas
advanced.clear_costmaps()

robot.destroy_node()
rclpy.shutdown()
```

### 3. Navegación Semántica (SemanticNavigation)
```python
import rclpy
from navegation.basicNav import RobotSkills
from navegation.semNav import SemanticNavigation

rclpy.init()
robot = RobotSkills()
semantic = SemanticNavigation(robot)
robot.navigator.waitUntilNav2Active()

# Navegación por nombres
semantic.go("cocina")
robot.wait_for_result()

# Acercarse manteniendo distancia
semantic.approach("mesa", approach_distance=0.8)
robot.wait_for_result()

# Mirar hacia ubicaciones
semantic.look("ventana")

# Mirar hacia coordenadas específicas
semantic.look_to_pose(5.0, 3.0)

# Patrullaje automático
semantic.patrol(["cocina", "sala", "entrada"], cycles=2)

# Rutas personalizadas
waypoints = ["cocina", "sala", "oficina", "cocina"]
semantic.create_route(waypoints, name="ruta_limpieza")

robot.destroy_node()
rclpy.shutdown()
```

## 📁 Estructura del Proyecto

```
robot/
├── navegation/
│   ├── basicNav.py          # RobotSkills (8 funciones básicas)
│   ├── advNav.py            # AdvancedNavigation (5 funciones avanzadas)  
│   ├── semNav.py            # SemanticNavigation (8 funciones semánticas)
│   └── demo/                # Tests y demostraciones
│       ├── basicTest.py     # Test navegación básica
│       ├── advTest.py       # Test gestión avanzada
│       ├── semTest.py       # Test navegación semántica
│       ├── test_modules.py  # Verificación de importaciones
│       └── README.md        # Documentación de tests
├── saved_poses.npy          # Poses guardadas (formato NumPy)
└── README.md               # Esta documentación
```

## 🎯 Casos de Uso Comunes

### Guardar y Navegar a Ubicaciones
```python
# 1. Navegar manualmente y guardar
robot.go_to_pose(2.5, 1.0, 1.57)
robot.wait_for_result()
advanced.save_current_pose("cocina", "Centro de la cocina")

# 2. Navegar semánticamente  
semantic.go("cocina")
robot.wait_for_result()

# 3. Acercarse manteniendo distancia
semantic.approach("cocina", approach_distance=0.8)
```

### Patrullaje y Rutas
```python
# Patrullaje simple
semantic.patrol(["entrada", "sala", "cocina"], cycles=3)

# Ruta personalizada con acciones
route = ["cocina", "sala", "oficina", "cocina"]  
semantic.create_route(route, name="ruta_inspeccion")
```

### Gestión de Ubicaciones
```python
# Listar todas las ubicaciones
locations = advanced.list_available_locations()
print(f"Disponibles: {locations}")

# Información detallada
info = advanced.get_pose_info("cocina")
print(f"Cocina: x={info['x']:.2f}, y={info['y']:.2f}")
```

## 🛠️ Cómo Ejecutar y Probar

### Tests Especializados por Módulo
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

### Ejecutar Tests en Secuencia
```bash
# Probar todo el sistema (21 funciones)
python3 basicTest.py && python3 advTest.py && python3 semTest.py
```

## 💾 Gestión de Datos

### Archivo de Poses
- **`saved_poses.npy`** - Ubicaciones guardadas en formato NumPy (binario eficiente)

### Estructura de datos interna (diccionario Python):
```python
# Los datos se almacenan en .npy pero internamente son un diccionario:
{
  "cocina": {
    "x": 2.5,
    "y": 1.0, 
    "yaw": 1.57,
    "yaw_degrees": 89.9,
    "description": "Centro de la cocina",
    "timestamp": "2025-09-29 14:30:15"
  },
  "sala": {
    "x": -1.0,
    "y": 3.0,
    "yaw": 0.0, 
    "yaw_degrees": 0.0,
    "description": "Frente al sofá",
    "timestamp": "2025-09-29 14:32:08"
  }
}

# Cómo acceder al archivo:
import numpy as np
saved_poses = np.load("saved_poses.npy", allow_pickle=True).item()
```

## ⚙️ Características Técnicas

### Funciones de Seguridad
- **Cancelación**: `cancel()` detiene navegación inmediatamente
- **Timeouts**: `wait_for_result(timeout)` evita bloqueos indefinidos  
- **Limpieza costmaps**: `clear_costmaps()` resuelve problemas de planificación
- **Rotación no bloqueante**: `rotate()` permite control preciso
- **Verificación de llegada**: `reached()` confirma éxito de navegación

### Optimizaciones
- **Procesamiento paralelo**: `clear_costmaps()` limpia local y global simultáneamente
- **Formato eficiente**: Poses guardadas en `.npy` (NumPy) para velocidad
- **Arquitectura modular**: Separación clara de responsabilidades
- **Manejo de errores**: Validación en cada función crítica

## 📚 Documentación y Referencias

### Archivos del Sistema
- `navegation/basicNav.py` - RobotSkills (8 funciones básicas)
- `navegation/advNav.py` - AdvancedNavigation (5 funciones avanzadas)
- `navegation/semNav.py` - SemanticNavigation (8 funciones semánticas)
- `navegation/demo/README.md` - Guía completa de tests

### Tests Disponibles
- `basicTest.py` - Test navegación básica independiente
- `advTest.py` - Test gestión avanzada independiente
- `semTest.py` - Test navegación semántica independiente
- `test_modules.py` - Verificación de importaciones

## 🏆 Resumen del Sistema

### **Total: 21 funciones especializadas**
- **RobotSkills**: 8 funciones básicas
- **AdvancedNavigation**: 5 funciones avanzadas  
- **SemanticNavigation**: 8 funciones semánticas

### Capacidades Principales
- ✅ **Navegación por coordenadas** (básico)
- ✅ **Navegación por nombres** (semántico)  
- ✅ **Gestión de poses persistente** (avanzado)
- ✅ **Control completo de movimiento** (básico)
- ✅ **Patrullaje automático** (semántico)
- ✅ **Limpieza de costmaps** (avanzado)
- ✅ **Approach inteligente** (semántico)
- ✅ **Rotación precisa** (básico + semántico)

### Ventajas Arquitecturales
1. **Modularidad**: Cada módulo es independiente
2. **Escalabilidad**: Fácil agregar nuevas funcionalidades
3. **Mantenibilidad**: Separación clara de responsabilidades
4. **Testabilidad**: Tests especializados por módulo
5. **Profesionalidad**: Código limpio y documentado

---
**Desarrollado para ROS2 Humble con Nav2**  
*Sistema de navegación robótica modular y profesional*
