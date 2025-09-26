# Robot Skills - Navegación Completa para ROS2

## Sistema Modular de Navegación

El robot tiene un **sistema modular** con tres componentes especializados:

### **RobotSkills** (Navegación Básica)
1. **`where_am_i()`** - Obtener posición y orientación actual
2. **`go_to_pose(x, y, yaw)`** - Navegación a coordenadas específicas
3. **`save_current_pose(name)`** - Guardar ubicaciones con nombre
4. **`is_moving()`** - Detectar si el robot está en movimiento
5. **`reached()`** - Verificar si llegó al objetivo
6. **`cancel()`** - Cancelar navegación actual
7. **`rotate()`** - Rotación en el lugar
8. **`clear_costmaps()`** - Limpiar mapas de navegación
9. **`wait_for_result()`** - Esperar resultado de navegación

### **SemanticNavigation** (Navegación Inteligente)
1. **`go(location_name)`** - Ir a ubicación por nombre
2. **`approach(location, distance)`** - Acercarse manteniendo distancia
3. **`look(location_name)`** - Mirar hacia ubicación nombrada
4. **`patrol(locations, cycles)`** - Patrullaje automático
5. **`create_route(waypoints)`** - Ejecutar rutas personalizadas
6. **`find_nearest_location()`** - Encontrar ubicación más cercana

### **AdvancedNavigation** (Funciones Avanzadas)
1. **`go_to_pose_stamped(pose)`** - Navegación con PoseStamped completo
2. **`look_to_pose(x, y)`** - Mirar hacia coordenadas específicas
3. **`rotate_improved()`** - Rotación mejorada no bloqueante
4. **`navigate_with_intermediate_goals()`** - Navegación por waypoints
5. **`orbit_around_point()`** - Crear órbitas circulares
6. **`spiral_navigation()`** - Navegación en espiral
7. **`figure_eight_pattern()`** - Patrón figura 8

## Ejemplo de Uso Modular

### Navegación Básica:
```python
import rclpy
from skills import RobotSkills

rclpy.init()
robot = RobotSkills()

# 1. Navegación básica por coordenadas
pose = robot.where_am_i()
robot.go_to_pose(2.0, 1.0, 0.0)
robot.wait_for_result()

# 2. Guardar ubicaciones importantes
robot.save_current_pose("mi_base", "Punto de inicio")

# 3. Control de navegación
if robot.is_moving():
    print("Robot en movimiento")

if robot.reached():
    print("Llegué al destino!")

robot.destroy_node()
rclpy.shutdown()
```

### Navegación Semántica:
```python
import rclpy
from skills import RobotSkills, SemanticNavigation

rclpy.init()
robot = RobotSkills()
semantic = SemanticNavigation(robot)

# 1. Navegación por nombres
semantic.go("cocina")
robot.wait_for_result()

# 2. Acercarse manteniendo distancia
semantic.approach("mesa", approach_distance=0.8)

# 3. Mirar hacia ubicaciones
semantic.look("ventana")

# 4. Patrullaje automático
semantic.patrol(["cocina", "salon", "entrada"], cycles=3)

# 5. Rutas personalizadas
waypoints = ["punto_a", "punto_b", "punto_c"]
semantic.create_route(waypoints, name="ruta_inspeccion")

robot.destroy_node()
rclpy.shutdown()
```

### Navegación Avanzada:
```python
import rclpy
from skills import RobotSkills, AdvancedNavigation

rclpy.init()
robot = RobotSkills()
advanced = AdvancedNavigation(robot)

# 1. Mirar hacia coordenadas específicas
advanced.look_to_pose(3.0, 2.0)

# 2. Navegación por waypoints
waypoints = [
    {'x': 1.0, 'y': 0.0, 'yaw': 0.0},
    {'x': 1.0, 'y': 1.0, 'yaw': 1.57},
    {'x': 0.0, 'y': 1.0, 'yaw': 3.14}
]
advanced.navigate_with_intermediate_goals(waypoints)

# 3. Crear órbita circular alrededor de un punto
advanced.orbit_around_point(center_x=0.0, center_y=0.0, radius=2.0)

# 4. Navegación en espiral
advanced.spiral_navigation(center_x=0.0, center_y=0.0, max_radius=3.0, turns=2)

# 5. Patrón figura 8
advanced.figure_eight_pattern(center_x=0.0, center_y=0.0, width=2.0, height=1.5)

robot.destroy_node()
rclpy.shutdown()
```

## Navegación y Poses

```python
# Guardar ubicación actual
robot.save_current_pose("mi_base", "Punto de partida")

# Ir a ubicación guardada
robot.go("mi_base")

# Acercarse a una ubicación manteniendo distancia
robot.approach("cocina", approach_distance=1.0)

# Mirar hacia una ubicación
robot.look("salon")
```

## Gestión de Ubicaciones

```python
# Listar ubicaciones disponibles
ubicaciones = robot.list_available_locations()
print(f"Puedo ir a: {ubicaciones}")

# Obtener info detallada de una ubicación
info = robot.get_pose_info("cocina")
print(f"Info cocina: {info}")

# Guardar ubicación actual
robot.save_current_pose("punto_importante", "Lugar especial")
```

## Cómo ejecutar

### Ejecutar ejemplo básico:
```bash
python3 ejemplo.py
```

### Ejecutar demo modular (recomendado):
```bash
python3 demo_modular_navigation.py
```

### Ejecutar navegación semántica independiente:
```bash
python3 skills/semantic_navigation.py
```

## Archivos de Configuración

El robot crea automáticamente:

- **`saved_poses.json`** - Ubicaciones guardadas con nombres

**Nota:** El mapeo de habitaciones y detección de objetos lo maneja el componente de conocimiento desarrollado por tu compañero.

### Ejemplo de `saved_poses.json`:
```json
{
  "cocina": {
    "x": 2.5,
    "y": 1.0,
    "yaw": 1.57,
    "description": "Centro de la cocina",
    "timestamp": 1640995200.0
  },
  "salon": {
    "x": -1.0,
    "y": 3.0, 
    "yaw": 0.0,
    "description": "Frente al sofá",
    "timestamp": 1640995260.0
  }
}
```

## Personalización

### Agregar ubicaciones:
```python
# Mover robot a ubicación deseada y guardar
robot.go_to_pose(x, y, yaw)
robot.wait_for_result()
robot.save_current_pose("mi_lugar", "Descripción del lugar")
```

## Funciones de Seguridad

- **Cancelación automática**: `cancel()` detiene navegación y movimiento
- **Limpieza de costmaps**: `clear_costmaps()` resuelve problemas de navegación
- **Timeouts**: `wait_for_result()` evita bloqueos indefinidos

## Documentación Completa

Para más detalles, consulta:
- `skills/skills.py` - Navegación básica (RobotSkills)
- `skills/semantic_navigation.py` - Navegación semántica avanzada  
- `demo_modular_navigation.py` - Demo del sistema modular
- `SKILLS_GUIDE.md` - Guía completa de todas las skills

## Sistema Modular Completo

### Arquitectura Modular:
```
Robot Navigation System
├── RobotSkills (skills.py)
│   └── Navegación básica y control fundamental
└── SemanticNavigation (semantic_navigation.py)
    └── Navegación inteligente y comportamientos complejos
```

### Capacidades del Sistema:
- **Navegación básica** por coordenadas
- **Navegación semántica** por nombres
- **Patrullaje automático** entre ubicaciones
- **Rutas personalizadas** con waypoints
- **Approach inteligente** con distancia controlada
- **Gestión de poses** persistente
- **Control de navegación** completo (cancelar, esperar, verificar)
- **Sistema modular** fácil de extender

### Ventajas de la Modularidad:
1. **Separación clara** de responsabilidades
2. **Fácil mantenimiento** y debugging
3. **Extensibilidad** - agregar nuevas funcionalidades sin tocar el código básico
4. **Reutilización** - usar componentes independientemente
5. **Integración limpia** con el sistema de mapeo de tu compañero

**Nota:** El mapeo de habitaciones, detección de objetos y conocimiento del entorno lo maneja el componente desarrollado por tu compañero de equipo.