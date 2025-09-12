# Robot Skills - Versión Simple 🤖

## ¿Qué puede hacer mi robot?

Tu robot tiene **6 skills básicas**:

1. **`where_am_i()`** - ¿Dónde estoy?
2. **`go_to_pose(x, y, yaw)`** - Ir a un punto
3. **`cancel()`** - Cancelar movimiento
4. **`is_moving()`** - ¿Me estoy moviendo?
5. **`reached()`** - ¿Llegué a la meta?
6. **`rotate(velocidad, tiempo)`** - Rotar en el lugar

## Ejemplo súper fácil

```python
import rclpy
from skills.skills import RobotSkills

# Crear robot
rclpy.init()
robot = RobotSkills()

# Ver dónde estoy
robot.where_am_i()

# Ir a un punto
robot.go_to_pose(2.0, 1.0)

# Esperar a llegar
while robot.is_moving():
    rclpy.spin_once(robot, timeout_sec=0.1)

# ¿Llegué?
if robot.reached():
    print("¡Llegué!")
    robot.rotate(0.5, 2.0)  # Rotar 2 segundos

# Limpiar
robot.destroy_node()
rclpy.shutdown()
```

## Cómo ejecutar

```bash
python3 ejemplo_simple.py
```

## Tips para principiantes

- **Siempre** usa `rclpy.init()` al inicio
- **Siempre** usa `robot.destroy_node()` y `rclpy.shutdown()` al final
- Para ir a un punto: `robot.go_to_pose(x, y)`
- Para rotar: `robot.rotate(velocidad, tiempo)`
- Las coordenadas (x, y) están en metros
- La velocidad de rotación está en radianes/segundo

¡Ya está! Súper simple 😊
