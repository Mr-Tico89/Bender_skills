# Guía de Skills de ROS2 para Principiantes

Esta guía explica cada skill que hemos creado, desde las más básicas hasta las más avanzadas, perfecto para alguien que empieza con ROS2.

## 🎯 Conceptos Fundamentales

### ¿Qué es una Skill?
Una **skill** es una habilidad específica que puede realizar tu robot, como:
- Moverse a un lugar
- Detectar objetos
- Rotar hacia una dirección
- Guardar su posición actual

### ¿Por qué usar Skills?
- **Reutilizable**: Una vez creada, puedes usarla en muchos lugares
- **Modular**: Cada skill hace una cosa específica
- **Combinable**: Puedes combinar skills simples para crear comportamientos complejos

---

## 📍 Skills de Navegación Básica

### 1. WhereAmISkill - "¿Dónde estoy?"

**¿Qué hace?** Obtiene la posición actual del robot en el mapa.

**Conceptos de ROS2:**
- **TF2**: Sistema de coordenadas de ROS2
- **Frames**: Sistemas de referencia (ej: "map", "base_link")
- **Transformadas**: Conversión entre sistemas de coordenadas

**Ejemplo de uso:**
```python
skill = WhereAmISkill()
if skill.execute():
    pos = skill.get_result()
    print(f"Estoy en x={pos['x']}, y={pos['y']}, orientación={pos['yaw_degrees']}°")
```

**Cuándo usarla:**
- Antes de planificar un movimiento
- Para logging/debugging
- Como base para otras skills

---

### 2. SavePositionSkill - "Guardar esta ubicación"

**¿Qué hace?** Guarda la posición actual con un nombre para usarla después.

**Conceptos de ROS2:**
- **Persistencia de datos**: Guardar información entre ejecuciones
- **JSON**: Formato estándar para guardar datos estructurados

**Ejemplo de uso:**
```python
skill = SavePositionSkill()
params = {'name': 'cocina', 'description': 'Cerca del refrigerador'}
skill.execute(params)
```

**Cuándo usarla:**
- Para crear waypoints importantes
- Para recordar lugares de interés
- Para crear rutas de patrullaje

---

### 3. IsMovingSkill - "¿Me estoy moviendo?"

**¿Qué hace?** Detecta si el robot está en movimiento actualmente.

**Conceptos de ROS2:**
- **Tópicos**: Canales de comunicación entre nodos
- **Suscripción**: Escuchar mensajes de otros nodos
- **Twist**: Tipo de mensaje para velocidades (linear + angular)

**Ejemplo de uso:**
```python
skill = IsMovingSkill()
if skill.execute():
    result = skill.get_result()
    if result['is_moving']:
        print("¡Estoy en movimiento!")
    else:
        print("Estoy parado")
```

**Cuándo usarla:**
- Antes de ejecutar una nueva acción de movimiento
- Para verificar que una acción se ejecutó correctamente
- Para safety checks

---

### 4. CancelNavigationSkill - "¡Parar ahora!"

**¿Qué hace?** Detiene inmediatamente el robot y cancela navegación.

**Conceptos de ROS2:**
- **Safety**: Mecanismos de seguridad en robótica
- **Publishers**: Enviar mensajes a tópicos
- **Emergency stop**: Parada de emergencia

**Ejemplo de uso:**
```python
skill = CancelNavigationSkill()
skill.execute({'emergency': True})  # Parada de emergencia
```

**Cuándo usarla:**
- Cuando detectas un obstáculo peligroso
- Para implementar botones de parada de emergencia
- Al inicio de otras skills para asegurar estado limpio

---

### 5. SimpleRotateSkill - "Girar X grados"

**¿Qué hace?** Rota el robot un ángulo específico.

**Conceptos de ROS2:**
- **Control de movimiento**: Enviar comandos de velocidad
- **Timing**: Calcular tiempo basado en velocidad y distancia
- **Grados vs Radianes**: Conversión entre unidades

**Ejemplo de uso:**
```python
skill = SimpleRotateSkill()
params = {
    'angle_degrees': 90.0,    # Girar 90° a la izquierda
    'angular_speed': 30.0     # A 30°/segundo
}
skill.execute(params)
```

**Cuándo usarla:**
- Para orientar el robot hacia un objetivo
- Como parte de secuencias de movimiento
- Para exploración básica

---

## 🎯 Skills de Navegación con Poses

### 6. GoToPoseSkill - "Ir a coordenada X,Y"

**¿Qué hace?** Navega el robot a una posición específica en el mapa.

**Conceptos de ROS2:**
- **Action Servers**: Servicios de larga duración con feedback
- **Nav2**: Stack de navegación de ROS2
- **Poses**: Posición + orientación en el espacio

**Ejemplo de uso:**
```python
skill = GoToPoseSkill()
params = {
    'x': 2.0,
    'y': 1.5,
    'yaw_degrees': 45.0,
    'timeout': 60.0
}
skill.execute(params)
```

**Cuándo usarla:**
- Para navegación autónoma
- Para ir a estaciones de trabajo específicas
- Como base para navegación semántica

---

### 7. ReachedSkill - "¿Llegué al objetivo?"

**¿Qué hace?** Verifica si el robot está en una posición objetivo.

**Conceptos de ROS2:**
- **Tolerancias**: Aceptar aproximaciones en lugar de exactitud
- **Validación**: Verificar resultados de acciones

**Ejemplo de uso:**
```python
skill = ReachedSkill()
params = {
    'target_x': 2.0,
    'target_y': 1.5,
    'position_tolerance': 0.3,  # 30cm de tolerancia
    'angle_tolerance': 10.0     # 10° de tolerancia
}
if skill.execute(params):
    if skill.get_result()['reached']:
        print("¡Llegué al objetivo!")
```

**Cuándo usarla:**
- Después de comandos de navegación
- Para validar posicionamiento
- Para decidir próximas acciones

---

### 8. LookToPoseSkill - "Mirar hacia un punto"

**¿Qué hace?** Rota el robot para mirar hacia coordenadas específicas.

**Conceptos de ROS2:**
- **Trigonometría**: Calcular ángulos entre puntos
- **atan2**: Función para calcular ángulos correctamente

**Ejemplo de uso:**
```python
skill = LookToPoseSkill()
params = {
    'target_x': 0.0,         # Mirar hacia el origen
    'target_y': 0.0,
    'angular_speed': 45.0    # 45°/segundo
}
skill.execute(params)
```

**Cuándo usarla:**
- Para orientar sensores hacia objetivos
- Antes de tomar fotos/mediciones
- Para interacciones con humanos

---

## 👁️ Skills de Percepción Básica

### 9. ObjectDetectionSkill - "¿Qué veo?"

**¿Qué hace?** Detecta objetos en las imágenes de la cámara.

**Conceptos de ROS2:**
- **Computer Vision**: Procesamiento de imágenes
- **OpenCV**: Librería estándar para visión computacional
- **HSV**: Espacio de color mejor para detectar colores específicos

**Ejemplo de uso:**
```python
skill = ObjectDetectionSkill()
params = {
    'duration': 5.0,
    'color_range': {
        'lower': [0, 50, 50],    # Rojo en HSV
        'upper': [10, 255, 255]
    },
    'min_area': 1000
}
if skill.execute(params):
    objects = skill.get_result()['objects']
    print(f"Detecté {len(objects)} objetos rojos")
```

**Cuándo usarla:**
- Para exploración de entornos
- Como base para otras skills de percepción
- Para tareas de pick & place

---

### 10. CountObjectsSkill - "¿Cuántos hay?"

**¿Qué hace?** Cuenta objetos específicos en la escena.

**Conceptos de ROS2:**
- **Composición**: Usar otras skills como componentes
- **Filtrado**: Aplicar criterios específicos a detecciones

**Ejemplo de uso:**
```python
skill = CountObjectsSkill()
params = {
    'object_type': 'blue_objects',
    'duration': 3.0,
    'min_count_threshold': 2
}
if skill.execute(params):
    count = skill.get_result()['count']
    print(f"Hay {count} objetos azules")
```

**Cuándo usarla:**
- Para inventarios
- Para verificar condiciones ("hay al menos 3 vasos?")
- Para tasks de clasificación

---

### 11. HasDetectionSkill - "¿Hay algo de tipo X?"

**¿Qué hace?** Verifica si hay objetos de un tipo específico.

**Conceptos de ROS2:**
- **Boolean logic**: Respuestas sí/no
- **Confidence scoring**: Niveles de confianza en detecciones

**Ejemplo de uso:**
```python
skill = HasDetectionSkill()
params = {
    'object_type': 'green_objects',
    'min_confidence': 0.7
}
if skill.execute(params):
    has_objects = skill.get_result()['has_confident_detection']
    if has_objects:
        print("¡Sí hay objetos verdes!")
```

**Cuándo usarla:**
- Para decisiones binarias
- Para precondiciones de otras acciones
- Para validación de tareas

---

## 🔧 Skills de Control de Detectores

### 12. StartDetectorSkill - "Encender detector"

**¿Qué hace?** Inicializa detectores específicos.

**Conceptos de ROS2:**
- **Resource management**: Gestión de recursos computacionales
- **Model loading**: Cargar modelos de ML

**Ejemplo de uso:**
```python
skill = StartDetectorSkill()
params = {
    'detector_type': 'yolo',
    'model_path': '/path/to/model.pt',
    'confidence_threshold': 0.6
}
skill.execute(params)
```

**Cuándo usarla:**
- Al inicio de aplicaciones de visión
- Para cambiar entre diferentes detectores
- Para optimizar recursos computacionales

---

### 13. StopDetectorSkill - "Apagar detector"

**¿Qué hace?** Detiene detectores para liberar recursos.

**Ejemplo de uso:**
```python
skill = StopDetectorSkill()
skill.execute({'detector_type': 'all'})  # Detener todos
```

**Cuándo usarla:**
- Al final de aplicaciones
- Para cambiar configuraciones
- Para liberar memoria/CPU

---

## 🚀 Cómo Usar el Framework

### Patrón Básico:
```python
import rclpy
from skills import NombreDeLaSkill

rclpy.init()

# 1. Crear la skill
skill = NombreDeLaSkill()

# 2. Definir parámetros
params = {'parametro1': valor1, 'parametro2': valor2}

# 3. Ejecutar
if skill.execute(params):
    # 4. Obtener resultado
    result = skill.get_result()
    print(f"Resultado: {result}")
else:
    print(f"Error: {skill.get_error_message()}")

# 5. Cleanup
skill.destroy_node()
rclpy.shutdown()
```

### Usando Skill Manager:
```python
import rclpy
from skills import SkillManager, WhereAmISkill

rclpy.init()

# 1. Crear manager
manager = SkillManager()

# 2. Registrar skills
manager.register_skill(WhereAmISkill, "where_am_i")

# 3. Ejecutar
manager.execute_skill("where_am_i", {})

# 4. Monitorear
status = manager.get_skill_status("where_am_i")
print(f"Estado: {status}")

# 5. Cleanup
manager.cleanup()
rclpy.shutdown()
```

## 📚 Próximos Pasos

1. **Práctica**: Ejecuta el script `demo_new_skills.py`
2. **Experimenta**: Modifica parámetros de las skills
3. **Extiende**: Crea tus propias skills heredando de `BaseSkill`
4. **Combina**: Usa múltiples skills para crear comportamientos complejos

## 🔍 Skills Faltantes (para implementar después)

- `GetCurrentRoomsSkill`: Obtener habitaciones del mapa
- `IsRobotInRoomSkill`: Verificar si está en habitación específica
- `IsRobotInMapSkill`: Verificar si está dentro del mapa
- `ClearCostmapsSkill`: Limpiar mapas de costos de Nav2
- `ApproachSkill`: Acercarse a ubicaciones
- `GoSemanticSkill`: Navegación semántica ("ir a cocina")

Estas son más avanzadas y requieren integración con mapas semánticos y Nav2.

---

¡Con estas skills básicas ya puedes crear robots muy capaces! 🎉
