# Guía de Skills de ROS2 

Solo las 4 skills **más esenciales** para aprender ROS2.

## ¿Qué es una Skill?

Una **skill** es algo que tu robot sabe hacer. Solo necesitas 4 básicas:
- **Mover** hacia adelante
- **Girar** izquierda/derecha  
- **Parar** inmediatamente
- **Esperar** unos segundos

## Las 4 Skills Esenciales

### 1. `SimpleMoveSkill` - Mover hacia adelante
```python
skill = SimpleMoveSkill()
skill.execute({'distance': 1.0})  # Mover 1 metro
```

### 2. `SimpleRotateSkill` - Girar
```python
skill = SimpleRotateSkill()
skill.execute({'angle_degrees': 90.0})   # Girar 90° izquierda
skill.execute({'angle_degrees': -90.0})  # Girar 90° derecha
```

### 3. `SimpleStopSkill` - Parar ahora
```python
skill = SimpleStopSkill()
skill.execute()  # ¡Para inmediatamente!
```

### 4. `SimpleWaitSkill` - Esperar
```python
skill = SimpleWaitSkill()
skill.execute({'seconds': 2.0})  # Esperar 2 segundos
```

## Cómo Usar

### Opción 1: Demo Ultra Simple
```bash
cd /home/robotica/Desktop/robot/skills
python3 ultra_simple_demo.py
```

### Opción 2: Usar una skill
```bash
python3 -c "
import rclpy
from essential_skills import SimpleMoveSkill

rclpy.init()
skill = SimpleMoveSkill()
skill.execute({'distance': 0.5})  # Mover 50cm
skill.destroy_node()
rclpy.shutdown()
print('¡Listo!')
"
```

### Opción 3: Mi primer robot cuadrado 
```python
#!/usr/bin/env python3
import rclpy
from essential_skills import SimpleMoveSkill, SimpleRotateSkill, SimpleWaitSkill

rclpy.init()

# Crear las 3 skills que necesito
move = SimpleMoveSkill()
rotate = SimpleRotateSkill() 
wait = SimpleWaitSkill()

# ¡Hacer un cuadrado!
for i in range(4):
    print(f"Lado {i+1} del cuadrado...")
    move.execute({'distance': 1.0})      # Mover 1 metro
    wait.execute({'seconds': 1.0})       # Pausa 1 segundo
    rotate.execute({'angle_degrees': 90.0})  # Girar 90°
    wait.execute({'seconds': 1.0})       # Pausa 1 segundo

print("¡Cuadrado completado! ")

# Limpiar
move.destroy_node()
rotate.destroy_node()
wait.destroy_node()
rclpy.shutdown()
```

##  Lo Que Aprendes con Solo 4 Skills

### 1. **Nodos ROS2**
- Cada skill es un "nodo" 
- Los nodos se comunican entre sí

### 2. **El Tópico `/cmd_vel`**
- Es donde mandas órdenes de movimiento
- Todas las skills de movimiento lo usan

### 3. **Mensajes `Twist`**
- Velocidad linear (adelante/atrás)
- Velocidad angular (girar izq/der)

### 4. **Ciclo básico ROS2**
- `rclpy.init()` → Empezar
- Crear nodos y usar skills
- `destroy_node()` → Limpiar
- `rclpy.shutdown()` → Terminar

## Cómo Funciona una Skill (Super Simple)

```python
class MiSkill(BaseSkill):
    def __init__(self):
        super().__init__("mi_skill")
        # Configurar publishers/subscribers
    
    def execute(self, parameters=None):
        # ¡Hacer algo útil aquí!
        return True
    
    def stop(self):
        # Parar si es necesario
        return True
```

## Ejercicios Súper Fáciles

### Ejercicio 1: Línea Recta 
```python
# 1. Mover 2 metros
# 2. Parar
# 3. ¡Listo!
```

### Ejercicio 2: Giro Completo 
```python
# 1. Girar 90° (4 veces)
# 2. Esperar 1 segundo entre giros
# 3. ¡Una vuelta completa!
```

### Ejercicio 3: Robot Nervioso 
```python
# 1. Mover un poquito
# 2. Parar
# 3. Girar un poquito
# 4. Repetir
```

## Preguntas Frecuentes

**P: ¿Solo 4 skills? ¿No es muy poco?**
R: ¡No! Con estas 4 ya puedes hacer patrones, figuras, secuencias. Es perfecto para empezar.

**P: ¿Funcionará sin un robot real?**
R: Sí, puedes ejecutar todo y ver los mensajes. Los conceptos se aprenden igual.

**P: ¿Cómo agrego más skills después?**
R: Cuando domines estas 4, puedes agregar perception, navegación, etc.

**P: ¿Qué significa cada parámetro?**
R: `distance` en metros, `angle_degrees` en grados, `seconds` en segundos.

## Siguientes Pasos

1. **Ejecuta**: `python3 ultra_simple_demo.py`
2. **Lee**: Abre `essential_skills.py` 
3. **Modifica**: Cambia distancias y ángulos
4. **Crea**: Haz patrones (cuadrado, triángulo, estrella)
5. **Expande**: Cuando ya domines, agrega más skills

## ¡Felicidades!

Con solo **4 skills esenciales** ya puedes:
- Mover un robot en cualquier dirección
- Crear secuencias y patrones
- Entender los fundamentos de ROS2
- Construir robots más complejos después


