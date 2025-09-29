# Demos de Navegación Robótica

Esta carpeta contiene todos los ejemplos y demostraciones del sistema de navegación modular.

## 📁 Guía de Demos

### **🚀 Para empezar - Demo Básico:**
```bash
python3 demo_skills_complete.py
```
**Qué muestra:** Funciones básicas de RobotSkills (where_am_i, go_to_pose, rotate, etc.)

### **🧠 Demo Navegación Semántica:**
```bash
python3 demo_modular_navigation.py
```
**Qué muestra:** SemanticNavigation (go, approach, patrol, create_route)

### **⚙️ Demo Sistema Completo:**
```bash
python3 demo_complete_modular.py
```
**Qué muestra:** Los tres módulos trabajando juntos (RobotSkills + SemanticNavigation + AdvancedNavigation)

### **🔧 Test de Importaciones:**
```bash
python3 test_modules.py
```
**Qué muestra:** Verifica que todos los módulos se importen correctamente

## 🏗️ Arquitectura del Sistema

```
RobotSkills (Básico)
├── where_am_i()          # Posición actual
├── go_to_pose()          # Navegación por coordenadas
├── is_moving()           # Estado de movimiento
├── reached()             # Verificar llegada
├── rotate()              # Rotación mejorada
├── cancel()              # Cancelar navegación
└── wait_for_result()     # Esperar resultado

SemanticNavigation (Semántico)
├── go()                  # Navegación por nombres
├── approach()            # Acercarse con distancia
├── look()                # Mirar hacia ubicación
├── patrol()              # Patrullaje automático
└── create_route()        # Rutas personalizadas

AdvancedNavigation (Avanzado)
├── save_current_pose()   # Guardar poses
├── load_saved_poses()    # Cargar poses
├── list_available_locations()  # Listar ubicaciones
├── get_pose_info()       # Info de poses
└── clear_costmaps()      # Limpiar costmaps
```

## 🎯 Cómo usar en tu código

```python
from navegation.basicNav import RobotSkills
from navegation.semNav import SemanticNavigation  
from navegation.advNav import AdvancedNavigation

# Crear instancias
robot = RobotSkills()
semantic = SemanticNavigation(robot)
advanced = AdvancedNavigation(robot)

# Usar las funciones
robot.go_to_pose(2.0, 1.0, 0.0)        # Básico
semantic.go("cocina")                    # Semántico
advanced.save_current_pose("mi_pos")     # Avanzado
```

## ⚠️ Requisitos

- ROS2 (Humble/Iron)
- Nav2 stack
- Mapa válido cargado
- Robot con navegación configurada

## 🚀 Ejecución rápida

```bash
# Demo todo-en-uno (recomendado)
python3 demo_complete_modular.py

# O ejecutar demos individuales según necesidad
```