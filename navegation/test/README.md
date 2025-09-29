# Tests del Sistema de Navegación Robótica

## Descripción

Este directorio contiene **tests** para cada módulo del sistema de navegación, permitiendo probar funciones específicas de manera independiente.

## Tests Especializados por Módulo

### `basicTest.py` - RobotSkills (Navegación Básica)
**Prueba las 8 funciones básicas:**

```bash
python3 basicTest.py
```

**Funciones probadas:**
1. `where_am_i()` - Obtener posición actual
2. `is_moving()` - Verificar si está en movimiento  
3. `go_to_pose()` - Navegar a coordenadas
4. `wait_for_result()` - Esperar resultado de navegación
5. `reached()` - Verificar si llegó al destino
6. `rotate()` - Rotar con tiempo específico
7. `stop()` - Detener movimiento
8. `cancel()` - Cancelar navegación actual

### `advTest.py` - AdvancedNavigation (Gestión Avanzada)  
**Prueba las 5 funciones avanzadas:**

```bash
python3 advTest.py
```

**Funciones probadas:**
1. `save_current_pose()` - Guardar posición actual
2. `load_saved_poses()` - Cargar poses guardadas
3. `list_available_locations()` - Listar ubicaciones disponibles
4. `get_pose_info()` - Obtener información de pose
5. `clear_costmaps()` - Limpiar mapas de costo

### `semTest.py` - SemanticNavigation (Navegación Semántica)
**Prueba las 6 funciones semánticas:**

```bash
python3 semTest.py
```

**Funciones probadas:**
1. `get_available_locations()` - Obtener ubicaciones disponibles
2. `go()` - Navegar a ubicación por nombre
3. `approach()` - Acercarse manteniendo distancia
4. `look()` - Mirar hacia ubicación
5. `patrol()` - Patrullar ruta con ciclos
6. `create_route()` - Crear ruta personalizada


## Uso Recomendado
### Durante Desarrollo
```bash
# Desarrollando funciones básicas
python3 basicTest.py

# Desarrollando gestión avanzada  
python3 advTest.py

# Desarrollando navegación semántica
python3 semTest.py
```

### Verificación Rápida
```bash
# Verificar que todos los módulos se importen correctamente
python3 test_modules.py
```

### Para Pruebas Completas
```bash
# Ejecutar todos los tests en secuencia
python3 basicTest.py && python3 advTest.py && python3 semTest.py
```

## Estructura de Cada Test

1. **Inicialización**: Configura el módulo específico
2. **Pruebas sistemáticas**: Cada función probada individualmente
3. **Verificación**: Confirmación de resultados
4. **Resumen**: Estadísticas del módulo probado
5. **Limpieza**: Liberación correcta de recursos

## Requisitos

- ROS2 Humble
- Nav2 activo y configurado
- Mapa cargado
- Módulos correspondientes funcionando

## Notas

- Cada test es **independiente** y puede ejecutarse por separado
- **semTest.py** crea ubicaciones de prueba automáticamente
- Los tests son **sistemáticos** y muestran resultados detallados
- Código **profesional** sin emojis innecesarios en la lógica