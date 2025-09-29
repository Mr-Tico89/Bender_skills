"""
Robot Skills Package for ROS2 - Modular Navigation System

Arquitectura modular organizada por complejidad:
- RobotSkills: Funciones básicas de navegación
- SemanticNavigation: Navegación inteligente por nombres
- AdvancedNavigation: Gestión de poses y funciones avanzadas
"""

from .basicNav import RobotSkills
from .semNav import SemanticNavigation
from .advNav import AdvancedNavigation

__all__ = [
    # Navegación básica (coordenadas, movimiento, estado)
    'RobotSkills',
    
    # Navegación semántica (nombres, rutas, patrullaje)
    'SemanticNavigation',
    
    # Gestión avanzada (poses, costmaps, funciones complejas)
    'AdvancedNavigation'
]
