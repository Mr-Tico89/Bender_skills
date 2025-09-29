"""
Robot Skills Package for ROS2 - Modular Navigation System

Arquitectura modular organizada por complejidad:
- RobotSkills: Funciones básicas de navegación
- SemanticNavigation: Navegación inteligente por nombres
- AdvancedNavigation: Gestión de poses y funciones avanzadas
"""

from .skills import RobotSkills
from .semantic_navigation import SemanticNavigation
from .advanced_navigation import AdvancedNavigation

__all__ = [
    # Navegación básica (coordenadas, movimiento, estado)
    'RobotSkills',
    
    # Navegación semántica (nombres, rutas, patrullaje)
    'SemanticNavigation',
    
    # Gestión avanzada (poses, costmaps, funciones complejas)
    'AdvancedNavigation'
]
