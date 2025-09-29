#!/usr/bin/env python3
"""
Prueba rápida del sistema modular de navegación
Verifica que los tres módulos se importen correctamente
"""

try:
    from skills import RobotSkills, SemanticNavigation, AdvancedNavigation
    print("✓ Importación exitosa de los tres módulos:")
    print("  - RobotSkills (navegación básica)")
    print("  - SemanticNavigation (navegación semántica)")  
    print("  - AdvancedNavigation (funciones avanzadas)")
    
    # Verificar métodos básicos
    print("\n✓ Métodos disponibles en RobotSkills:")
    basic_methods = [m for m in dir(RobotSkills) if not m.startswith('_')]
    print(f"  Total: {len(basic_methods)} métodos")
    
    print("\n✓ Métodos disponibles en SemanticNavigation:")
    semantic_methods = [m for m in dir(SemanticNavigation) if not m.startswith('_')]
    print(f"  Total: {len(semantic_methods)} métodos")
    
    print("\n✓ Métodos disponibles en AdvancedNavigation:")
    advanced_methods = [m for m in dir(AdvancedNavigation) if not m.startswith('_')]
    print(f"  Total: {len(advanced_methods)} métodos")
    
    print("\n🎉 Sistema modular listo para usar!")
    print("\nPara usar en tu código:")
    print("```python")
    print("from skills import RobotSkills, SemanticNavigation, AdvancedNavigation")
    print("robot = RobotSkills()")
    print("semantic = SemanticNavigation(robot)")
    print("advanced = AdvancedNavigation(robot)")
    print("```")
    
except ImportError as e:
    print(f"❌ Error de importación: {e}")
except Exception as e:
    print(f"❌ Error inesperado: {e}")