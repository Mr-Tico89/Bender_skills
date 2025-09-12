"""
Robot Skills Package for ROS2 - ULTRA SIMPLE VERSION

Solo las 4 skills esenciales para empezar.
"""

from .base_skill import BaseSkill, SkillStatus
from .skill_manager import SkillManager

# LAS 4 SKILLS ESENCIALES
from .essential_skills import (
    SimpleMoveSkill,
    SimpleRotateSkill, 
    SimpleStopSkill,
    SimpleWaitSkill
)

__all__ = [
    # Framework básico
    'BaseSkill',
    'SkillStatus', 
    'SkillManager',
    
    # Las 4 skills esenciales
    'SimpleMoveSkill',
    'SimpleRotateSkill',
    'SimpleStopSkill', 
    'SimpleWaitSkill'
]
