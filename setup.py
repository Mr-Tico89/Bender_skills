from setuptools import setup, find_packages

package_name = 'bender_skills'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        # Core ROS2
        'rclpy',
        # Navigation and geometry
        'nav2_simple_commander',
        'nav2_msgs', 
        'geometry_msgs',
        # Transformations
        'tf_transformations',
        'tf2_ros',
        'tf2_geometry_msgs',
        # Services
        'std_srvs',
        # Scientific computing
        'numpy',
    ],
    zip_safe=True,
    description='Sistema de navegación robótica modular para ROS2 - Bender Skills con arquitectura de 3 capas',
    license='MIT',
    maintainer='Robotica Team',
    maintainer_email='robotica@bender.com',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Tests especializados por módulo
            'basic_test = navegation.demo.basicTest:main',
            'adv_test = navegation.demo.advTest:main', 
            'sem_test = navegation.demo.semTest:main',
            # Verificación de módulos
            'test_modules = navegation.demo.test_modules:main',
            # Módulos principales (para importación directa)
            'basic_nav = navegation.basicNav:main',
            'sem_nav = navegation.semNav:main',
            'adv_nav = navegation.advNav:main',
        ],
    },
)
