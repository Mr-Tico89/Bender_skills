from setuptools import setup, find_packages

package_name = 'robot_skills'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'geometry_msgs',
        'nav2_simple_commander',
        'tf_transformations',
    ],
    zip_safe=True,
    description='Skills simples para robot en ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_skills = skills.skills:main',
            'ejemplo_simple = ejemplo_simple:main',
        ],
    },
)
