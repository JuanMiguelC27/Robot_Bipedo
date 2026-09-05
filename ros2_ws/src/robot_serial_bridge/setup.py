from setuptools import find_packages, setup

package_name = 'robot_serial_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='david',
    maintainer_email='david.collazos_her@uao.edu.co',
    description='Puente serial-ROS: publica las posiciones del firmware control_pata en /servo_states',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'serial_bridge = robot_serial_bridge.serial_bridge:main',
        ],
    },
)
