from setuptools import setup

package_name = 'ros2_grbl_ramps_service'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/grbl_bridge.launch.py']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
    ],
    install_requires=['setuptools','pyserial'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ROS 2 services to control GRBL-Mega-5X on Arduino Mega + RAMPS 1.4',
    license='MIT',
    entry_points={
        'console_scripts': [
            'grbl_bridge_node.py = ros2_grbl_ramps_service.grbl_bridge_node:main',
            'ultrasonic_guard_node.py = ros2_grbl_ramps_service.ultrasonic_guard_node:main',
        ],
    },
)
