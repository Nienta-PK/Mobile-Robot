from setuptools import setup, find_packages
package_name = 'ired_aruco'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ired_aruco.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tata',
    maintainer_email='tata@example.com',
    description='ArUco detector publishing IDs and 3D poses',
    license='MIT',
    entry_points={
        'console_scripts': [
            'aruco_node = ired_aruco.aruco_node:main',
        ],
    },
)
