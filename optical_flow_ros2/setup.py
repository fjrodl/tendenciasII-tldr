from setuptools import setup

package_name = 'optical_flow_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name +'/launch', ['launch/optical_flow.launch.py' ]),
        ('share/' + package_name +'/launch', ['launch/optical_flow_rviz.launch.py' ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fjrodl',
    maintainer_email='fjrodl@unileon.es',
    description='Nodo ROS 2 para Optical Flow + Odometry',
    license='MIT',
    entry_points={
        'console_scripts': [
            'optical_flow_node = optical_flow_ros2.optical_flow_node:main'
        ],
    },
)