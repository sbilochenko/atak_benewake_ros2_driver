
"""Package setup for the Benewake ROS 2 driver."""

from setuptools import find_packages, setup

package_name = 'benewake_ros2_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            'share/' + package_name + '/config',
            ['config/lidar.yaml'],
        ),
        (
            'share/' + package_name + '/launch',
            ['launch/lidar.launch.py'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='serhii',
    maintainer_email='',
    description='ROS 2 UART driver for Benewake TFmini-S',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tfmini_s_driver = benewake_ros2_driver.driver_node:main',
            'distance_monitor = benewake_ros2_driver.distance_monitor:main',
        ],
    },
)
