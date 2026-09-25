import os
from glob import glob

from setuptools import setup

package_name = 'bno055_imu'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'firmware', 'bno055_imu_bridge'),
         glob('firmware/bno055_imu_bridge/*.ino')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mohamed',
    maintainer_email='todo@todo.todo',
    description='Pont série BNO055 vers sensor_msgs/Imu',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bno055_serial_node = bno055_imu.bno055_serial_node:main',
        ],
    },
)
