from setuptools import setup
import os
from glob import glob

package_name = 'seekur_driver'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*')),        # Tous les fichiers launch
    (os.path.join('share', package_name, 'config'), glob('config/*')),       # Tous les fichiers config
    (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),           # Tous les fichiers urdf
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Driver ROS2 pour robot SeekurJR',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'seekur_driver_node = seekur_driver.seekur_driver_node:main',
            'seekur_protocol_simulator = seekur_driver.seekur_protocol_simulator:main',  # Nouvelle ligne
        ],
    },

)