from setuptools import setup
import os
from glob import glob

package_name = 'seekur_driver'

# data_files ne copie que des FICHIERS et ne descend pas dans les sous-dossiers.
# files() : glob qui ignore les dossiers (evite l'erreur "not a regular file").
# tree()  : installe un dossier recursivement en gardant sa structure.

def files(pattern):
    """glob limité aux fichiers : un sous-dossier ne casse plus le build."""
    return [f for f in glob(pattern) if os.path.isfile(f)]


def tree(src):
    """Installe récursivement src/ sous share/<package>/src/ (structure conservée)."""
    return [(os.path.join('share', package_name, d), [os.path.join(d, f) for f in fs])
            for d, _, fs in os.walk(src) if fs]


setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), files('launch/*')),
        (os.path.join('share', package_name, 'config'), files('config/*')),
        (os.path.join('share', package_name, 'worlds'), files('worlds/*')),
        (os.path.join('share', package_name, 'urdf'), files('urdf/*')),
    ] + tree('description/meshes') + tree('worlds/models'),
    
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