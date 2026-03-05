from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'acts_simulator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
    
    (os.path.join('share', package_name, 'urdf/components'), glob('urdf/components/*.xacro')),
    (os.path.join('share', package_name, 'urdf/sensors'), glob('urdf/sensors/*.xacro')),
    (os.path.join('share', package_name, 'urdf/macros'), glob('urdf/macros/*.xacro')),
    
    (os.path.join('share', package_name, 'meshes/core'), glob('meshes/core/*')),
    (os.path.join('share', package_name, 'meshes/propellers/ccw'), glob('meshes/propellers/ccw/*')),
    (os.path.join('share', package_name, 'meshes/propellers/cw'), glob('meshes/propellers/cw/*')),

    (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),

    (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chiara',
    maintainer_email='s7687956@studenti.unige.it',
    description='Aerial cable towed system simulation',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
