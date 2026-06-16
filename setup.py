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
    (os.path.join('share', package_name, 'config/gz_bridge'), glob('config/gz_bridge/*.yaml')),
    (os.path.join('share', package_name, 'config/sensors'), glob('config/sensors/*.yaml')),

    (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro') + glob('urdf/*.sdf')),
    (os.path.join('share', package_name, 'urdf/components'), glob('urdf/components/*.xacro')),
    (os.path.join('share', package_name, 'urdf/sensors'), glob('urdf/sensors/*.xacro')),
    (os.path.join('share', package_name, 'urdf/macros'), glob('urdf/macros/*.xacro')),
    (os.path.join('share', package_name, 'urdf', 'special_cases'), glob('urdf/special_cases/*.xacro')),
    
    (os.path.join('share', package_name, 'meshes/core'), glob('meshes/core/*')),
    (os.path.join('share', package_name, 'meshes/propellers/ccw'), glob('meshes/propellers/ccw/*')),
    (os.path.join('share', package_name, 'meshes/propellers/cw'), glob('meshes/propellers/cw/*')),
    (os.path.join('share', package_name, 'meshes/core'), glob('meshes/core/*.dae')),
    (os.path.join('share', package_name, 'meshes/propellers/cw'), glob('meshes/propellers/cw/*.dae')),
    (os.path.join('share', package_name, 'meshes/propellers/ccw'), glob('meshes/propellers/ccw/*.dae')),

    (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),

    (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    
    (os.path.join('share', package_name, 'config'), glob('config/*.json') + glob('config/*.yaml')),

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
            'controller_node = acts_simulator.controller_node:main',
            'drone_start = acts_simulator.drone_start:main', 
            'gz_pose_to_odom = acts_simulator.gz_pose_to_odom:main',
        ],
    },
)
