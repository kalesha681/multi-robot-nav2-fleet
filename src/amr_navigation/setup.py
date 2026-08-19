import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'amr_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abhinash',
    maintainer_email='abhinash@todo.todo',
    description='Heterogeneous fleet navigation, Nav2 stacks, goal management, and readiness coordination',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_manager_node = amr_navigation.mission_manager_node:main',
            'clock_readiness_gate = amr_navigation.readiness_nodes:clock_gate_main',
            'robot_readiness_coordinator = amr_navigation.readiness_nodes:robot_coordinator_main',
            'slope_cost_node = amr_navigation.slope_cost_node:main',
        ],
    },
)
