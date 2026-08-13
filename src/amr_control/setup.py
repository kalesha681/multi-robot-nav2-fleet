import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'amr_control'

setup(
    name=package_name,
    version='0.0.0',
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
    maintainer='cp-lab',
    maintainer_email='cp-lab@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mission_manager_node = amr_control.mission_manager_node:main',
            'clock_readiness_gate = amr_control.readiness_nodes:clock_gate_main',
            'robot_readiness_coordinator = amr_control.readiness_nodes:robot_coordinator_main',
        ],
    },
)
