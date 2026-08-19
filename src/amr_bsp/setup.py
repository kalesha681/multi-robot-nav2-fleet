import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'amr_bsp'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abhinash',
    maintainer_email='abhinash@todo.todo',
    description='Board Support Package / Sensor Validation Layer for multi-robot AMR fleet',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_validator_node = amr_bsp.sensor_validator_node:main',
        ],
    },
)
