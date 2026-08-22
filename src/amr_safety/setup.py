import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'amr_safety'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kalesha Shaik',
    maintainer_email='kalesha681@gmail.com',
    description='Independent safety override and dynamic braking supervisor for multi-robot fleet',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_override_node = amr_safety.safety_override_node:main',
        ],
    },
)
