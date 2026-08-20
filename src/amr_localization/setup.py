import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'amr_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abhinash',
    maintainer_email='abhinashkota@gmail.com',
    description='Extended Kalman Filter (EKF) sensor fusion and odometry localization for multi-robot AMR fleet',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ekf_fusion_node = amr_localization.ekf_fusion_node:main',
            'spawn_when_ready = amr_localization.spawn_when_ready:main',
        ],
    },
)
