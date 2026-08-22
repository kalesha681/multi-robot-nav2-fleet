import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'amr_mapping'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kalesha Shaik',
    maintainer_email='kalesha681@gmail.com',
    description='Multi-robot cooperative SLAM, map fusion, and selective frontier update package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_fusion_node = amr_mapping.map_fusion_node:main',
        ],
    },
)
