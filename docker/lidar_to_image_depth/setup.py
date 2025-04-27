import os
from glob import glob
from setuptools import setup

package_name = 'lidar_to_image_depth'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    py_modules=[
        'lidar_to_image_depth.lidar_to_image_depth',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Remote control package for ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_to_image_depth = lidar_to_image_depth.lidar_to_image_depth:main',
        ],
    
    },    
    data_files=[
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
    ],
)