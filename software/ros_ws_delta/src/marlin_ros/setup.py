from glob import glob
import os

from setuptools import setup

package_name = 'marlin_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.yaml')),
    ],
    install_requires=[
        'pyserial',
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@example.com',
    description='Fast, lightweight ROS2 package to interface with Marlin device',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'marlin_node = marlin_ros.controller:main',
        ],
    },
)