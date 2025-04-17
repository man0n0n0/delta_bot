from setuptools import setup

package_name = 'arducam_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/default_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Manoah Camporini',
    author_email='camporini@protonmail.com',
    maintainer='Manoah Camporini',
    maintainer_email='camporini@protonmail.com',
    keywords=['ROS', 'ToF', 'Arducam', 'Point Cloud'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='ArduCam Time-of-Flight camera ROS2 package with rotation capability.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tof_pointcloud = ' + package_name + '.tof_pointcloud:main',
        ],
    },
)