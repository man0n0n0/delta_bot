from setuptools import find_packages, setup

package_name = 'marlin_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='svevqx',
    maintainer_email='camporini@protonmail.com',
    description='Marlin CNC serial comunication tools',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'minimal_publisher_service = marlin_ros.minimal_publisher_service:main'
        ],
    },
)
