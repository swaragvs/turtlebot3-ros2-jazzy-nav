import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'antropi_tb3_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install all launch files to share/<package_name>/launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Install all yaml configuration files to share/<package_name>/config
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Install map files (yaml and pgm) to share/<package_name>/maps
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Swarag V S',
    maintainer_email='swaragvs09@gmail.com',
    description='TurtleBot3 Autonomous Navigation and Obstacle Avoidance using Nav2 on ROS 2 Jazzy',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [],
    },
)
