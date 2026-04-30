import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'unitree_python_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tabi43',
    maintainer_email='marco.tabita@edu.unige.it',
    description='Python ROS 2 package',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'stand_and_sit_node = unitree_python_pkg.stand_and_sit_node:main',
            'stand_and_sit_sdk_node = unitree_python_pkg.stand_and_sit_sdk_node:main',
        ],
    },
)
