import os
from glob import glob

from setuptools import find_packages, setup

package_name = "go1_ros2_real"

data_files = [
    ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
    (f"share/{package_name}", ["package.xml"]),
    (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    (os.path.join("share", package_name, "urdf"), glob("urdf/*.urdf")),
]

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools"],
    entry_points={
        'console_scripts': [
            'hold_position=go1_ros2_real.hold_position_node:main',
            'stand_and_sit=go1_ros2_real.stand_and_sit_node:main',
        ],
    },
    zip_safe=True,
    maintainer="tabi43",
    maintainer_email="marco.tabita@edu.unige.it",
    description="Go1 real robot launch and RViz configuration",
    license="TODO: License declaration",
    tests_require=["pytest"],
)
