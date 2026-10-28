#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'linker_hand_ros2_sdk'

data_files = [
    ('share/ament_index/resource_index/packages',
     ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (
        os.path.join('share', package_name, 'launch'),
        sorted(glob('launch/*.launch.py')),
    ),
]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    package_data={
        package_name: [
            "LinkerHand/lib/linux-x86_64-ubuntu22/libcanbus.so",
        ],
    },
    data_files=data_files,
    install_requires=['setuptools', 'python-can'],
    zip_safe=True,
    maintainer='linker-robot',
    maintainer_email='linker-robot@todo.todo',
    description='ROS2 SDK for Linker Hand',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'linker_hand_sdk = linker_hand_ros2_sdk.linker_hand:main',
            'linker_hand_o30i = linker_hand_ros2_sdk.o30i_node:main',
            'linker_hand_advanced_o6 = linker_hand_ros2_sdk.linker_hand_advanced_o6:main',
            'linker_hand_advanced_l6 = linker_hand_ros2_sdk.linker_hand_advanced_l6:main',
            'linker_hand_advanced_l7 = linker_hand_ros2_sdk.linker_hand_advanced_l7:main',
            'linker_hand_advanced_l10 = linker_hand_ros2_sdk.linker_hand_advanced_l10:main',
            'linker_hand_advanced_g20 = linker_hand_ros2_sdk.linker_hand_advanced_g20:main',
            'linker_hand_g20_palm_touch = linker_hand_ros2_sdk.linker_hand_g20_palm_touch:main',
        ],
    },
)
