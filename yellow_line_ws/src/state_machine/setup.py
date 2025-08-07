from setuptools import setup
import os
from glob import glob

package_name = 'state_machine'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='机器人状态机控制系统',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_machine_node = state_machine.state_machine_node:main',
            'qr_detector_node = state_machine.qr_detector_node:main',
            'voice_node = state_machine.voice_node:main',
        ],
    },
) 