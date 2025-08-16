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
        # 安装launch文件
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # 安装配置文件
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
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
            'green_arrow_detector = state_machine.green_arrow_detector:main',
            'red_barrier_detector = state_machine.red_barrier_detector:main',
            'yellow_marker_detector = state_machine.yellow_marker_detector:main',
            'yellow_line_walker = state_machine.yellow_line_walker:main',
        ],
    },
) 