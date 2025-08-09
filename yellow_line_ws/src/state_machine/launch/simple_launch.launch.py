#!/usr/bin/env python3
"""
简单启动文件 - 启动所有必要的节点
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """生成launch描述"""
    
    return LaunchDescription([
        # 二维码检测节点
        Node(
            package='state_machine',
            executable='qr_detector_node',
            name='qr_detector_node',
            output='screen'
        ),
        
        # 状态机控制节点
        Node(
            package='state_machine', 
            executable='state_machine_node',
            name='state_machine_node',
            output='screen'
        ),
        
        # 语音交互节点
        Node(
            package='state_machine',
            executable='voice_node', 
            name='voice_node',
            output='screen'
        ),
    ])