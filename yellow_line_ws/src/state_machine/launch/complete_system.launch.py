#!/usr/bin/env python3

"""
完整系统启动文件 - 启动所有必要的节点

这个启动文件会启动以下节点：
1. 状态机节点 - 核心控制逻辑
2. 二维码检测节点 - 识别A-1/A-2/B-1/B-2
3. 语音节点 - 播报二维码、箭头方向、倒计时
4. 绿色箭头识别节点 - 识别箭头方向
5. 红色限高杆识别节点 - 识别限高杆距离
6. 黄色标志物识别节点 - 识别标志物距离
7. 黄线走中间控制节点 - S形路径控制
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 声明启动参数
    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='true',
        description='Enable debug mode for image publishers'
    )
    
    # 获取参数值
    debug_mode = LaunchConfiguration('debug_mode')
    
    return LaunchDescription([
        debug_mode_arg,
        
        # 1. 状态机节点 - 核心控制逻辑
        Node(
            package='state_machine',
            executable='state_machine_node',
            name='state_machine_node',
            output='screen',
            parameters=[{
                'debug_mode': debug_mode
            }]
        ),
        
        # 2. 二维码检测节点
        Node(
            package='state_machine',
            executable='qr_detector_node',
            name='qr_detector_node',
            output='screen',
            parameters=[{
                'debug_mode': debug_mode,
                'image_topic': '/image_rgb'
            }]
        ),
        
        # 3. 语音播报节点
        Node(
            package='state_machine',
            executable='voice_node',
            name='voice_node',
            output='screen'
        ),
        
        # 4. 绿色箭头识别节点
        Node(
            package='state_machine',
            executable='green_arrow_detector',
            name='green_arrow_detector',
            output='screen',
            parameters=[{
                'debug_mode': debug_mode
            }]
        ),
        
        # 5. 红色限高杆识别节点
        Node(
            package='state_machine',
            executable='red_barrier_detector',
            name='red_barrier_detector',
            output='screen',
            parameters=[{
                'debug_mode': debug_mode
            }]
        ),
        
        # 6. 黄色标志物识别节点
        Node(
            package='state_machine',
            executable='yellow_marker_detector',
            name='yellow_marker_detector',
            output='screen',
            parameters=[{
                'debug_mode': debug_mode
            }]
        ),
        
        # 7. 黄线走中间控制节点
        Node(
            package='state_machine',
            executable='yellow_line_walker',
            name='yellow_line_walker',
            output='screen'
        )
    ])


if __name__ == '__main__':
    generate_launch_description()