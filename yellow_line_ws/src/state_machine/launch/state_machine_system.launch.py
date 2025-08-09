#!/usr/bin/env python3
"""
启动所有状态机相关节点
包括：
- qr_detector_node: 二维码检测节点
- state_machine_node: 状态机控制节点  
- voice_node: 语音交互节点
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    """生成launch描述"""
    
    # 声明launch参数
    declare_enable_voice = DeclareLaunchArgument(
        'enable_voice',
        default_value='true',
        description='是否启用语音节点'
    )
    
    declare_enable_qr = DeclareLaunchArgument(
        'enable_qr', 
        default_value='true',
        description='是否启用二维码检测节点'
    )
    
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='日志级别'
    )
    
    # 获取launch配置
    enable_voice = LaunchConfiguration('enable_voice')
    enable_qr = LaunchConfiguration('enable_qr')
    log_level = LaunchConfiguration('log_level')
    
    # 二维码检测节点
    qr_detector_node = Node(
        package='state_machine',
        executable='qr_detector_node',
        name='qr_detector_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }],
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(enable_qr)
    )
    
    # 状态机控制节点
    state_machine_node = Node(
        package='state_machine',
        executable='state_machine_node', 
        name='state_machine_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }],
        arguments=['--ros-args', '--log-level', log_level],
    )
    
    # 语音交互节点
    voice_node = Node(
        package='state_machine',
        executable='voice_node',
        name='voice_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }],
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(enable_voice)
    )
    
    # 启动信息
    start_info = LogInfo(
        msg=[
            '启动状态机系统...\n',
            '- QR检测节点: ', enable_qr, '\n',
            '- 状态机节点: 已启用\n',
            '- 语音节点: ', enable_voice, '\n',
            '日志级别: ', log_level
        ]
    )
    
    return LaunchDescription([
        # Launch参数声明
        declare_enable_voice,
        declare_enable_qr,
        declare_log_level,
        
        # 启动信息
        start_info,
        
        # 节点启动
        qr_detector_node,
        state_machine_node, 
        voice_node,
    ])