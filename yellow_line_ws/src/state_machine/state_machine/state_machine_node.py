#!/usr/bin/env python3

"""
状态机节点 - 机器人运动控制状态机
启动方法：
    1. 构建工作空间：
       cd ~/yellow_line_ws
       colcon build --packages-select state_machine
       source install/setup.bash

    2. 启动节点：
       ros2 run state_machine state_machine_node

使用方法：
    1. 启动任务（正常流程）：
       ros2 topic pub -1 /state_machine/start_command std_msgs/msg/String "data: 'START'"

    2. 启动并进入指定状态（调试模式）：
       # 使用状态名称
       ros2 topic pub -1 /state_machine/start_command std_msgs/msg/String "data: 'START:IN_QR_A'"
       ros2 topic pub -1 /state_machine/start_command std_msgs/msg/String "data: 'START:IN_S2'"
       ros2 topic pub -1 /state_machine/start_command std_msgs/msg/String "data: 'START:IN_QRB'"
       
       # 使用状态值
       ros2 topic pub -1 /state_machine/start_command std_msgs/msg/String "data: 'START:2'"
       ros2 topic pub -1 /state_machine/start_command std_msgs/msg/String "data: 'START:12'"
       ros2 topic pub -1 /state_machine/start_command std_msgs/msg/String "data: 'START:18'"

    3. 手动设置识别值（调试模式）：
       # 设置A点二维码识别值
       ros2 topic pub -1 /state_machine/start_command std_msgs/msg/String "data: 'SET_QR_A:A-1'"
       ros2 topic pub -1 /state_machine/start_command std_msgs/msg/String "data: 'SET_QR_A:A-2'"
       
       # 设置B点二维码识别值
       ros2 topic pub -1 /state_machine/start_command std_msgs/msg/String "data: 'SET_QR_B:B-1'"
       ros2 topic pub -1 /state_machine/start_command std_msgs/msg/String "data: 'SET_QR_B:B-2'"
       
       # 设置绿色箭头识别值
       ros2 topic pub -1 /state_machine/start_command std_msgs/msg/String "data: 'SET_ARROW:left'"
       ros2 topic pub -1 /state_machine/start_command std_msgs/msg/String "data: 'SET_ARROW:right'"

    4. 设置超时默认值：
       # 设置A点二维码超时默认值
       ros2 topic pub -1 /state_machine/start_command std_msgs/msg/String "data: 'SET_QR_A_DEFAULT:A-2'"
       
       # 设置B点二维码超时默认值  
       ros2 topic pub -1 /state_machine/start_command std_msgs/msg/String "data: 'SET_QR_B_DEFAULT:B-2'"
       
       # 设置绿色箭头超时默认值
       ros2 topic pub -1 /state_machine/start_command std_msgs/msg/String "data: 'SET_ARROW_DEFAULT:right'"

    5. 停止任务：
       ros2 topic pub -1 /state_machine/start_command std_msgs/msg/String "data: 'STOP'"

    6. 重置状态机：
       ros2 topic pub -1 /state_machine/start_command std_msgs/msg/String "data: 'RESET'"

    7. 监控状态：
       ros2 topic echo /state_machine/state_info

    8. 查看运动控制指令：
       ros2 topic echo /mi_desktop_48_b0_2d_7b_03_d0/motion_servo_cmd

超时处理：
    - A点二维码识别超时（20秒）：自动使用默认值 A-1
    - B点二维码识别超时（20秒）：自动使用默认值 B-1  
    - 绿色箭头识别超时（15秒）：自动使用默认值 left

状态列表（调试时可直接跳转）：
    START=0, START_TO_QRA=1, IN_QR_A=2, QRA_TO_A1=3, QRA_TO_A2=4,
    IN_A1=5, IN_A2=6, A1_TO_S1=7, A2_TO_S1=8, IN_S1=9, S1_TO_S2=10,
    IN_S2=11, S2_TO_L1=12, S2_TO_R1=13, IN_L1=14, IN_R1=15,
    L1_TO_QRB=16, R1_TO_QRB=17, IN_QRB=18, QRB_TO_B1=19, QRB_TO_B2=20,
    IN_B1=21, IN_B2=22, B1_TO_B2=23, B2_TO_B1=24, B1_TO_L1=25,
    B1_TO_R1=26, B2_TO_L1=27, B2_TO_R1=28, L1_TO_S2=29, R1_TO_S2=30,
    IN_S2_R=31, S2_TO_S1=32, IN_S1_R=33, S1_TO_A1=34, S1_TO_A2=35,
    A1_TO_START=36, A2_TO_START=37, YELLOW_LIGHT_STOP=38,
    YELLOW_LIGHT_COUNTDOWN=39, END=45, ERROR=46

"""

from itertools import count
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import subprocess
import signal
import os
import yaml
from pathlib import Path
from enum import Enum
# 导入ROS2消息类型
from protocol.msg import MotionServoCmd
# 导入服务类型
from protocol.srv import MotionResultCmd
# 导入图像消息类型
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
# 导入黄线检测器 - 使用包名导入
from state_machine.yellow_line_detector import YellowLineDetector

class State(Enum):
    """状态机状态枚举 - 根据状态机超详细说明.md"""
    START = 0            # 等待启动命令
    START_TO_QRA = 1     # 从起点运动到二维码A点
    IN_QR_A = 2          # 在二维码A点进行识别
    QRA_TO_A1 = 3        # 从二维码A点运动到A1点
    QRA_TO_A2 = 4        # 从二维码A点运动到A2点
    IN_A1 = 5            # 在A1点停止
    IN_A2 = 6            # 在A2点停止
    A1_TO_S1 = 7         # 从A1点运动到S1点
    A2_TO_S1 = 8         # 从A2点运动到S1点
    IN_S1 = 9            # 在S1点
    S1_TO_S2 = 10        # 从S1点运动到S2点
    IN_S2 = 11           # 在S2点
    S2_TO_L1 = 12        # 从S2点运动到L1点
    S2_TO_R1 = 13        # 从S2点运动到R1点
    IN_L1 = 14           # 在L1点
    IN_R1 = 15           # 在R1点
    L1_TO_QRB = 16       # 从L1点运动到QRB点
    R1_TO_QRB = 17       # 从R1点运动到QRB点
    IN_QRB = 18          # 在QRB点进行识别
    QRB_TO_B1 = 19       # 从QRB点运动到B1点
    QRB_TO_B2 = 20       # 从QRB点运动到B2点
    IN_B1 = 21           # 在B1点
    IN_B2 = 22           # 在B2点
    B1_TO_B2 = 23        # 从B1点运动到B2点
    B2_TO_B1 = 24        # 从B2点运动到B1点
    B1_TO_L1 = 25        # 从B1点运动到L1点
    B1_TO_R1 = 26        # 从B1点运动到R1点
    B2_TO_L1 = 27        # 从B2点运动到L1点
    B2_TO_R1 = 28        # 从B2点运动到R1点
    L1_TO_S2 = 29        # 从L1点运动到S2点（返程）
    R1_TO_S2 = 30        # 从R1点运动到S2点（返程）
    IN_S2_R = 31         # 在S2点（返程）
    S2_TO_S1 = 32        # 从S2点运动到S1点（返程）
    IN_S1_R = 33         # 在S1点（返程）
    S1_TO_A1 = 34        # 从S1点运动到A1点（返程）
    S1_TO_A2 = 35        # 从S1点运动到A2点（返程）
    A1_TO_START = 36     # 从A1点运动到起点
    A2_TO_START = 37     # 从A2点运动到起点
    YELLOW_LIGHT_STOP = 38 # 在黄灯前停止倒计时
    YELLOW_LIGHT_COUNTDOWN = 39 # 黄灯倒计时状态
    CURVE_TRACK_FOLLOW = 40    # 曲线赛道黄线跟踪
    DOCK_APPROACH = 41         # 接近库位
    DOCK_ENTER = 42           # 进入库位
    DOCK_LYING = 43           # 在库位内趴下等待
    CHARGE_RETURN = 44        # 返回充电站
    END = 45             # 任务结束
    ERROR = 46           # 错误状态

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')
        
        # 节点管理相关
        self.managed_nodes = {}  # 管理的节点进程字典 {node_name: process}
        # 相机节点管理相关
        self.camera_nodes = {}  # 管理的相机节点进程字典 {node_name: process}
        self.camera_service_client = None  # AI相机服务客户端
        
        # 相机状态管理
        self.camera_states = {
            'rgb_system': False,     # RGB相机系统状态
            'ai_system': False,      # AI相机系统状态
            'realsense': False,      # RealSense相机状态
            'stereo_camera': False,  # 立体相机状态
            'camera_server': False   # AI相机服务器状态
        }
        self.camera_switch_lock = False  # 相机切换锁，防止并发切换
        self.last_camera_switch_time = None  # 上次相机切换时间
        self.camera_switch_cooldown = 3.0    # 相机切换冷却时间（秒）
        self.node_configs = {
            'qr_detector_node': {
                'package': 'state_machine',
                'executable': 'qr_detector_node',
                'required_states': [State.IN_QR_A, State.IN_QRB],
                'args': []  # 默认参数，IN_QRB状态时会使用特殊参数
            },
            'green_arrow_detector': {
                'package': 'state_machine', 
                'executable': 'green_arrow_detector',
                'required_states': [State.IN_S2]
            },
            'red_barrier_detector': {
                'package': 'state_machine',
                'executable': 'red_barrier_detector', 
                'required_states': [State.S2_TO_L1, State.B1_TO_L1, State.B2_TO_L1]
            },
            'yellow_marker_detector': {
                'package': 'state_machine',
                'executable': 'yellow_marker_detector',
                'required_states': [State.S2_TO_R1, State.B1_TO_R1, State.B2_TO_R1]
            },
            'voice_node': {
                'package': 'state_machine',
                'executable': 'voice_node',
                'required_states': [State.IN_QR_A, State.IN_QRB, State.IN_S2, State.IN_R1]  # 根据需要动态启动
            },
            'yellow_line_walker': {
                'package': 'state_machine',
                'executable': 'yellow_line_walker',
                'required_states': [State.S1_TO_S2, State.S2_TO_S1]
            }
        }
        
        # 状态机状态
        self.current_state = State.START
        self.previous_state = State.START

        # 任务完成标志
        self.task_completed = False
        
        # 二维码识别结果
        self.qr_result = None
        self.initial_qr_result = None  # 初始A点二维码结果，用于返程时的逻辑判断
        self.qrb_result = None  # QRB点的二维码结果
        self.qr_detection_timeout = 60.0  # 二维码识别超时时间（秒），降低超时时间
        self.qr_detection_start_time = None
        self.qr_a_default_value = "A-1"  # A点二维码超时默认值
        self.qr_b_default_value = "B-1"  # B点二维码超时默认值
        
        # 绿色箭头识别结果
        self.green_arrow_result = None
        self.green_arrow_detection_timeout = 15.0  # 绿色箭头识别超时时间（秒）
        self.green_arrow_detection_start_time = None
        self.green_arrow_default_value = "right"  # 绿色箭头超时默认值
        
        # 语音播报相关
        self.voice_completed = False
        self.voice_timeout = 10.0  # 语音播报超时时间（秒），增加到10秒避免与QR重试冲突
        self.voice_start_time = None
        self.waiting_for_voice = False
        self.voice_complete_timer = None  # 语音播报完成超时定时器
        self.pending_qr_node_stop = False  # 是否有待停止的二维码节点
        self.r1_next_state = None  # R1状态的下一状态（根据前一状态决定）
        self.l1_next_state = None  # L1状态的下一状态（根据前一状态决定）
        self.a1_next_state = None  # A1状态的下一状态（根据前一状态决定）
        self.a2_next_state = None  # A2状态的下一状态（根据前一状态决定）
        
        # 前进步数记录器 - 记录前进运动(速度为[0.4,0,0])的步数
        self.x1_steps = 0  # S2到L1路径中前进到红色限高杆的步数
        self.y1_steps = 0  # S2到R1路径中前进到黄色标志物的步数
        self.x2_steps = 0  # B1/B2到L1路径中前进到红色限高杆的步数
        self.y2_steps = 0  # B1/B2到R1路径中前进到黄色标志物的步数
        self.z1_steps = 0  # QR识别时前进尝试的步数
        
        # B区任务相关
        self.b_zone_task_count = 0  # B区任务完成次数（B1和B2之间的切换次数）
        self.max_b_zone_tasks = 2   # 最大B区任务次数，完成后进入返程
        
        # QR识别重试计数器
        self.qr_retry_count = 0  
        self.max_qr_retries = 5  # 最多重试5次
        
        # 图像处理相关
        self.bridge = CvBridge()
        # 使用统一的黄线检测器处理所有图像
        self.yellow_line_detector = YellowLineDetector()
        # 黄色标志物检测器通过节点管理系统按需启动，不在这里直接实例化
        
        # 图像数据结构改进
        self.fisheye_image = None
        self.fisheye_image_timestamp = None
        self.fisheye_image_seq = 0
        
        self.rgb_image = None
        self.rgb_image_timestamp = None
        self.rgb_image_seq = 0
        
        self.left_fisheye_image = None
        self.left_fisheye_image_timestamp = None
        self.left_fisheye_image_seq = 0
        
        # 位置检查状态管理
        self.position_check_context = {}
        self.pending_position_recheck = False
        self.position_correction_attempts = 0
        self.max_correction_attempts = 3
        self.last_movement_completion_time = None
        self.image_wait_threshold = 0.5  # 等待图像更新的时间阈值（秒）
        
        # 黄灯检测相关（已由独立的yellow_marker_detector和voice_node节点处理）
        self.countdown_timer = None
        self.countdown_seconds = 5  # 倒计时秒数
        self.current_countdown = 0
        
        # 运动控制参数
        self.movement_timeout = 90.0  # 运动超时时间（秒）
        self.movement_start_time = None
        self.movement_step = 0  # 运动步骤计数器
        self.movement_total_steps = 0  # 总步骤数
        self.movement_sequence = []  # 运动序列
        
        self.roi_params_ycy = {
            'top_ratio': 0.6,
            'bottom_ratio': 1.0,
            'left_ratio': 0.0,
            'right_ratio': 1.0
        }

        self.roi_params_dy = {
            'top_ratio': 0.6,
            'bottom_ratio': 1,
            'left_ratio': 0.0,
            'right_ratio': 1.0
        }

        self.threshold_params_ycy = {
            'position_threshold': 0.08,
            'area_threshold': 800
        }

        self.threshold_params_dy = {
            'distance_threshold': 0.81,
            'area_threshold': 300
        }
        
        # 加载视觉检查参数配置
        self.load_vision_check_config()
        
        # 基础运动类型配置 - 根据状态机超详细说明.md和MotionServoCmd接口规范
        # 接口约束: vel_des[1] y方向最大值1.5, vel_des[2] yaw最大值2.0
        self.basic_movements = {
            # 运动类型1: 向前，+x 速度0.5，频率5，次数15
            'forward': {
                'motion_id': 303,        # 机器人运控姿态
                'cmd_type': 1,           # 指令类型: 1=Data, 2=End
                'cmd_source': 4,         # 指令来源: 4=Algo
                'value': 0,              # 0=内八步态, 2=垂直步态
                'vel_des': [0.4, 0.0, 0.0],  # [x, y(≤1.5), yaw(≤2.0)] m/s
                'rpy_des': [0.0, 0.0, 0.0],  # 当前暂不开放
                'pos_des': [0.0, 0.0, 0.0],  # 当前暂不开放
                'acc_des': [0.0, 0.0, 0.0],  # 当前暂不开放
                'ctrl_point': [0.0, 0.0, 0.0],  # 当前暂不开放
                'foot_pose': [0.0, 0.0, 0.0],   # 当前暂不开放
                'step_height': [0.05, 0.05],    # 抬腿高度，默认0.05m
                'frequency': 5.0,
                'count': 10
            },
            # 运动类型2: 向后，-x 速度0.5，频率5，次数15
            'backward': {
                'motion_id': 303,        # 机器人运控姿态
                'cmd_type': 1,           # 指令类型: 1=Data, 2=End
                'cmd_source': 4,         # 指令来源: 4=Algo
                'value': 0,              # 0=内八步态, 2=垂直步态
                'vel_des': [-0.3, 0.0, 0.0], # [x, y(≤1.5), yaw(≤2.0)] m/s
                'rpy_des': [0.0, 0.0, 0.0],  # 当前暂不开放
                'pos_des': [0.0, 0.0, 0.0],  # 当前暂不开放
                'acc_des': [0.0, 0.0, 0.0],  # 当前暂不开放
                'ctrl_point': [0.0, 0.0, 0.0],  # 当前暂不开放
                'foot_pose': [0.0, 0.0, 0.0],   # 当前暂不开放
                'step_height': [0.05, 0.05],    # 抬腿高度，默认0.05m
                'frequency': 5.0,
                'count': 10
            },
            # 运动类型3: 向右，-y 速度0.3，频率5，次数10
            'right': {
                'motion_id': 303,        # 机器人运控姿态
                'cmd_type': 1,           # 指令类型: 1=Data, 2=End
                'cmd_source': 4,         # 指令来源: 4=Algo
                'value': 0,              # 0=内八步态, 2=垂直步态
                'vel_des': [0.0, -0.3, 0.0], # [x, y(≤1.5), yaw(≤2.0)] m/s
                'rpy_des': [0.0, 0.0, 0.0],  # 当前暂不开放
                'pos_des': [0.0, 0.0, 0.0],  # 当前暂不开放
                'acc_des': [0.0, 0.0, 0.0],  # 当前暂不开放
                'ctrl_point': [0.0, 0.0, 0.0],  # 当前暂不开放
                'foot_pose': [0.0, 0.0, 0.0],   # 当前暂不开放
                'step_height': [0.05, 0.05],    # 抬腿高度，默认0.05m
                'frequency': 5.0,
                'count': 21
            },
            # 运动类型4: 向左，+y 速度0.3，频率5，次数10
            'left': {
                'motion_id': 303,        # 机器人运控姿态
                'cmd_type': 1,           # 指令类型: 1=Data, 2=End
                'cmd_source': 4,         # 指令来源: 4=Algo
                'value': 0,              # 0=内八步态, 2=垂直步态
                'vel_des': [0.0, 0.3, 0.0],  # [x, y(≤1.5), yaw(≤2.0)] m/s
                'rpy_des': [0.0, 0.0, 0.0],  # 当前暂不开放
                'pos_des': [0.0, 0.0, 0.0],  # 当前暂不开放
                'acc_des': [0.0, 0.0, 0.0],  # 当前暂不开放
                'ctrl_point': [0.0, 0.0, 0.0],  # 当前暂不开放
                'foot_pose': [0.0, 0.0, 0.0],   # 当前暂不开放
                'step_height': [0.05, 0.05],    # 抬腿高度，默认0.05m
                'frequency': 5.0,
                'count': 21
            },
            # 运动类型5: 右转90度，角速度-1，频率5，次数9
            'turn_right': {
                'motion_id': 303,        # 机器人运控姿态
                'cmd_type': 1,           # 指令类型: 1=Data, 2=End
                'cmd_source': 4,         # 指令来源: 4=Algo
                'value': 0,              # 0=内八步态, 2=垂直步态
                'vel_des': [0.0, 0.0, -0.9], # [x, y(≤1.5), yaw(≤2.0)] m/s
                'rpy_des': [0.0, 0.0, 0.0],  # 当前暂不开放
                'pos_des': [0.0, 0.0, 0.0],  # 当前暂不开放
                'acc_des': [0.0, 0.0, 0.0],  # 当前暂不开放
                'ctrl_point': [0.0, 0.0, 0.0],  # 当前暂不开放
                'foot_pose': [0.0, 0.0, 0.0],   # 当前暂不开放
                'step_height': [0.05, 0.05],    # 抬腿高度，默认0.05m
                'frequency': 5.0,
                'count': 11
            },
            # 运动类型6: 左转90度，角速度1，频率5，次数9
            'turn_left': {
                'motion_id': 303,        # 机器人运控姿态
                'cmd_type': 1,           # 指令类型: 1=Data, 2=End
                'cmd_source': 4,         # 指令来源: 4=Algo
                'value': 0,              # 0=内八步态, 2=垂直步态
                'vel_des': [0.0, 0.0, 0.9],  # [x, y(≤1.5), yaw(≤2.0)] m/s
                'rpy_des': [0.0, 0.0, 0.0],  # 当前暂不开放
                'pos_des': [0.0, 0.0, 0.0],  # 当前暂不开放
                'acc_des': [0.0, 0.0, 0.0],  # 当前暂不开放
                'ctrl_point': [0.0, 0.0, 0.0],  # 当前暂不开放
                'foot_pose': [0.0, 0.0, 0.0],   # 当前暂不开放
                'step_height': [0.05, 0.05],    # 抬腿高度，默认0.05m
                'frequency': 5.0,
                'count': 11
            },
            # 运动类型7: 急停，使用服务调用
            'emergency_stop': {
                'motion_id': 0,          # 急停
                'cmd_source': 4,         # 指令来源: 4=Algo
                'vel_des': [0.0, 0.0, 0.0],  # [x, y(≤1.5), yaw(≤2.0)] m/s
                'rpy_des': [0.0, 0.0, 0.0],  # 当前暂不开放
                'pos_des': [0.0, 0.0, 0.0],  # 当前暂不开放
                'acc_des': [0.0, 0.0, 0.0],  # 当前暂不开放
                'ctrl_point': [0.0, 0.0, 0.0],  # 当前暂不开放
                'foot_pose': [0.0, 0.0, 0.0],   # 当前暂不开放
                'step_height': [0.05, 0.05],    # 抬腿高度，默认0.05m
                'duration': 0,           # 持续时间
                'is_service_call': True  # 标记为服务调用
            },
            # 运动类型8: 站立，使用服务调用
            'stand': {
                'motion_id': 111,        # 站立
                'cmd_source': 4,         # 指令来源: 4=Algo
                'vel_des': [0.0, 0.0, 0.0],  # [x, y(≤1.5), yaw(≤2.0)] m/s
                'rpy_des': [0.0, 0.0, 0.0],  # 当前暂不开放
                'pos_des': [0.0, 0.0, 0.0],  # 当前暂不开放
                'acc_des': [0.0, 0.0, 0.0],  # 当前暂不开放
                'ctrl_point': [0.0, 0.0, 0.0],  # 当前暂不开放
                'foot_pose': [0.0, 0.0, 0.0],   # 当前暂不开放
                'step_height': [0.05, 0.05],    # 抬腿高度，默认0.05m
                'duration': 0,           # 持续时间
                'is_service_call': True  # 标记为服务调用
            },
            # 运动类型9: 高阻尼趴下，使用服务调用
            'lie_down': {
                'motion_id': 101,        # 高阻尼趴下
                'cmd_source': 4,         # 指令来源: 4=Algo
                'vel_des': [0.0, 0.0, 0.0],  # [x, y(≤1.5), yaw(≤2.0)] m/s
                'rpy_des': [0.0, 0.0, 0.0],  # 当前暂不开放
                'pos_des': [0.0, 0.0, 0.0],  # 当前暂不开放
                'acc_des': [0.0, 0.0, 0.0],  # 当前暂不开放
                'ctrl_point': [0.0, 0.0, 0.0],  # 当前暂不开放
                'foot_pose': [0.0, 0.0, 0.0],   # 当前暂不开放
                'step_height': [0.05, 0.05],    # 抬腿高度，默认0.05m
                'duration': 0,           # 持续时间
                'is_service_call': True  # 标记为服务调用
            }
        }

        
        # 运动控制定时器和当前配置
        self.motion_timer = None
        self.current_motion_config = None
        self.motion_count = 0  # 当前运动指令发送次数
        
        # lie_down等待相关
        self.lie_down_wait_timer = None
        self.pending_lie_down_params = None
        self.lie_down_final_timer = None
        self.position_check_restart_timer = None
        
        # 定时器管理 - 状态绑定的定时器字典
        self.state_timers = {}  # {state: {timer_name: timer_object}}
        self.global_timers = {}  # {timer_name: timer_object} 全局定时器
        
        # 创建发布者和订阅者
        self.setup_communication()
        
        # 启动RGB相机功能
        self.start_rgb_cameras()
        
        # 创建定时器
        self.create_timer(0.1, self.state_machine_loop)  # 10Hz状态机循环
        
        print("[状态机] 节点已启动，等待命令...")
        print("[状态机] 按需启动模式已启用")

    def load_vision_check_config(self):
        """加载视觉检查参数配置文件 - 支持源码开发和编译安装双模式"""
        try:
            config_path = None
            
            # 方法1: 尝试使用 ament_index_python (编译安装后的标准路径)
            try:
                from ament_index_python.packages import get_package_share_directory
                import os
                
                pkg_share_dir = get_package_share_directory('state_machine')
                config_path = os.path.join(pkg_share_dir, 'config', 'vision_check_params.yaml')
                
                if os.path.exists(config_path):
                    print(f"[状态机] 使用安装目录配置文件: {config_path}")
                else:
                    config_path = None
                    
            except Exception as e:
                print(f"[状态机] ament_index_python 查找失败: {str(e)}")
                config_path = None
            
            # 方法2: 如果方法1失败，尝试使用源码目录路径 (开发调试时)
            if config_path is None:
                source_config_path = Path(__file__).parent / "config" / "vision_check_params.yaml"
                if source_config_path.exists():
                    config_path = str(source_config_path)
                    print(f"[状态机] 使用源码目录配置文件: {config_path}")
            
            # 如果两种方法都失败，使用默认配置
            if config_path is None or not os.path.exists(config_path):
                print(f"[状态机] 配置文件不存在: {config_path}")
                print("[状态机] 使用默认参数配置")
                self.vision_config = None
                return
            
            # 加载YAML配置文件
            with open(config_path, 'r', encoding='utf-8') as f:
                self.vision_config = yaml.safe_load(f)
            
            print(f"[状态机] 成功加载视觉检查参数配置: {config_path}")
            print(f"[状态机] 配置包含 {len(self.vision_config.get('check_point_configs', {}))} 个检查点配置")
            
        except Exception as e:
            print(f"[状态机] 加载配置文件失败: {str(e)}")
            print("[状态机] 使用默认参数配置")
            self.vision_config = None
    
    def get_check_params(self, config_name, detection_type):
        """
        根据配置名获取检查参数
        
        Args:
            config_name: 检查点配置名称
            detection_type: 检测类型 ('ycy' 或 'dy')
            
        Returns:
            tuple: (roi_params, threshold_params)
        """
        try:
            print(f"[调试] get_check_params 调用 - config_name: {config_name}, detection_type: {detection_type}")
            print(f"[调试] vision_config 是否存在: {self.vision_config is not None}")
            
            if not self.vision_config:
                # 使用默认参数
                print(f"[调试] vision_config 为空，使用默认参数")
                if detection_type == 'ycy':
                    return self.roi_params_ycy, self.threshold_params_ycy
                else:
                    return self.roi_params_dy, self.threshold_params_dy
            
            # 从配置中获取检查点配置
            print(f"[调试] 在 check_point_configs 中查找: {config_name}")
            check_config = self.vision_config['check_point_configs'].get(config_name)
            print(f"[调试] 查找结果: {check_config}")
            
            if not check_config:
                # 使用默认配置
                print(f"[调试] 配置 {config_name} 不存在，使用默认配置")
                default_config = self.vision_config['default_configs'][detection_type]
                roi_name = default_config['roi_params']
                threshold_name = default_config['threshold_params']
                print(f"[状态机] 检查点 {config_name} 未找到，使用默认配置: {roi_name}/{threshold_name}")
            else:
                roi_name = check_config['roi_params']
                threshold_name = check_config['threshold_params']
                print(f"[调试] 找到配置 - ROI名称: {roi_name}, 阈值名称: {threshold_name}")
            
            # 获取ROI参数
            print(f"[调试] 获取ROI参数: {roi_name}")
            roi_params = self.vision_config['roi_params'][roi_name]
            print(f"[调试] ROI参数内容: {roi_params}")
            
            # 获取阈值参数
            print(f"[调试] 获取阈值参数: {threshold_name}")
            threshold_params = self.vision_config['threshold_params'][threshold_name]
            print(f"[调试] 阈值参数内容: {threshold_params}")
            
            print(f"[状态机] 使用配置 {config_name}: ROI={roi_name}, 阈值={threshold_name}")
            return roi_params, threshold_params
            
        except Exception as e:
            print(f"[状态机] 获取参数配置失败 {config_name}: {str(e)}")
            # 回退到默认参数
            if detection_type == 'ycy':
                return self.roi_params_ycy, self.threshold_params_ycy
            else:
                return self.roi_params_dy, self.threshold_params_dy

    def start_required_nodes(self, new_state):
        """根据新状态启动所需的节点"""
        for node_name, config in self.node_configs.items():
            # 检查当前状态是否需要这个节点
            if new_state in config['required_states']:
                if node_name not in self.managed_nodes:
                    self.start_node(node_name, config)
                    
    def stop_unused_nodes(self, new_state):
        """停止当前状态不需要的节点"""
        nodes_to_stop = []
        for node_name, config in self.node_configs.items():
            # 如果节点正在运行，但当前状态不需要它
            if (node_name in self.managed_nodes and 
                new_state not in config['required_states']):
                nodes_to_stop.append(node_name)
        
        for node_name in nodes_to_stop:
            self.stop_node(node_name)
            
    def start_node(self, node_name, config, extra_args=None):
        """启动一个节点"""
        try:
            print(f"[状态机] 启动节点: {node_name}")
            
            # 构建ros2 run命令
            cmd = [
                'ros2', 'run', 
                config['package'], 
                config['executable'],
                '--ros-args', '--log-level', 'info'
            ]
            
            # 添加配置中的参数
            if 'args' in config and config['args']:
                cmd.extend(config['args'])
            
            # 添加额外参数
            if extra_args:
                cmd.extend(extra_args)
            
            # 设置环境变量
            env = os.environ.copy()
            
            # 启动进程
            process = subprocess.Popen(
                cmd,
                env=env,
                stdout=None,
                stderr=None,
                preexec_fn=os.setsid  # 创建新的进程组
            )
            
            self.managed_nodes[node_name] = process
            print(f"[状态机] 节点 {node_name} 启动成功 (PID: {process.pid})")
            
            # 等待一小段时间确保节点启动
            time.sleep(0.5)
            
        except Exception as e:
            print(f"[状态机] 启动节点 {node_name} 失败: {str(e)}")
            
    def stop_node(self, node_name):
        """停止一个节点"""
        try:
            if node_name in self.managed_nodes:
                process = self.managed_nodes[node_name]
                print(f"[状态机] 停止节点: {node_name} (PID: {process.pid})")
                
                # 优雅关闭：发送SIGTERM
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                
                # 等待最多3秒
                try:
                    process.wait(timeout=3)
                    print(f"[状态机] 节点 {node_name} 已优雅关闭")
                except subprocess.TimeoutExpired:
                    # 强制关闭：发送SIGKILL
                    print(f"[状态机] 强制关闭节点 {node_name}")
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                    process.wait()
                
                del self.managed_nodes[node_name]
                
        except Exception as e:
            print(f"[状态机] 停止节点 {node_name} 失败: {str(e)}")
            # 即使失败也从字典中移除
            if node_name in self.managed_nodes:
                del self.managed_nodes[node_name]
    
    def camera_health_monitor(self):
        """相机健康状态监控"""
        health_status = self.check_camera_health()
        
        # 检查是否有相机异常
        for camera_name, status in health_status.items():
            if status == 'stopped' and self.camera_states.get(camera_name, False):
                print(f"[相机监控] 检测到{camera_name}异常停止，尝试重启")
                # 根据相机类型重启
                if camera_name in ['realsense', 'stereo_camera']:
                    self.camera_states['rgb_system'] = False
                elif camera_name == 'camera_server':
                    self.camera_states['ai_system'] = False
        
        # 打印相机状态信息（每分钟一次）
        import time
        if not hasattr(self, '_last_status_print_time') or time.time() - self._last_status_print_time > 60:
            current_mode = self.get_current_camera_mode()
            print(f"[相机监控] 当前模式: {current_mode}, 状态: {health_status}")
            self._last_status_print_time = time.time()
    
    
    def start_rgb_cameras(self):
        """启动RGB相机和鱼眼相机（对应start_rgb.sh的功能）"""
        # 检查是否已经启动
        if self.camera_states['rgb_system']:
            print("[状态机] RGB相机系统已经运行，跳过启动")
            return True
            
        try:
            print("[状态机] 启动RGB相机和鱼眼相机...")
            
            # 1. 启动realsense相机
            print("[状态机] 启动RealSense相机...")
            realsense_cmd = ['ros2', 'launch', 'realsense2_camera', 'on_dog.py']
            realsense_process = subprocess.Popen(
                realsense_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            self.camera_nodes['realsense'] = realsense_process
            print(f"[状态机] RealSense相机已启动 (PID: {realsense_process.pid})")
            
            # 等待启动
            time.sleep(5)
            
            # 检查进程是否正常
            if realsense_process.poll() is not None:
                print(f"[状态机] RealSense相机启动失败，进程已退出")
                return False
            
            # 2. 配置和激活realsense相机
            print("[状态机] 配置RealSense相机...")
            if not self._run_command(['ros2', 'lifecycle', 'set', '/camera/camera', 'configure']):
                print("[状态机] RealSense相机配置失败")
                return False
            if not self._run_command(['ros2', 'lifecycle', 'set', '/camera/camera', 'activate']):
                print("[状态机] RealSense相机激活失败")
                return False
            print("[状态机] RealSense相机已激活")
            self.camera_states['realsense'] = True
            
            # 3. 启动立体相机
            print("[状态机] 启动立体相机...")
            stereo_cmd = ['ros2', 'launch', 'camera_test', 'stereo_camera.py']
            stereo_process = subprocess.Popen(
                stereo_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            self.camera_nodes['stereo_camera'] = stereo_process
            print(f"[状态机] 立体相机已启动 (PID: {stereo_process.pid})")
            
            # 等待立体相机启动
            time.sleep(5)
            
            # 检查进程是否正常
            if stereo_process.poll() is not None:
                print(f"[状态机] 立体相机启动失败，进程已退出")
                return False
            
            # 4. 配置和激活立体相机
            print("[状态机] 配置立体相机...")
            if not self._run_command(['ros2', 'lifecycle', 'set', '/stereo_camera', 'configure']):
                print("[状态机] 立体相机配置失败")
                return False
            if not self._run_command(['ros2', 'lifecycle', 'set', '/stereo_camera', 'activate']):
                print("[状态机] 立体相机激活失败")
                return False
            print("[状态机] 立体相机已激活")
            self.camera_states['stereo_camera'] = True
            
            # 更新系统状态
            self.camera_states['rgb_system'] = True
            print("[状态机] RGB相机系统启动完成")
            return True
            
        except Exception as e:
            print(f"[状态机] 启动RGB相机失败: {str(e)}")
            # 清理已启动的进程
            self.stop_rgb_cameras()
            return False
    
    def start_ai_cameras(self):
        """启动AI相机（对应start_ai.sh的功能）"""
        # 检查是否已经启动
        if self.camera_states['ai_system']:
            print("[状态机] AI相机系统已经运行，跳过启动")
            return True
            
        try:
            print("[状态机] 启动AI相机...")
            
            # 1. 启动camera_server节点
            print("[状态机] 启动camera_server节点...")
            ai_cmd = ['ros2', 'run', 'camera_test', 'camera_server']
            ai_process = subprocess.Popen(
                ai_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            self.camera_nodes['camera_server'] = ai_process
            print(f"[状态机] camera_server已启动 (PID: {ai_process.pid})")
            
            # 等待camera_server完全启动
            time.sleep(3)
            
            # 检查进程是否正常
            if ai_process.poll() is not None:
                print(f"[状态机] camera_server启动失败，进程已退出")
                return False
            
            # 2. 配置AI相机服务
            print("[状态机] 配置AI相机服务...")
            # 创建AI相机服务客户端
            if not self.camera_service_client:
                from protocol.srv import CameraService
                self.camera_service_client = self.create_client(
                    CameraService,
                    '/mi_desktop_48_b0_2d_7b_03_d0/camera_service'
                )
            
            # 等待服务可用
            if self.camera_service_client.wait_for_service(timeout_sec=5):
                # 调用服务配置AI相机
                request = CameraService.Request()
                request.command = 9
                request.width = 640
                request.height = 480
                request.fps = 0
                
                future = self.camera_service_client.call_async(request)
                # 这里简化处理，在生产环境中应该等待回调
                time.sleep(1)
                print("[状态机] AI相机配置完成")
                self.camera_states['camera_server'] = True
            else:
                print("[状态机] AI相机服务不可用，跳过配置")
                return False
            
            # 更新系统状态
            self.camera_states['ai_system'] = True
            print("[状态机] AI相机系统启动完成")
            return True
            
        except Exception as e:
            print(f"[状态机] 启动AI相机失败: {str(e)}")
            # 清理已启动的进程
            self.stop_ai_cameras()
            return False
    
    def stop_rgb_cameras(self):
        """停止RGB相机和鱼眼相机"""
        try:
            print("[状态机] 停止RGB相机系统...")
            
            # 停止RealSense相机
            if 'realsense' in self.camera_nodes:
                self._stop_camera_node('realsense')
                self.camera_states['realsense'] = False
            
            # 停止立体相机
            if 'stereo_camera' in self.camera_nodes:
                self._stop_camera_node('stereo_camera')
                self.camera_states['stereo_camera'] = False
            
            # 更新系统状态
            self.camera_states['rgb_system'] = False
            print("[状态机] RGB相机系统已停止")
            
        except Exception as e:
            print(f"[状态机] 停止RGB相机失败: {str(e)}")
            # 即使失败也更新状态
            self.camera_states['rgb_system'] = False
            self.camera_states['realsense'] = False
            self.camera_states['stereo_camera'] = False
    
    def stop_ai_cameras(self):
        """停止AI相机"""
        try:
            print("[状态机] 停止AI相机系统...")
            
            # 停止camera_server
            if 'camera_server' in self.camera_nodes:
                self._stop_camera_node('camera_server')
                self.camera_states['camera_server'] = False
            
            # 更新系统状态
            self.camera_states['ai_system'] = False
            print("[状态机] AI相机系统已停止")
            
        except Exception as e:
            print(f"[状态机] 停止AI相机失败: {str(e)}")
            # 即使失败也更新状态
            self.camera_states['ai_system'] = False
            self.camera_states['camera_server'] = False
    
    def _stop_camera_node(self, camera_name):
        """停止指定的相机节点"""
        try:
            if camera_name in self.camera_nodes:
                process = self.camera_nodes[camera_name]
                print(f"[状态机] 停止相机节点: {camera_name} (PID: {process.pid})")
                
                # 优雅关闭：发送SIGTERM
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                
                # 等待最多3秒
                try:
                    process.wait(timeout=3)
                    print(f"[状态机] 相机节点 {camera_name} 已优雅关闭")
                except subprocess.TimeoutExpired:
                    # 强制关闭：发送SIGKILL
                    print(f"[状态机] 强制关闭相机节点 {camera_name}")
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                    process.wait()
                
                del self.camera_nodes[camera_name]
                
        except Exception as e:
            print(f"[状态机] 停止相机节点 {camera_name} 失败: {str(e)}")
            # 即使失败也从字典中移除
            if camera_name in self.camera_nodes:
                del self.camera_nodes[camera_name]
    
    def _run_command(self, cmd):
        """运行系统命令"""
        try:
            result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True, timeout=10)
            if result.returncode != 0:
                print(f"[状态机] 命令执行失败: {' '.join(cmd)}, 错误: {result.stderr}")
            else:
                print(f"[状态机] 命令执行成功: {' '.join(cmd)}")
            return result.returncode == 0
        except Exception as e:
            print(f"[状态机] 执行命令异常: {' '.join(cmd)}, 错误: {str(e)}")
            return False
    
    def check_camera_health(self):
        """检查相机健康状态"""
        health_status = {}
        
        for camera_name, process in self.camera_nodes.items():
            if process and process.poll() is None:
                health_status[camera_name] = 'running'
            else:
                health_status[camera_name] = 'stopped'
                self.camera_states[camera_name] = False
        
        return health_status
    
    def wait_for_camera_ready(self, camera_type, timeout=5.0):
        """
        等待相机就绪并产生稳定的图像流
        
        Args:
            camera_type: 'rgb', 'ai'
            timeout: 超时时间（秒）
            
        Returns:
            bool: 相机是否就绪
        """
        import time
        start_time = time.time()
        
        print(f"[状态机] 等待{camera_type}相机就绪...")
        
        while time.time() - start_time < timeout:
            if camera_type == 'rgb':
                # 检查RGB和鱼眼图像是否都就绪
                if (self.rgb_image is not None and 
                    self.fisheye_image is not None and
                    hasattr(self, 'left_fisheye_image') and 
                    self.left_fisheye_image is not None):
                    print(f"[状态机] {camera_type}相机图像流就绪")
                    return True
            elif camera_type == 'ai':
                # AI相机目前主要用于QR识别，可以检查相关图像流
                # 这里简化处理，只检查服务是否可用
                if (self.camera_service_client and 
                    self.camera_service_client.service_is_ready()):
                    print(f"[状态机] {camera_type}相机服务就绪")
                    return True
            
            time.sleep(0.2)  # 等待200ms
        
        print(f"[状态机] 等待{camera_type}相机就绪超时")
        return False
    
    def smart_camera_switch(self, target_mode):
        """
        智能相机切换，已注释掉AI相机切换功能，保持全程使用RGB相机
        RGB相机应全程运行，不再切换到AI相机
        
        Args:
            target_mode: 'rgb' 或 'ai'
            
        Returns:
            bool: 切换是否成功
        """
        import time
        
        # 注释掉AI相机切换功能，强制返回RGB模式成功
        if target_mode == 'ai':
            print("[状态机] AI相机切换已禁用，继续使用RGB相机")
            return True  # 假装切换成功，继续使用RGB相机
        
        # 检查相机切换锁
        if self.camera_switch_lock:
            print("[状态机] 相机切换正在进行中，跳过")
            return False
        
        # 检查冷却时间
        if (self.last_camera_switch_time and 
            time.time() - self.last_camera_switch_time < self.camera_switch_cooldown):
            remaining_time = self.camera_switch_cooldown - (time.time() - self.last_camera_switch_time)
            print(f"[状态机] 相机切换冷却中，剩余{remaining_time:.1f}秒")
            return False
        
        # 检查是否已经在目标模式（仅RGB检查）
        current_mode = self.get_current_camera_mode()
        if current_mode == target_mode and target_mode == 'rgb':
            print(f"[状态机] 相机已经在{target_mode}模式，无需切换")
            return True
        
        # 开始切换（仅RGB）
        self.camera_switch_lock = True
        success = False
        
        try:
            print(f"[状态机] 开始相机切换: {current_mode} -> {target_mode}")
            
            # 注释掉AI相机切换代码
            # if target_mode == 'ai':
            #     # 切换到AI模式：先停止RGB，再启动AI
            #     print("[状态机] 互斥切换：停止RGB相机以启动AI相机")
            #     if self.camera_states['rgb_system']:
            #         self.stop_rgb_cameras()
            #         time.sleep(2)  # 等待RGB相机完全停止
            #     
            #     # 启动AI相机
            #     print("[状态机] 互斥切换：启动AI相机")
            #     if not self.camera_states['ai_system']:
            #         success = self.start_ai_cameras()
            #         if success:
            #             success = self.wait_for_camera_ready('ai')
            #     else:
            #         success = True
                    
            if target_mode == 'rgb':
                # 确保RGB相机运行
                print("[状态机] 确保RGB相机运行")
                if not self.camera_states['rgb_system']:
                    success = self.start_rgb_cameras()
                    if success:
                        success = self.wait_for_camera_ready('rgb')
                else:
                    success = True
            
            if success:
                self.last_camera_switch_time = time.time()
                print(f"[状态机] 相机切换成功: {current_mode} -> {target_mode}")
            else:
                print(f"[状态机] 相机切换失败: {target_mode}")
                
        except Exception as e:
            print(f"[状态机] 相机切换异常: {str(e)}")
            success = False
        finally:
            self.camera_switch_lock = False
        
        return success
    
    def get_current_camera_mode(self):
        """获取当前相机模式"""
        if self.camera_states['rgb_system'] and not self.camera_states['ai_system']:
            return 'rgb'
        elif self.camera_states['ai_system'] and not self.camera_states['rgb_system']:
            return 'ai'
        elif self.camera_states['rgb_system'] and self.camera_states['ai_system']:
            return 'mixed'  # 两种模式同时运行
        else:
            return 'none'   # 没有相机运行
                
    def cleanup_all_nodes(self):
        """清理所有管理的节点和相机"""
        print("[状态机] 清理所有节点和相机...")
        
        # 清理节点
        node_names = list(self.managed_nodes.keys())
        for node_name in node_names:
            self.stop_node(node_name)
        
        # 清理相机节点
        self.stop_rgb_cameras()
        self.stop_ai_cameras()

    def create_state_timer(self, timer_name: str, period: float, callback, bind_to_state=None):
        """创建状态绑定的定时器
        
        Args:
            timer_name: 定时器名称
            period: 定时器周期（秒）
            callback: 回调函数
            bind_to_state: 绑定到的状态，状态改变时会自动清理
            
        Returns:
            timer对象
        """
        # 如果绑定到状态
        if bind_to_state is not None:
            if bind_to_state not in self.state_timers:
                self.state_timers[bind_to_state] = {}
            
            # 如果同名定时器已存在，先取消
            if timer_name in self.state_timers[bind_to_state]:
                self.state_timers[bind_to_state][timer_name].cancel()
                print(f"[状态机] 替换已存在的状态定时器: {timer_name} -> {bind_to_state}")
            
            # 创建新定时器
            timer = self.create_timer(period, callback)
            self.state_timers[bind_to_state][timer_name] = timer
            print(f"[状态机] 创建状态绑定定时器: {timer_name} -> {bind_to_state} ({period}s)")
        else:
            # 全局定时器
            if timer_name in self.global_timers:
                self.global_timers[timer_name].cancel()
                print(f"[状态机] 替换已存在的全局定时器: {timer_name}")
            
            timer = self.create_timer(period, callback)
            self.global_timers[timer_name] = timer
            print(f"[状态机] 创建全局定时器: {timer_name} ({period}s)")
        
        return timer

    def cancel_state_timer(self, timer_name: str, state=None):
        """取消指定的状态定时器
        
        Args:
            timer_name: 定时器名称
            state: 状态，如果为None则取消全局定时器
        """
        if state is not None and state in self.state_timers:
            if timer_name in self.state_timers[state]:
                self.state_timers[state][timer_name].cancel()
                del self.state_timers[state][timer_name]
                print(f"[状态机] 已取消状态定时器: {timer_name} ({state})")
                return True
        elif timer_name in self.global_timers:
            self.global_timers[timer_name].cancel()
            del self.global_timers[timer_name]
            print(f"[状态机] 已取消全局定时器: {timer_name}")
            return True
        return False

    def cleanup_state_timers(self, state):
        """清理指定状态的所有定时器
        
        Args:
            state: 要清理定时器的状态
        """
        if state in self.state_timers:
            timer_names = list(self.state_timers[state].keys())
            count = 0
            for timer_name in timer_names:
                try:
                    self.state_timers[state][timer_name].cancel()
                    count += 1
                except Exception as e:
                    print(f"[状态机] 取消定时器失败: {timer_name}: {str(e)}")
            
            self.state_timers[state].clear()
            if count > 0:
                print(f"[状态机] 已清理状态 {state} 的 {count} 个定时器")
    
    def cleanup_all_timers(self):
        """清理所有定时器"""
        total_count = 0
        
        # 清理所有状态绑定定时器
        for state in list(self.state_timers.keys()):
            count = len(self.state_timers[state])
            self.cleanup_state_timers(state)
            total_count += count
        
        # 清理所有全局定时器
        for timer_name in list(self.global_timers.keys()):
            try:
                self.global_timers[timer_name].cancel()
                total_count += 1
            except Exception as e:
                print(f"[状态机] 取消全局定时器失败: {timer_name}: {str(e)}")
        
        self.global_timers.clear()
        
        if total_count > 0:
            print(f"[状态机] 已清理所有 {total_count} 个定时器")

        
    def setup_communication(self):
        """设置通信接口"""
        
        # 运动控制命令发布者 - 使用MotionServoCmd接口
        # 话题格式: /{设备名}/motion_servo_cmd
        self.motion_cmd_publisher = self.create_publisher(
            MotionServoCmd,
            '/mi_desktop_48_b0_2d_7b_03_d0/motion_servo_cmd',
            10
        )
        
        # 运动控制服务客户端 - 用于站立、趴下、急停等
        self.motion_service_client = self.create_client(
            MotionResultCmd,
            'mi_desktop_48_b0_2d_7b_03_d0/motion_result_cmd'
        )
        
        # 二维码信息订阅者
        self.qr_info_subscription = self.create_subscription(
            String,
            '/qr_detector/qr_info',
            self.qr_info_callback,
            10
        )
        
        # 状态信息发布者
        self.state_publisher = self.create_publisher(
            String,
            '/state_machine/state_info',
            10
        )
        
        # 启动命令订阅者
        self.start_command_subscription = self.create_subscription(
            String,
            '/state_machine/start_command',
            self.start_command_callback,
            10
        )
        
        # 鱼眼相机图像订阅者 (1500x800 RG10)
        self.fisheye_image_subscription = self.create_subscription(
            Image,
            '/image_right',
            self.fisheye_image_callback,
            10
        )
        
        # RGB相机图像订阅者 (1600x1200 RG10)
        self.rgb_image_subscription = self.create_subscription(
            Image,
            '/image_rgb',
            self.rgb_image_callback,
            10
        )
        
        # 左鱼眼相机图像订阅者
        self.left_fisheye_subscription = self.create_subscription(
            Image,
            '/image_left',
            self.left_fisheye_callback,
            10
        )
        
        # 绿色箭头方向订阅者
        self.green_arrow_subscription = self.create_subscription(
            String,
            '/green_arrow_detector/direction',
            self.green_arrow_callback,
            10
        )
        
        # 红色限高杆距离订阅者
        self.red_barrier_subscription = self.create_subscription(
            String,
            '/red_barrier_detector/distance',
            self.red_barrier_callback,
            10
        )
        
        # 黄色标志物距离订阅者
        self.yellow_marker_subscription = self.create_subscription(
            String,
            '/yellow_marker_detector/distance',
            self.yellow_marker_callback,
            10
        )
        
        # 两条黄线走中间控制发布者
        self.yellow_line_control_publisher = self.create_publisher(
            String,
            '/yellow_line_walker/command',
            10
        )
        
        # 语音播报完成信号订阅者
        self.voice_complete_subscription = self.create_subscription(
            String,
            '/voice_node/complete_signal',
            self.voice_complete_callback,
            10
        )
        
        # 语音触发发布者 - 用于触发voice_node倒计时
        self.voice_trigger_publisher = self.create_publisher(
            String,
            '/voice_node/countdown_trigger',
            10
        )
        
        # 启动相机健康监控定时器
        self.camera_health_timer = self.create_timer(10.0, self.camera_health_monitor)
        
    def start_command_callback(self, msg):
        """处理启动命令
        支持的命令格式：
        1. START - 从START状态开始正常流程
        2. START:<state_name> - 启动并直接进入指定状态
        3. START:<state_value> - 启动并直接进入指定状态值对应的状态
        4. STOP - 停止当前任务
        5. RESET - 重置状态机
        6. SET_QR_A:<value> - 设置A点二维码识别值（A-1或A-2）
        7. SET_QR_B:<value> - 设置B点二维码识别值（B-1或B-2）
        8. SET_ARROW:<value> - 设置绿色箭头识别值（left或right）
        9. SET_QR_A_DEFAULT:<value> - 设置A点二维码超时默认值
        10. SET_QR_B_DEFAULT:<value> - 设置B点二维码超时默认值
        11. SET_ARROW_DEFAULT:<value> - 设置绿色箭头超时默认值
        12. EXIT_B_ZONE - 强制退出B区循环，进入返程状态（根据绿色箭头方向相反方向返程）
        
        示例：
        - START:IN_QR_A 或 START:2 - 直接进入二维码A识别状态
        - SET_QR_A:A-2 - 手动设置A点二维码为A-2
        - SET_ARROW:right - 手动设置绿色箭头方向为right
        - SET_QR_A_DEFAULT:A-2 - 设置A点二维码超时默认值为A-2
        - EXIT_B_ZONE - 强制退出B区任务，根据绿色箭头相反方向开始返程
        """
        command_parts = msg.data.strip().split(':', 1)
        command = command_parts[0]
        
        if command == "START":
            if len(command_parts) == 1:
                # 原有的START命令逻辑
                if self.current_state == State.START and not self.task_completed:
                    print("[状态机] 收到启动命令")
                    self.transition_to_state(State.START_TO_QRA)
                elif self.task_completed:
                    print("[状态机] 任务已完成，请发送RESET重置")
                else:
                    print(f"[状态机] 当前状态 {self.current_state.name} 不支持START命令")
            else:
                # 新增的带状态参数的START命令
                state_param = command_parts[1]
                target_state = self._parse_state_parameter(state_param)
                
                if target_state is not None:
                    print(f"[状态机] 收到启动命令，目标状态: {target_state.name}")
                    self.transition_to_state(target_state)
                else:
                    print(f"[状态机] 无效的状态参数: {state_param}")
                    
        elif command == "SET_QR_A":
            if len(command_parts) == 2:
                qr_value = command_parts[1]
                if qr_value in ["A-1", "A-2"]:
                    self.qr_result = qr_value
                    self.initial_qr_result = qr_value
                    print(f"[状态机] 手动设置A点二维码识别值: {qr_value}")
                    if self.current_state == State.IN_QR_A:
                        self.process_qr_a_result()
                else:
                    print(f"[状态机] 无效的A点二维码值: {qr_value}，支持: A-1, A-2")
            else:
                print("[状态机] SET_QR_A命令格式错误，应为: SET_QR_A:<value>")
                
        elif command == "SET_QR_B":
            if len(command_parts) == 2:
                qr_value = command_parts[1]
                if qr_value in ["B-1", "B-2"]:
                    self.qrb_result = qr_value
                    print(f"[状态机] 手动设置B点二维码识别值: {qr_value}")
                    if self.current_state == State.IN_QRB:
                        self.process_qr_b_result()
                else:
                    print(f"[状态机] 无效的B点二维码值: {qr_value}，支持: B-1, B-2")
            else:
                print("[状态机] SET_QR_B命令格式错误，应为: SET_QR_B:<value>")
                
        elif command == "SET_ARROW":
            if len(command_parts) == 2:
                arrow_value = command_parts[1]
                if arrow_value in ["left", "right"]:
                    # 检查是否已经有识别结果，防止重复设置
                    if self.green_arrow_result is not None:
                        print(f"[状态机] 绿色箭头已设置为: {self.green_arrow_result}，忽略重复设置: {arrow_value}")
                        return
                        
                    self.green_arrow_result = arrow_value
                    print(f"[状态机] 手动设置绿色箭头识别值: {arrow_value}")
                    
                    # 重置绿色箭头识别开始时间，防止超时检查继续触发
                    self.green_arrow_detection_start_time = None
                    
                    if self.current_state == State.IN_S2:
                        self._process_green_arrow_result(arrow_value)
                else:
                    print(f"[状态机] 无效的绿色箭头值: {arrow_value}，支持: left, right")
            else:
                print("[状态机] SET_ARROW命令格式错误，应为: SET_ARROW:<value>")
                
        elif command == "SET_QR_A_DEFAULT":
            if len(command_parts) == 2:
                default_value = command_parts[1]
                if default_value in ["A-1", "A-2"]:
                    self.qr_a_default_value = default_value
                    print(f"[状态机] 设置A点二维码超时默认值: {default_value}")
                else:
                    print(f"[状态机] 无效的A点二维码默认值: {default_value}，支持: A-1, A-2")
            else:
                print("[状态机] SET_QR_A_DEFAULT命令格式错误，应为: SET_QR_A_DEFAULT:<value>")
                
        elif command == "SET_QR_B_DEFAULT":
            if len(command_parts) == 2:
                default_value = command_parts[1]
                if default_value in ["B-1", "B-2"]:
                    self.qr_b_default_value = default_value
                    print(f"[状态机] 设置B点二维码超时默认值: {default_value}")
                else:
                    print(f"[状态机] 无效的B点二维码默认值: {default_value}，支持: B-1, B-2")
            else:
                print("[状态机] SET_QR_B_DEFAULT命令格式错误，应为: SET_QR_B_DEFAULT:<value>")
                
        elif command == "SET_ARROW_DEFAULT":
            if len(command_parts) == 2:
                default_value = command_parts[1]
                if default_value in ["left", "right"]:
                    self.green_arrow_default_value = default_value
                    print(f"[状态机] 设置绿色箭头超时默认值: {default_value}")
                else:
                    print(f"[状态机] 无效的绿色箭头默认值: {default_value}，支持: left, right")
            else:
                print("[状态机] SET_ARROW_DEFAULT命令格式错误，应为: SET_ARROW_DEFAULT:<value>")
                
        elif command == "EXIT_B_ZONE":
            if self.current_state in [State.IN_B1, State.IN_B2, State.B1_TO_B2, State.B2_TO_B1]:
                print("[状态机] 收到退出B区命令，强制进入返程状态")
                self.b_zone_task_count = self.max_b_zone_tasks  # 设置为最大值，触发退出
                if self.current_state == State.IN_B1:
                    # 根据绿色箭头方向决定返程路径（相反方向）
                    if self.green_arrow_result == "left":
                        print("[状态机] 来时向左，从B1强制返程到R1")
                        self.transition_to_state(State.B1_TO_R1)
                    elif self.green_arrow_result == "right":
                        print("[状态机] 来时向右，从B1强制返程到L1")
                        self.transition_to_state(State.B1_TO_L1)
                    else:
                        print("[状态机] 没有绿色箭头记录，从B1强制返程到L1")
                        self.transition_to_state(State.B1_TO_L1)
                elif self.current_state == State.IN_B2:
                    # 根据绿色箭头方向决定返程路径（相反方向）
                    if self.green_arrow_result == "left":
                        print("[状态机] 来时向左，从B2强制返程到R1")
                        self.transition_to_state(State.B2_TO_R1)
                    elif self.green_arrow_result == "right":
                        print("[状态机] 来时向右，从B2强制返程到L1")
                        self.transition_to_state(State.B2_TO_L1)
                    else:
                        print("[状态机] 没有绿色箭头记录，从B2强制返程到L1")
                        self.transition_to_state(State.B2_TO_L1)
                else:
                    print(f"[状态机] 当前状态 {self.current_state.name} 将在下次状态转换时自动退出B区")
            else:
                print(f"[状态机] 当前状态 {self.current_state.name} 不在B区，无法执行退出B区命令")
                    
        elif command == "STOP":
            print("[状态机] 收到停止命令")
            # 停止运动定时器
            self.stop_motion_timer()
            self.transition_to_state(State.START)
        elif command == "RESET":
            print("[状态机] 收到重置命令")
            self.reset_state_machine()
        else:
            print(f"[状态机] 收到未知命令: {msg.data}")
    
    def _process_green_arrow_result(self, direction):
        """处理绿色箭头识别结果"""
        if direction == "left":
            self.transition_to_state(State.S2_TO_L1)
        elif direction == "right":
            self.transition_to_state(State.S2_TO_R1)
        else:
            print(f"[状态机] 未知箭头方向: {direction}")
            self.transition_to_state(State.ERROR)
    
    def _parse_state_parameter(self, state_param):
        """解析状态参数，支持状态名称或状态值"""
        try:
            # 尝试按状态值解析（数字）
            if state_param.isdigit():
                state_value = int(state_param)
                for state in State:
                    if state.value == state_value:
                        return state
                print(f"[状态机] 未找到状态值 {state_value} 对应的状态")
                return None
            else:
                # 尝试按状态名称解析
                state_name = state_param.upper()
                try:
                    return State[state_name]
                except KeyError:
                    print(f"[状态机] 未找到状态名称 {state_name}")
                    return None
        except Exception as e:
            print(f"[状态机] 解析状态参数时出错: {e}")
            return None
    
    def qr_info_callback(self, msg):
        """处理二维码识别结果"""
        try:
            # 解析二维码信息 - 新格式只包含二维码内容，不包含距离
            # 格式: "A-1" 或 "A-2" 或 "B-1" 或 "B-2"
            qr_code = msg.data.strip()
            
            self.get_logger().info(f'收到二维码识别结果: {qr_code}')
            
            if self.current_state == State.IN_QR_A and self.qr_result is None:
                self.qr_result = qr_code
                self.initial_qr_result = qr_code  # 保存初始二维码结果
                print(f"[状态机] A点二维码识别完成: {qr_code}，等待语音播报完成信号")
                
                # 设置等待语音播报标志
                self.waiting_for_voice = True
                self.voice_start_time = time.time()
                self.pending_qr_node_stop = True
                
                # 启动语音播报超时定时器（10秒） - 使用状态绑定定时器
                self.voice_complete_timer = self.create_state_timer('voice_timeout', 10.0, self._voice_timeout_callback, State.IN_QR_A)
                
                self.process_qr_a_result()
            elif self.current_state == State.IN_QRB and self.qrb_result is None:
                self.qrb_result = qr_code
                print(f"[状态机] B点二维码识别完成: {qr_code}，等待语音播报完成信号")
                
                # 设置等待语音播报标志
                self.waiting_for_voice = True
                self.voice_start_time = time.time()
                self.pending_qr_node_stop = True
                
                # 启动语音播报超时定时器（10秒） - 使用状态绑定定时器
                self.voice_complete_timer = self.create_state_timer('voice_timeout', 10.0, self._voice_timeout_callback, State.IN_QRB)
                
                self.process_qr_b_result()
            else:
                # 处理重复识别结果或无效状态的二维码识别
                if self.current_state == State.IN_QR_A and self.qr_result is not None:
                    print(f"[状态机] A点二维码已识别为: {self.qr_result}，忽略重复识别结果: {qr_code}")
                elif self.current_state == State.IN_QRB and self.qrb_result is not None:
                    print(f"[状态机] B点二维码已识别为: {self.qrb_result}，忽略重复识别结果: {qr_code}")
                elif self.current_state not in [State.IN_QR_A, State.IN_QRB]:
                    print(f"[状态机] 当前状态 {self.current_state.name} 不支持二维码识别，忽略识别结果: {qr_code}")
                else:
                    print(f"[状态机] 未处理的二维码识别情况: 状态={self.current_state.name}, 结果={qr_code}")
                    
        except Exception as e:
            self.get_logger().error(f'处理二维码信息时出错: {str(e)}')
    
    def voice_complete_callback(self, msg):
        """语音播报完成信号回调"""
        try:
            signal = msg.data.strip()
            if signal == "COMPLETE" and self.waiting_for_voice:
                print("[状态机] 收到语音播报完成信号")
                
                # 根据当前状态处理不同的完成逻辑
                if self.current_state == State.IN_R1:
                    print("[状态机] R1状态收到voice完成信号")
                    self.waiting_for_voice = False
                    self._handle_r1_voice_complete()
                elif self.current_state in [State.IN_QR_A, State.IN_QRB]:
                    print("[状态机] 二维码状态收到voice完成信号，关闭相关节点")
                    self._handle_voice_complete()
                elif self.current_state == State.IN_S2:
                    print("[状态机] S2状态收到voice完成信号，关闭相关节点")
                    self._handle_voice_complete_arrow()
                else:
                    print(f"[状态机] 在状态{self.current_state.name}收到voice完成信号，但无处理逻辑")
                    
        except Exception as e:
            self.get_logger().error(f'处理语音完成信号时出错: {str(e)}')
    
    def _voice_timeout_callback(self):
        """语音播报超时回调（二维码）"""
        if self.waiting_for_voice:
            print("[状态机] 二维码语音播报超时，强制关闭相关节点")
            self._handle_voice_complete()
    
    def _voice_timeout_callback_arrow(self):
        """语音播报超时回调（绿色箭头）"""
        if self.waiting_for_voice:
            print("[状态机] 绿色箭头语音播报超时，强制关闭相关节点")
            self._handle_voice_complete_arrow()
    
    def _handle_voice_complete(self):
        """处理语音播报完成（无论是正常完成还是超时）"""
        # 重置语音播报状态
        self.waiting_for_voice = False
        self.voice_start_time = None
        
        # 语音定时器会通过状态转换自动清理，不需要手动取消
        self.voice_complete_timer = None
        
        # 如果有待停止的二维码节点，现在停止它
        if self.pending_qr_node_stop:
            if 'qr_detector_node' in self.managed_nodes:
                print("[状态机] 关闭二维码识别节点")
                self.stop_node('qr_detector_node')
            self.pending_qr_node_stop = False
    
    def _handle_voice_complete_arrow(self):
        """处理绿色箭头语音播报完成（无论是正常完成还是超时）"""
        # 重置语音播报状态
        self.waiting_for_voice = False
        self.voice_start_time = None
        
        # 语音定时器会通过状态转换自动清理，不需要手动取消
        self.voice_complete_timer = None
        
        # 关闭绿色箭头识别节点
        if 'green_arrow_detector' in self.managed_nodes:
            print("[状态机] 关闭绿色箭头识别节点")
            self.stop_node('green_arrow_detector')
    
    def left_fisheye_callback(self, msg):
        """左鱼眼相机图像回调"""
        try:
            import time
            self.left_fisheye_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.left_fisheye_image_timestamp = time.time()
            self.left_fisheye_image_seq += 1
            
        except Exception as e:
            self.get_logger().error(f'处理左鱼眼相机图像时出错: {str(e)}')
    
    def green_arrow_callback(self, msg):
        """绿色箭头方向回调"""
        try:
            direction = msg.data.strip()
            self.get_logger().info(f'收到绿色箭头方向: {direction}')
            
            # 检查是否已经处理过绿色箭头识别结果
            if self.current_state == State.IN_S2 and self.green_arrow_result is None:
                self.green_arrow_result = direction
                print(f"[状态机] 绿色箭头识别完成: {direction}，等待语音播报完成信号")
                
                # 重置绿色箭头识别开始时间，防止超时检查重复触发
                self.green_arrow_detection_start_time = None
                
                # 设置等待语音播报标志（对于绿色箭头识别）
                self.waiting_for_voice = True
                self.voice_start_time = time.time()
                
                # 启动语音播报超时定时器（10秒），但不设置pending_qr_node_stop（因为这里是绿色箭头检测器） - 使用状态绑定定时器
                self.voice_complete_timer = self.create_state_timer('voice_timeout_arrow', 10.0, self._voice_timeout_callback_arrow, State.IN_S2)
                
                self._process_green_arrow_result(direction)
            elif self.green_arrow_result is not None:
                print(f"[状态机] 绿色箭头已识别为: {self.green_arrow_result}，忽略重复识别结果: {direction}")
            else:
                # 处理无效状态的绿色箭头识别
                if self.current_state != State.IN_S2:
                    print(f"[状态机] 当前状态 {self.current_state.name} 不支持绿色箭头识别，忽略识别结果: {direction}")
                else:
                    print(f"[状态机] 未处理的绿色箭头识别情况: 状态={self.current_state.name}, 结果={direction}")
        except Exception as e:
            self.get_logger().error(f'处理绿色箭头方向时出错: {str(e)}')
    
    def red_barrier_callback(self, msg):
        """红色限高杆距离回调"""
        try:
            distance = float(msg.data.strip())
            self.get_logger().info(f'检测到红色限高杆，距离: {distance}')
            
            # 根据当前状态和距离信息停止前进并记录步数
            if self.current_state == State.S2_TO_L1:
                self.stop_forward_and_record_x1()
            elif self.current_state in [State.B1_TO_L1, State.B2_TO_L1]:
                self.stop_forward_and_record_x2()
            else:
                # 处理无效状态的红色限高杆检测
                print(f"[状态机] 当前状态 {self.current_state.name} 不支持红色限高杆检测，忽略检测结果: {distance}")
                
        except Exception as e:
            self.get_logger().error(f'处理红色限高杆距离时出错: {str(e)}')
    
    def yellow_marker_callback(self, msg):
        """黄色标志物距离回调"""
        try:
            distance = float(msg.data.strip())
            self.get_logger().info(f'检测到黄色标志物，距离: {distance}')
            
            # 根据当前状态和距离信息停止前进并记录步数
            if self.current_state == State.S2_TO_R1:
                self.stop_forward_and_record_y1()
            elif self.current_state in [State.B1_TO_R1, State.B2_TO_R1]:
                self.stop_forward_and_record_y2()
            else:
                # 处理无效状态的黄色标志物检测
                print(f"[状态机] 当前状态 {self.current_state.name} 不支持黄色标志物检测，忽略检测结果: {distance}")
                
        except Exception as e:
            self.get_logger().error(f'处理黄色标志物距离时出错: {str(e)}')
    
    def stop_forward_and_record_x1(self):
        """停止前进并记录x1步数"""
        self.x1_steps = self.motion_count
        self.get_logger().info(f'记录x1步数: {self.x1_steps}')
        self.stop_motion_timer()
        self.transition_to_state(State.IN_L1)
    
    def stop_forward_and_record_y1(self):
        """停止前进并记录y1步数"""
        self.y1_steps = self.motion_count
        self.get_logger().info(f'记录y1步数: {self.y1_steps}')
        self.stop_motion_timer()
        self.transition_to_state(State.IN_R1)
    
    def stop_forward_and_record_x2(self):
        """停止前进并记录x2步数"""
        self.x2_steps = self.motion_count
        self.get_logger().info(f'记录x2步数: {self.x2_steps}')
        self.stop_motion_timer()
        self.transition_to_state(State.IN_L1)
    
    def stop_forward_and_record_y2(self):
        """停止前进并记录y2步数"""
        self.y2_steps = self.motion_count
        self.get_logger().info(f'记录y2步数: {self.y2_steps}')
        self.stop_motion_timer()
        self.transition_to_state(State.IN_R1)
    
    def fisheye_image_callback(self, msg):
        """鱼眼相机图像回调"""
        try:
            import time
            self.fisheye_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.fisheye_image_timestamp = time.time()
            self.fisheye_image_seq += 1
            
        except Exception as e:
            self.get_logger().error(f'处理鱼眼相机图像时出错: {str(e)}')
    
    def rgb_image_callback(self, msg):
        """RGB相机图像回调"""
        try:
            import time
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.rgb_image_timestamp = time.time()
            self.rgb_image_seq += 1
            
        except Exception as e:
            self.get_logger().error(f'处理RGB相机图像时出错: {str(e)}')
    
    def wait_for_fresh_image(self, image_type, timeout=2.0):
        """
        等待新鲜图像
        
        Args:
            image_type: 'rgb', 'fisheye_right', 'fisheye_left'
            timeout: 超时时间（秒）
            
        Returns:
            bool: 是否获得新鲜图像
        """
        import time
        start_time = time.time()
        original_seq = None
        
        # 获取当前图像序列号
        if image_type == 'rgb':
            original_seq = self.rgb_image_seq
        elif image_type == 'fisheye_right':
            original_seq = self.fisheye_image_seq
        elif image_type == 'fisheye_left':
            original_seq = self.left_fisheye_image_seq
        else:
            return False
            
        print(f"[状态机] 等待{image_type}新鲜图像，当前序列号: {original_seq}")
        
        # 等待图像更新
        while time.time() - start_time < timeout:
            current_seq = None
            if image_type == 'rgb':
                current_seq = self.rgb_image_seq
            elif image_type == 'fisheye_right':
                current_seq = self.fisheye_image_seq
            elif image_type == 'fisheye_left':
                current_seq = self.left_fisheye_image_seq
                
            if current_seq > original_seq:
                print(f"[状态机] 获得{image_type}新图像，序列号: {current_seq}")
                return True
                
            time.sleep(0.1)  # 等待100ms
        
        print(f"[状态机] 等待{image_type}新图像超时")
        return False
    
    def is_image_fresh(self, image_type, threshold=None):
        """
        检查图像是否新鲜
        
        Args:
            image_type: 'rgb', 'fisheye_right', 'fisheye_left'
            threshold: 时间阈值（秒），默认使用实例变量
            
        Returns:
            bool: 图像是否新鲜
        """
        import time
        if threshold is None:
            threshold = self.image_wait_threshold
            
        if not self.last_movement_completion_time:
            return True  # 没有运动完成记录，认为新鲜
        
        image_timestamp = None
        if image_type == 'rgb' and self.rgb_image_timestamp:
            image_timestamp = self.rgb_image_timestamp
        elif image_type == 'fisheye_right' and self.fisheye_image_timestamp:
            image_timestamp = self.fisheye_image_timestamp
        elif image_type == 'fisheye_left' and self.left_fisheye_image_timestamp:
            image_timestamp = self.left_fisheye_image_timestamp
            
        if not image_timestamp:
            return False  # 没有图像时间戳
        
        # 检查图像是否在运动完成后更新
        is_fresh = image_timestamp > (self.last_movement_completion_time + threshold)
        print(f"[状态机] {image_type}图像新鲜度检查: {'FRESH' if is_fresh else 'STALE'}")
        print(f"[状态机] 图像时间: {image_timestamp:.3f}, 运动完成时间: {self.last_movement_completion_time:.3f}")
        return is_fresh
    
    
    def check_rgb_position(self, config_name=None):
        """检查RGB相机中的位置 - 根据之前的运动类型决定调整方向"""
        # 检查运动完成后是否需要等待图像更新
        if (hasattr(self, 'last_movement_completion_time') and 
            self.last_movement_completion_time and 
            not self.is_image_fresh('rgb')):
            
            print("[状态机] RGB图像不新鲜，等待新图像...")
            if not self.wait_for_fresh_image('rgb'):
                print("[状态机] 等待RGB新图像失败，使用当前图像")
        
        if self.rgb_image is None:
            print("[状态机] RGB图像未就绪，等待...")
            return
        
        # 保存检查上下文
        self.position_check_context = {
            'check_type': 'rgb_position',
            'config_name': config_name,
            'image_type': 'rgb',
            'detection_type': 'ycy'
        }
        
        # 获取检查参数
        print(f"[调试] check_rgb_position 接收到的config_name: {config_name}")
        if config_name:
            print(f"[调试] 使用配置名称: {config_name}")
            roi_params, threshold_params = self.get_check_params(config_name, 'ycy')
            print(f"[调试] 获取的ROI参数: {roi_params}")
            print(f"[调试] 获取的阈值参数: {threshold_params}")
        else:
            print("[调试] 使用默认参数")
            roi_params, threshold_params = self.roi_params_ycy, self.threshold_params_ycy
            print(f"[调试] 默认ROI参数: {roi_params}")
            print(f"[调试] 默认阈值参数: {threshold_params}")
        
        # 保存关键RGB图像用于调试
        import cv2
        import time
        timestamp = int(time.time())
        debug_image_path = f"/tmp/rgb_debug_{timestamp}.jpg"
        cv2.imwrite(debug_image_path, self.rgb_image)
        print(f"[状态机] 已保存RGB调试图像: {debug_image_path}")
        
        # 使用RGB相机检测机器人在两条黄线中间的位置
        position = self.yellow_line_detector.detect_position(
            self.rgb_image,
            detection_type='ycy',  # 两条黄线中间检测
            roi_params=roi_params,
            camera_type='rgb',
            threshold_params=threshold_params
        )
        print(f"[状态机] RGB相机检测位置: {position}")
        
        # 检查是否达到最大修正次数
        if self.position_correction_attempts >= self.max_correction_attempts:
            print(f"[状态机] 已达到最大修正次数 {self.max_correction_attempts}，强制进入下一阶段")
            self.position_correction_attempts = 0
            self.proceed_to_next_stage()
            return
        
        # 根据之前的运动类型决定修正方式
        prev_movement_type = None
        if (hasattr(self, 'movement_sequence') and 
            hasattr(self, 'movement_step') and 
            self.movement_step > 0 and 
            self.movement_step <= len(self.movement_sequence)):
            # 获取前一个运动类型
            prev_movement = self.movement_sequence[self.movement_step - 1]
            if isinstance(prev_movement, tuple):
                prev_movement_type = prev_movement[0]
            else:
                prev_movement_type = prev_movement
            print(f"[状态机] 前一个运动类型: {prev_movement_type}")
        
        # RGB相机YCY位置修正逻辑
        if position == 'left':
            if prev_movement_type in ['turn_left', 'turn_right']:
                # 旋转运动 -> 旋转修正
                print("[状态机] RGB检测：机器人偏向左线，右转1步调整")
                self.execute_position_correction('turn_right', {'count': 1})
            else:
                # 平移运动 -> Y轴平移修正
                print("[状态机] RGB检测：机器人偏向左线，右走1步远离左线")
                self.execute_position_correction('right', {'count': 1})
        elif position == 'right':
            if prev_movement_type in ['turn_left', 'turn_right']:
                # 旋转运动 -> 旋转修正
                print("[状态机] RGB检测：机器人偏向右线，左转1步调整")
                self.execute_position_correction('turn_left', {'count': 1})
            else:
                # 平移运动 -> Y轴平移修正
                print("[状态机] RGB检测：机器人偏向右线，左走1步远离右线")
                self.execute_position_correction('left', {'count': 1})
        else:
            print("[状态机] 位置正确，在两条黄线中间，进入下一阶段")
            self.position_correction_attempts = 0
            self.proceed_to_next_stage()
    
    def check_rgb_distance(self, config_name=None):
        """检查RGB相机中的距离 - 用于确认到达特定位置"""
        # 检查运动完成后是否需要等待图像更新
        if (hasattr(self, 'last_movement_completion_time') and 
            self.last_movement_completion_time and 
            not self.is_image_fresh('rgb')):
            
            print("[状态机] RGB图像不新鲜，等待新图像...")
            if not self.wait_for_fresh_image('rgb'):
                print("[状态机] 等待RGB新图像失败，使用当前图像")
        
        if self.rgb_image is None:
            print("[状态机] RGB图像未就绪，使用默认逻辑进入下一阶段")
            self.proceed_to_next_stage()
            return
        
        # 保存检查上下文
        self.position_check_context = {
            'check_type': 'rgb_distance',
            'config_name': config_name,
            'image_type': 'rgb',
            'detection_type': 'dy'
        }
        
        # 获取检查参数
        if config_name:
            roi_params, threshold_params = self.get_check_params(config_name, 'dy')
        else:
            roi_params, threshold_params = self.roi_params_dy, self.threshold_params_dy
        
        # 保存调试图像
        import cv2
        import time
        timestamp = int(time.time())
        debug_image_path = f"/tmp/rgb_distance_debug_{timestamp}.jpg"
        cv2.imwrite(debug_image_path, self.rgb_image)
        print(f"[状态机] 已保存RGB距离检测调试图像: {debug_image_path}")
        
        # 使用距离检测功能
        distance_result = self.yellow_line_detector.detect_position(
            self.rgb_image,
            detection_type='dy',  # 距离黄线位置检测
            roi_params=roi_params,
            camera_type='rgb',
            target_position='center',
            threshold_params=threshold_params
        )
        print(f"[状态机] RGB距离检测结果: {distance_result}")
        
        # 检查是否达到最大修正次数
        if self.position_correction_attempts >= self.max_correction_attempts:
            print(f"[状态机] 已达到最大修正次数 {self.max_correction_attempts}，强制进入下一阶段")
            self.position_correction_attempts = 0
            self.proceed_to_next_stage()
            return
        
        # RGB DY位置修正：X轴移动(前方有线->前进2步；后方有线->后退2步)
        if distance_result == 'front':
            print("[状态机] RGB检测：前方有黄线，前进2步越过")
            self.execute_position_correction('forward', {'count': 2})
        elif distance_result == 'back':
            print("[状态机] RGB检测：后方有黄线，后退2步避开")
            self.execute_position_correction('backward', {'count': 2})
        else:
            print("[状态机] RGB距离检查完成，进入下一阶段")
            self.position_correction_attempts = 0
            self.proceed_to_next_stage()
    
    def check_right_fisheye_distance(self, config_name=None):
        """检查右鱼眼相机距离 - 用于到达QRB点前的位置确认"""
        # 检查运动完成后是否需要等待图像更新
        if (hasattr(self, 'last_movement_completion_time') and 
            self.last_movement_completion_time and 
            not self.is_image_fresh('fisheye_right')):
            
            print("[状态机] 右鱼眼图像不新鲜，等待新图像...")
            if not self.wait_for_fresh_image('fisheye_right'):
                print("[状态机] 等待右鱼眼新图像失败，使用当前图像")
        
        if self.fisheye_image is None:
            print("[状态机] 右鱼眼图像未就绪，使用默认逻辑进入下一阶段")
            self.proceed_to_next_stage()
            return
        
        # 保存检查上下文
        self.position_check_context = {
            'check_type': 'right_fisheye_distance',
            'config_name': config_name,
            'image_type': 'fisheye_right',
            'detection_type': 'dy'
        }
        
        # 获取检查参数
        if config_name:
            roi_params, threshold_params = self.get_check_params(config_name, 'dy')
        else:
            roi_params, threshold_params = self.roi_params_dy, self.threshold_params_dy
        
        # 保存调试图像
        import cv2
        import time
        timestamp = int(time.time())
        debug_image_path = f"/tmp/right_fisheye_distance_debug_{timestamp}.jpg"
        cv2.imwrite(debug_image_path, self.fisheye_image)
        print(f"[状态机] 已保存右鱼眼距离检测调试图像: {debug_image_path}")
        
        # 使用距离检测功能
        distance_result = self.yellow_line_detector.detect_position(
            self.fisheye_image,
            detection_type='dy',  # 距离黄线位置检测
            camera_type='fisheye_right',
            roi_params=roi_params,
            target_position='center',
            threshold_params=threshold_params
        )
        print(f"[状态机] 右鱼眼距离检测结果: {distance_result}")
        
        # 检查是否达到最大修正次数
        if self.position_correction_attempts >= self.max_correction_attempts:
            print(f"[状态机] 已达到最大修正次数 {self.max_correction_attempts}，强制进入下一阶段")
            self.position_correction_attempts = 0
            self.proceed_to_next_stage()
            return
        
        # 右鱼眼DY位置修正：Y轴移动(前方有线->右走1步；后方有线->左走1步)
        if distance_result == 'front':
            print("[状态机] 右鱼眼检测：前方有黄线，右走1步避开")
            self.execute_position_correction('right', {'count': 1})
        elif distance_result == 'back':
            print("[状态机] 右鱼眼检测：后方有黄线，左走1步避开")
            self.execute_position_correction('left', {'count': 1})
        else:
            print("[状态机] 右鱼眼距离检查完成，进入下一阶段")
            self.position_correction_attempts = 0
            self.proceed_to_next_stage()
    
    def check_left_fisheye_distance(self, config_name=None):
        """检查左鱼眼相机距离 - 用于到达QRB点前的位置确认"""
        # 检查运动完成后是否需要等待图像更新
        if (hasattr(self, 'last_movement_completion_time') and 
            self.last_movement_completion_time and 
            not self.is_image_fresh('fisheye_left')):
            
            print("[状态机] 左鱼眼图像不新鲜，等待新图像...")
            if not self.wait_for_fresh_image('fisheye_left'):
                print("[状态机] 等待左鱼眼新图像失败，使用当前图像")
        
        if hasattr(self, 'left_fisheye_image') and self.left_fisheye_image is not None:
            # 保存检查上下文
            self.position_check_context = {
                'check_type': 'left_fisheye_distance',
                'config_name': config_name,
                'image_type': 'fisheye_left',
                'detection_type': 'dy'
            }
            
            # 获取检查参数
            if config_name:
                roi_params, threshold_params = self.get_check_params(config_name, 'dy')
            else:
                roi_params, threshold_params = self.roi_params_dy, self.threshold_params_dy
            
            # 保存调试图像
            import cv2
            import time
            timestamp = int(time.time())
            debug_image_path = f"/tmp/left_fisheye_distance_debug_{timestamp}.jpg"
            cv2.imwrite(debug_image_path, self.left_fisheye_image)
            print(f"[状态机] 已保存左鱼眼距离检测调试图像: {debug_image_path}")
            
            # 使用距离检测功能
            distance_result = self.yellow_line_detector.detect_position(
                self.left_fisheye_image,
                detection_type='dy',  # 距离黄线位置检测
                camera_type='fisheye_left',
                roi_params=roi_params,
                target_position='center',
                threshold_params=threshold_params
            )
            print(f"[状态机] 左鱼眼距离检测结果: {distance_result}")
            
            # 检查是否达到最大修正次数
            if self.position_correction_attempts >= self.max_correction_attempts:
                print(f"[状态机] 已达到最大修正次数 {self.max_correction_attempts}，强制进入下一阶段")
                self.position_correction_attempts = 0
                self.proceed_to_next_stage()
                return
            
            # 左鱼眼DY位置修正：Y轴移动(前方有线->左走1步；后方有线->右走1步)
            if distance_result == 'front':
                print("[状态机] 左鱼眼检测：前方有黄线，左走1步避开")
                self.execute_position_correction('left', {'count': 1})
            elif distance_result == 'back':
                print("[状态机] 左鱼眼检测：后方有黄线，右走1步避开")
                self.execute_position_correction('right', {'count': 1})
            else:
                print("[状态机] 左鱼眼距离检查完成，进入下一阶段")
                self.position_correction_attempts = 0
                self.proceed_to_next_stage()
        else:
            print("[状态机] 左鱼眼图像未就绪，使用默认逻辑进入下一阶段")
            self.proceed_to_next_stage()
    
    def check_left_fisheye_ycy_position(self, config_name=None):
        """检查左鱼眼相机YCY位置"""
        if hasattr(self, 'left_fisheye_image') and self.left_fisheye_image is not None:
            # 获取检查参数
            if config_name:
                roi_params, threshold_params = self.get_check_params(config_name, 'ycy')
            else:
                roi_params, threshold_params = self.roi_params_ycy, self.threshold_params_ycy
            
            # 保存调试图像
            import cv2
            import time
            timestamp = int(time.time())
            debug_image_path = f"/tmp/left_fisheye_debug_{timestamp}.jpg"
            cv2.imwrite(debug_image_path, self.left_fisheye_image)
            print(f"[状态机] 已保存左鱼眼调试图像: {debug_image_path}")
            
            position = self.yellow_line_detector.detect_position(
                self.left_fisheye_image,
                detection_type='ycy',  # 两条黄线中间检测
                camera_type='fisheye_left',
                roi_params=roi_params,
                threshold_params=threshold_params
            )
            print(f"[状态机] 左鱼眼YCY检测位置: {position}")
            
            # 获取前一个运动类型
            prev_movement_type = None
            if (hasattr(self, 'movement_sequence') and 
                hasattr(self, 'movement_step') and 
                self.movement_step > 0 and 
                self.movement_step <= len(self.movement_sequence)):
                prev_movement = self.movement_sequence[self.movement_step - 1]
                if isinstance(prev_movement, tuple):
                    prev_movement_type = prev_movement[0]
                else:
                    prev_movement_type = prev_movement
                print(f"[状态机] 前一个运动类型: {prev_movement_type}")
            
            # 左鱼眼YCY位置修正逻辑
            if position == 'left':
                if prev_movement_type in ['turn_left', 'turn_right']:
                    # 旋转运动 -> 旋转修正
                    print("[状态机] 左鱼眼检测：机器人偏向左线，右转1步调整")
                    self.execute_position_correction('turn_right', {'count': 1})
                else:
                    # 平移运动 -> X轴平移修正
                    print("[状态机] 左鱼眼检测：机器人偏向左线，前进2步远离左线")
                    self.execute_position_correction('forward', {'count': 2})
            elif position == 'right':
                if prev_movement_type in ['turn_left', 'turn_right']:
                    # 旋转运动 -> 旋转修正
                    print("[状态机] 左鱼眼检测：机器人偏向右线，左转1步调整")
                    self.execute_position_correction('turn_left', {'count': 1})
                else:
                    # 平移运动 -> X轴平移修正
                    print("[状态机] 左鱼眼检测：机器人偏向右线，后退2步远离右线")
                    self.execute_position_correction('backward', {'count': 2})
            else:
                print("[状态机] 位置正确，在两条黄线中间，进入下一阶段")
                self.proceed_to_next_stage()
        else:
            print("[状态机] 左鱼眼图像未就绪，使用默认逻辑进入下一阶段")
            self.proceed_to_next_stage()
    
    def check_right_fisheye_ycy_position(self, config_name=None):
        """检查右鱼眼相机YCY位置"""
        # 检查运动完成后是否需要等待图像更新
        if (hasattr(self, 'last_movement_completion_time') and 
            self.last_movement_completion_time and 
            not self.is_image_fresh('fisheye_right')):
            
            print("[状态机] 右鱼眼图像不新鲜，等待新图像...")
            if not self.wait_for_fresh_image('fisheye_right'):
                print("[状态机] 等待右鱼眼新图像失败，使用当前图像")
        
        if self.fisheye_image is not None:
            # 保存检查上下文
            self.position_check_context = {
                'check_type': 'right_fisheye_ycy_position',
                'config_name': config_name,
                'image_type': 'fisheye_right',
                'detection_type': 'ycy'
            }
            
            # 获取检查参数
            if config_name:
                roi_params, threshold_params = self.get_check_params(config_name, 'ycy')
            else:
                roi_params, threshold_params = self.roi_params_ycy, self.threshold_params_ycy
            
            # 保存调试图像
            import cv2
            import time
            timestamp = int(time.time())
            debug_image_path = f"/tmp/right_fisheye_debug_{timestamp}.jpg"
            cv2.imwrite(debug_image_path, self.fisheye_image)
            print(f"[状态机] 已保存右鱼眼调试图像: {debug_image_path}")
            
            position = self.yellow_line_detector.detect_position(
                self.fisheye_image,
                detection_type='ycy',  # 两条黄线中间检测
                camera_type='fisheye_right',
                roi_params=roi_params,
                threshold_params=threshold_params
            )
            print(f"[状态机] 右鱼眼YCY检测位置: {position}")
            
            # 检查是否达到最大修正次数
            if self.position_correction_attempts >= self.max_correction_attempts:
                print(f"[状态机] 已达到最大修正次数 {self.max_correction_attempts}，强制进入下一阶段")
                self.position_correction_attempts = 0
                self.proceed_to_next_stage()
                return
            
            # 获取前一个运动类型
            prev_movement_type = None
            if (hasattr(self, 'movement_sequence') and 
                hasattr(self, 'movement_step') and 
                self.movement_step > 0 and 
                self.movement_step <= len(self.movement_sequence)):
                prev_movement = self.movement_sequence[self.movement_step - 1]
                if isinstance(prev_movement, tuple):
                    prev_movement_type = prev_movement[0]
                else:
                    prev_movement_type = prev_movement
                print(f"[状态机] 前一个运动类型: {prev_movement_type}")
            
            # 右鱼眼YCY位置修正逻辑
            if position == 'left':
                if prev_movement_type in ['turn_left', 'turn_right']:
                    # 旋转运动 -> 旋转修正
                    print("[状态机] 右鱼眼检测：机器人偏向左线，右转1步调整")
                    self.execute_position_correction('turn_right', {'count': 1})
                else:
                    # 平移运动 -> X轴平移修正
                    print("[状态机] 右鱼眼检测：机器人偏向左线，后退2步远离左线")
                    self.execute_position_correction('backward', {'count': 2})
            elif position == 'right':
                if prev_movement_type in ['turn_left', 'turn_right']:
                    # 旋转运动 -> 旋转修正
                    print("[状态机] 右鱼眼检测：机器人偏向右线，左转1步调整")
                    self.execute_position_correction('turn_left', {'count': 1})
                else:
                    # 平移运动 -> X轴平移修正
                    print("[状态机] 右鱼眼检测：机器人偏向右线，前进2步远离右线")
                    self.execute_position_correction('forward', {'count': 2})
            else:
                print("[状态机] 位置正确，在两条黄线中间，进入下一阶段")
                self.position_correction_attempts = 0
                self.proceed_to_next_stage()
        else:
            print("[状态机] 右鱼眼图像未就绪，使用默认逻辑进入下一阶段")
            self.proceed_to_next_stage()
    
    def forward_until_red_barrier(self):
        """前进直到检测到红色限高杆 - 启动前进并等待红色限高杆检测器触发停止"""
        print("[状态机] 开始前进，等待红色限高杆检测器触发停止")
        # 启动持续前进运动，等待red_barrier_callback中的停止逻辑
        self.send_motion_command('forward', {'count': 200,'vel_des':[0.2, 0.0, 0.0]})  # 设置足够大的次数，等待外部停止
    
    def forward_until_yellow_marker(self):
        """前进直到检测到黄色标志物 - 启动前进并等待黄色标志物检测器触发停止"""
        # 确保RGB相机正常运行
        if not self.camera_states['rgb_system']:
            print("[状态机] forward_until_yellow_marker需要RGB相机，启动RGB相机")
            self.smart_camera_switch('rgb')
            # 等待相机就绪
            import time
            time.sleep(1.0)
        
        print("[状态机] 开始前进，等待黄色标志物检测器触发停止")
        # 启动持续前进运动，等待yellow_marker_callback中的停止逻辑
        self.send_motion_command('forward', {'count': 200,'vel_des':[0.2, 0.0, 0.0]})  # 设置足够大的次数，等待外部停止
    

    
    def start_yellow_light_stop_sequence(self):
        """黄灯前停止并开始倒计时（已废弃，由独立节点处理）"""
        print("[状态机] 到达黄灯前停止位置，开始5秒倒计时")
        self.current_countdown = self.countdown_seconds
        self.transition_to_state(State.YELLOW_LIGHT_COUNTDOWN)
    
    def start_yellow_light_countdown_sequence(self):
        """执行黄灯倒计时（已废弃，由独立节点处理）"""
        print(f"[状态机] 黄灯倒计时: {self.current_countdown}")
        
        # 语音播报当前倒计时数字
        self.publish_voice_command(str(self.current_countdown))
        
        if self.current_countdown > 0:
            self.current_countdown -= 1
            # 创建1秒定时器，继续倒计时 - 使用状态绑定定时器
            self.countdown_timer = self.create_state_timer('countdown', 1.0, self.countdown_timer_callback, State.YELLOW_LIGHT_COUNTDOWN)
        else:
            print("[状态机] 黄灯倒计时完成，继续前进")
            # 继续下一个运动步骤
            self.proceed_to_next_stage()
    
    def countdown_timer_callback(self):
        """倒计时定时器回调（已废弃，由独立节点处理）"""
        # 取消定时器
        if hasattr(self, 'countdown_timer') and self.countdown_timer:
            self.countdown_timer.cancel()
            self.countdown_timer = None
        
        # 继续倒计时
        self.start_yellow_light_countdown_sequence()
    
    def publish_r1_countdown_trigger(self):
        """发布R1倒计时触发信号到voice_node"""
        try:
            # 发布信息触发voice_node开始倒计时，而不是依赖距离检测
            trigger_msg = String()
            trigger_msg.data = "R1_COUNTDOWN_START"  # 触发voice_node开始倒计时
            
            # 这里需要一个专门的话题来触发voice_node倒计时
            # 如果还没有这个发布者，我们需要在setup_communication中创建它
            if hasattr(self, 'voice_trigger_publisher'):
                self.voice_trigger_publisher.publish(trigger_msg)
                print("[状态机] 已发布R1倒计时触发信号")
            else:
                print("[状态机] 警告：voice_trigger_publisher不存在，需要在setup_communication中创建")
        except Exception as e:
            print(f"[状态机] 发布R1倒计时触发信号失败: {str(e)}")
    
    def publish_voice_command(self, message: str):
        """发布语音播报命令到语音节点（已废弃，由独立节点处理）"""
        try:
            # 这里应该发布到语音节点的话题
            # 假设语音节点订阅 /voice_command 话题
            voice_msg = String()
            voice_msg.data = message
            # 注意：需要在setup_communication中创建这个发布者
            # self.voice_command_publisher.publish(voice_msg)
            print(f"[状态机] 语音播报: {message}")
        except Exception as e:
            print(f"[状态机] 语音播报失败: {str(e)}")
    
    def execute_position_correction(self, movement_type, custom_params=None):
        """执行位置修正"""
        self.position_correction_attempts += 1
        print(f"[状态机] 执行位置修正: {movement_type} (第{self.position_correction_attempts}/{self.max_correction_attempts}次)")
        
        if custom_params is None:
            custom_params = {'count': 1}  # 只执行一步
        
        # 保存当前的位置检查类型，修正完成后需要重新检查
        self.pending_position_recheck = True
        self.send_motion_command(movement_type, custom_params)
    
    
    def proceed_to_next_stage(self):
        """进入下一阶段"""
        # 重置修正次数
        self.position_correction_attempts = 0
        # 继续执行运动序列的下一个动作
        self.movement_step += 1
        self.execute_next_movement()
    
    def transition_to_state(self, new_state):
        """状态转换"""
        if new_state != self.current_state:
            # 自动清理旧状态的定时器
            self.cleanup_state_timers(self.current_state)
            
            # 在状态转换前处理脚本管理
            self.handle_state_exit(self.current_state, new_state)
            
            self.previous_state = self.current_state
            self.current_state = new_state
            
            # 简约的状态输出
            self.print_status()
            
            # 发布状态信息
            state_msg = String()
            state_msg.data = f"State:{self.current_state.value}"
            self.state_publisher.publish(state_msg)
            
            # 节点管理：启动所需节点，停止不需要的节点
            self.start_required_nodes(new_state)
            self.stop_unused_nodes(new_state)
            
            # 执行状态进入动作
            self.on_state_entered(new_state)
    
    def handle_state_exit(self, current_state, new_state):
        """处理状态退出时的相机管理 - 确保RGB相机保持运行，不再切换AI相机"""
        # 注释掉AI相机切换逻辑，保持RGB相机全程运行
        # ai_required_states = [State.IN_S2, State.IN_QRB]
        
        # if current_state in ai_required_states and new_state not in ai_required_states:
        #     print(f"[状态机] 离开{current_state.name}状态，从AI相机切换回RGB相机")
        #     # 使用互斥切换确保AI相机停止，RGB相机启动
        #     self.smart_camera_switch('rgb')
        
        # 确保RGB相机保持运行
        rgb_required_states = [
            State.START_TO_QRA, State.QRA_TO_A1, State.QRA_TO_A2,
            State.A1_TO_S1, State.A2_TO_S1, State.IN_S1, State.S1_TO_S2,
            State.S2_TO_L1, State.S2_TO_R1, State.IN_L1, State.IN_R1,
            State.L1_TO_QRB, State.R1_TO_QRB, State.IN_S2, State.IN_QRB,  # 添加原本需要AI相机的状态
            State.QRB_TO_B1, State.QRB_TO_B2,
            State.L1_TO_S2, State.R1_TO_S2, State.IN_S2_R, State.S2_TO_S1,
            State.IN_S1_R, State.S1_TO_A1, State.S1_TO_A2
        ]
        
        if new_state in rgb_required_states and not self.camera_states['rgb_system']:
            print(f"[状态机] 进入{new_state.name}状态需要RGB相机，确保RGB相机运行")
            self.smart_camera_switch('rgb')
    
    def print_status(self):
        """打印当前状态"""
        status_map = {
            State.START: "等待启动命令",
            State.START_TO_QRA: "从起点运动到二维码A点",
            State.IN_QR_A: "二维码识别中",
            State.QRA_TO_A1: "从二维码A点运动到A1点", 
            State.QRA_TO_A2: "从二维码A点运动到A2点",
            State.IN_A1: "在A1点停止",
            State.IN_A2: "在A2点停止",
            State.A1_TO_S1: "从A1点运动到S1点",
            State.A2_TO_S1: "从A2点运动到S1点",
            State.IN_S1: "在S1点",
            State.S1_TO_S2: "从S1到S2（曲线赛道）",
            State.IN_S2: "在S2点（绿色箭头识别）",
            State.S2_TO_L1: "从S2到L1点",
            State.S2_TO_R1: "从S2到R1点", 
            State.IN_L1: "在L1点",
            State.IN_R1: "在R1点",
            State.L1_TO_QRB: "从L1到QRB点",
            State.R1_TO_QRB: "从R1到QRB点",
            State.IN_QRB: "在QRB点（二维码B识别）",
            State.QRB_TO_B1: "从QRB到B1库位",
            State.QRB_TO_B2: "从QRB到B2库位",
            State.IN_B1: "在B1库位",
            State.IN_B2: "在B2库位",
            State.B1_TO_B2: "从B1到B2库位",
            State.B2_TO_B1: "从B2到B1库位",
            State.B1_TO_L1: "从B1返回L1点",
            State.B1_TO_R1: "从B1返回R1点",
            State.B2_TO_L1: "从B2返回L1点",
            State.B2_TO_R1: "从B2返回R1点",
            State.L1_TO_S2: "从L1返回S2点",
            State.R1_TO_S2: "从R1返回S2点",
            State.IN_S2_R: "在S2点（返程）",
            State.S2_TO_S1: "从S2返回S1点",
            State.IN_S1_R: "在S1点（返程）",
            State.S1_TO_A1: "从S1返回A1点",
            State.S1_TO_A2: "从S1返回A2点",
            State.A1_TO_START: "从A1返回充电站",
            State.A2_TO_START: "从A2返回充电站",
            State.YELLOW_LIGHT_STOP: "黄灯前停止",
            State.YELLOW_LIGHT_COUNTDOWN: "黄灯倒计时",
            State.END: "任务结束",
            State.ERROR: "错误"
        }
        
        status = status_map.get(self.current_state, "未知状态")
        print(f"[状态机] 当前状态: {status}")
        
        if self.qr_result:
            print(f"[状态机] 二维码结果: {self.qr_result}")
        
        if self.task_completed:
            print("[状态机] 任务已完成")
    
    def on_state_entered(self, state):
        """状态进入时的处理"""
        if state == State.START_TO_QRA:
            self.start_to_qra_sequence()
        elif state == State.IN_QR_A:
            print("[状态机] IN_QR_A状态：使用RGB相机进行QR检测")
            # 确保qr_detector_node使用RGB相机话题
            if 'qr_detector_node' not in self.managed_nodes:
                config = self.node_configs['qr_detector_node']
                # 明确指定使用RGB相机话题
                extra_args = ['--ros-args', '-p', 'image_topic:=/image_rgb']
                self.start_node('qr_detector_node', config, extra_args)
            self.start_qr_detection()
        elif state == State.QRA_TO_A1:
            self.qra_to_a1_sequence()
        elif state == State.QRA_TO_A2:
            self.qra_to_a2_sequence()
        elif state == State.IN_A1:
            self.start_in_a1_sequence()
        elif state == State.IN_A2:
            self.start_in_a2_sequence()
        elif state == State.A1_TO_S1:
            self.start_a1_to_s1_sequence()
        elif state == State.A2_TO_S1:
            self.start_a2_to_s1_sequence()
        elif state == State.IN_S1:
            self.start_in_s1_sequence()
        elif state == State.S1_TO_S2:
            self.start_s1_to_s2_sequence()
        elif state == State.IN_S2:
            # 注释掉AI相机切换，继续使用RGB相机
            # if self.smart_camera_switch('ai'):
            #     self.start_in_s2_sequence()
            # else:
            #     print("[状态机] 相机切换失败，使用当前相机继续")
            #     self.start_in_s2_sequence()
            print("[状态机] IN_S2状态：继续使用RGB相机")
            self.start_in_s2_sequence()
        elif state == State.S2_TO_L1:
            self.start_s2_to_l1_sequence()
        elif state == State.S2_TO_R1:
            self.start_s2_to_r1_sequence()
        elif state == State.IN_L1:
            self.start_in_l1_sequence()
        elif state == State.IN_R1:
            self.start_in_r1_sequence()
        elif state == State.L1_TO_QRB:
            self.start_l1_to_qrb_sequence()
        elif state == State.R1_TO_QRB:
            self.start_r1_to_qrb_sequence()
        elif state == State.IN_QRB:
            # 注释掉AI相机切换，继续使用RGB相机，启动qr_detector_node
            # if self.smart_camera_switch('ai'):
            #     # 启动qr_detector_node节点，使用特殊参数
            #     if 'qr_detector_node' not in self.managed_nodes:
            #         config = self.node_configs['qr_detector_node']
            #         extra_args = ['--ros-args', '-p', 'image_topic:=/mi_desktop_48_b0_2d_7b_03_d0/image']
            #         self.start_node('qr_detector_node', config, extra_args)
            #     self.start_qrb_detection()
            # else:
            #     print("[状态机] 相机切换失败，使用当前相机继续")
            #     # 仍然启动QR检测
            #     if 'qr_detector_node' not in self.managed_nodes:
            #         config = self.node_configs['qr_detector_node']
            #         extra_args = ['--ros-args', '-p', 'image_topic:=/mi_desktop_48_b0_2d_7b_03_d0/image']
            #         self.start_node('qr_detector_node', config, extra_args)
            #     self.start_qrb_detection()
            print("[状态机] IN_QRB状态：继续使用RGB相机进行QR检测")
            # 启动qr_detector_node节点，使用RGB相机话题
            if 'qr_detector_node' not in self.managed_nodes:
                config = self.node_configs['qr_detector_node']
                # 修改为使用RGB相机话题
                extra_args = ['--ros-args', '-p', 'image_topic:=/image_rgb']
                self.start_node('qr_detector_node', config, extra_args)
            self.start_qrb_detection()
        elif state == State.QRB_TO_B1:
            self.start_qrb_to_b1_sequence()
        elif state == State.QRB_TO_B2:
            self.start_qrb_to_b2_sequence()
        elif state == State.IN_B1:
            self.start_in_b1_sequence()
        elif state == State.IN_B2:
            self.start_in_b2_sequence()
        elif state == State.B1_TO_B2:
            self.start_b1_to_b2_sequence()
        elif state == State.B2_TO_B1:
            self.start_b2_to_b1_sequence()
        elif state == State.B1_TO_L1:
            self.start_b1_to_l1_sequence()
        elif state == State.B1_TO_R1:
            self.start_b1_to_r1_sequence()
        elif state == State.B2_TO_L1:
            self.start_b2_to_l1_sequence()
        elif state == State.B2_TO_R1:
            self.start_b2_to_r1_sequence()
        elif state == State.L1_TO_S2:
            self.start_l1_to_s2_sequence()
        elif state == State.R1_TO_S2:
            self.start_r1_to_s2_sequence()
        elif state == State.IN_S2_R:
            self.start_in_s2_r_sequence()
        elif state == State.S2_TO_S1:
            self.start_s2_to_s1_sequence()
        elif state == State.IN_S1_R:
            self.start_in_s1_r_sequence()
        elif state == State.S1_TO_A1:
            self.start_s1_to_a1_sequence()
        elif state == State.S1_TO_A2:
            self.start_s1_to_a2_sequence()
        elif state == State.A1_TO_START:
            self.start_a1_to_start_sequence()
        elif state == State.A2_TO_START:
            self.start_a2_to_start_sequence()
        elif state == State.YELLOW_LIGHT_STOP:
            self.start_yellow_light_stop_sequence()
        elif state == State.YELLOW_LIGHT_COUNTDOWN:
            self.start_yellow_light_countdown_sequence()
        elif state == State.END:
            self.on_task_completed()
        elif state == State.ERROR:
            self.on_error_state()
    
    def start_to_qra_sequence(self):
        """start_to_qra状态：站起->前进->检查鱼眼位置->右转->检查RGB位置"""
        print("[状态机] 开始start_to_qra运动序列")
        self.movement_sequence = [
            ('stand', None),                               # 运动类型8: 站立
            ('forward', {'count': 12}),                    # 运动类型1: 向前
            ('check_right_fisheye_ycy_position', 'start_to_qra_fisheye_ycy'),
            ('turn_right', {'count': 11}),                 # 运动类型5: 右转
            ('check_right_fisheye_ycy_position', 'start_to_qra_fisheye_ycy'),

        ]
        self.movement_step = 0
        self.execute_next_movement()
    
    def qra_to_a1_sequence(self):
        """qra_to_a1状态：前23,检查-右鱼眼-ycy,右21,检查-右鱼眼-dy，后9,检查-rgb-dy"""
        print("[状态机] 开始qra_to_a1运动序列")
        forward_steps = max(24 - self.z1_steps, 0)  # 确保步数不为负
        print(f"[状态机] QR识别前进了{self.z1_steps}步，现在前进{forward_steps}步(24-{self.z1_steps})")
        self.movement_sequence = [
            ('forward', {'count': forward_steps}),             # 前24-z1步
            #            ('check_right_fisheye_ycy_position', None)
            ('right', {'count': 21}),                          # 右21步
            #            ('check_right_fisheye_distance', None)
            ('backward', {'count': 9}),                        # 后9步
            #            ('check_rgb_distance', None)
        ]
        self.movement_step = 0
        self.execute_next_movement()
    
    def qra_to_a2_sequence(self):
        """qra_to_a2状态：前23,检查-左鱼眼-ycy,左21,检查-左鱼眼-dy，后9,检查-rgb-dy"""
        print("[状态机] 开始qra_to_a2运动序列")
        forward_steps = max(24 - self.z1_steps, 0)  # 确保步数不为负 
        print(f"[状态机] QR识别前进了{self.z1_steps}步，现在前进{forward_steps}步(24-{self.z1_steps})")
        self.movement_sequence = [
            ('forward', {'count': forward_steps}),             # 前24-z1步
            #            ('check_left_fisheye_ycy_position', None)
            ('left', {'count': 21}),                           # 左21步
            #            ('check_left_fisheye_distance', None)
            ('backward', {'count': 9}),                        # 后9步
            #            ('check_rgb_distance', None)
        ]
        self.movement_step = 0
        self.execute_next_movement()
    
    def start_in_a1_sequence(self):
        """in_a1状态：根据前一状态决定下一状态，执行lie_down"""
        print(f"[状态机] 在A1点，执行lie_down (前一状态: {self.previous_state.name})")
        
        # 根据前一状态决定下一状态
        if self.previous_state == State.QRA_TO_A1:
            self.a1_next_state = State.A1_TO_S1      # 前进到中间区域
            print("[状态机] 前一状态为QRA_TO_A1，下一状态将为A1_TO_S1")
        elif self.previous_state == State.S1_TO_A1:
            self.a1_next_state = State.A1_TO_START   # 返程到充电站
            print("[状态机] 前一状态为S1_TO_A1，下一状态将为A1_TO_START")
        else:
            # 默认情况（异常情况）
            self.a1_next_state = State.A1_TO_S1
            print(f"[状态机] 前一状态为{self.previous_state.name}，使用默认下一状态A1_TO_S1")
        
        self.movement_sequence = ['lie_down']
        self.movement_step = 0
        self.execute_next_movement()
    
    def start_in_a2_sequence(self):
        """in_a2状态：根据前一状态决定下一状态，执行lie_down"""
        print(f"[状态机] 在A2点，执行lie_down (前一状态: {self.previous_state.name})")
        
        # 根据前一状态决定下一状态
        if self.previous_state == State.QRA_TO_A2:
            self.a2_next_state = State.A2_TO_S1      # 前进到中间区域
            print("[状态机] 前一状态为QRA_TO_A2，下一状态将为A2_TO_S1")
        elif self.previous_state == State.S1_TO_A2:
            self.a2_next_state = State.A2_TO_START   # 返程到充电站
            print("[状态机] 前一状态为S1_TO_A2，下一状态将为A2_TO_START")
        else:
            # 默认情况（异常情况）
            self.a2_next_state = State.A2_TO_S1
            print(f"[状态机] 前一状态为{self.previous_state.name}，使用默认下一状态A2_TO_S1")
        
        self.movement_sequence = ['lie_down']
        self.movement_step = 0
        self.execute_next_movement()
    
    def start_a1_to_s1_sequence(self):
        """a1_to_s1状态：站立，前9,检查-左鱼眼-ycy，左21，检查-右鱼眼-dy，后24，检查-rgb-dy，左转11，检查-右鱼眼-ycy"""
        print("[状态机] 开始a1_to_s1运动序列")
        self.movement_sequence = [
            ('stand', None),                               # 运动类型8: 站立
            ('forward', {'count': 9}),                     # 前进9步
            # ('check_left_fisheye_ycy_position', None),                      # 检查左鱼眼YCY
            ('left', {'count': 21}),                       # 左21步
            # ('check_right_fisheye_distance', None),                      # 检查右鱼眼DY
            ('backward', {'count': 24}),                   # 后24步
            # ('check_rgb_distance', None),                      # 检查RGB DY
            ('turn_left', {'count': 11}),                  # 左转11步
            # ('check_right_fisheye_ycy_position', None),                      # 检查右鱼眼YCY
        ]
        self.movement_step = 0
        self.execute_next_movement()

    def start_a2_to_s1_sequence(self):
        """a2_to_s1状态：站立，前9,检查-右鱼眼-ycy，右21，检查-左鱼眼-dy，后24，检查-rgb-dy，左转11，检查-右鱼眼-ycy"""
        print("[状态机] 开始a2_to_s1运动序列")
        self.movement_sequence = [
            ('stand', None),                               # 运动类型8: 站立
            ('forward', {'count': 9}),                     # 前进9步
            # ('check_right_fisheye_ycy_position', None),                      # 检查右鱼眼YCY
            ('right', {'count': 21}),                      # 右21步
            # ('check_left_fisheye_distance', None),                      # 检查左鱼眼DY
            ('backward', {'count': 24}),                   # 后24步
            # ('check_rgb_distance', None),                      # 检查RGB DY
            ('turn_left', {'count': 11}),                  # 左转11步
            # ('check_right_fisheye_ycy_position', None),                      # 检查右鱼眼YCY
        ]
        self.movement_step = 0
        self.execute_next_movement()

    def start_in_s1_sequence(self):
        """in_s1状态：直接进入下一状态"""
        print("[状态机] 在S1点，直接进入s1_to_s2状态")
        self.transition_to_state(State.S1_TO_S2)
    
    def start_s1_to_s2_sequence(self):
        """s1_to_s2状态：调用两条黄线走中间模块"""
        print("[状态机] 开始s1_to_s2序列：调用两条黄线走中间模块")
        # 发送开始命令给黄线走中间控制模块
        command_msg = String()
        command_msg.data = "START"
        self.yellow_line_control_publisher.publish(command_msg)
        
        # 等待一段时间后自动进入下一状态（实际应该根据模块完成信号）
        def goto_s2():
            self.transition_to_state(State.IN_S2)
        self.create_state_timer('s1_to_s2_complete', 5.0, goto_s2, State.S1_TO_S2)
    
    def start_in_s2_sequence(self):
        """in_s2状态：识别RGB图片中的绿色箭头方向"""
        print("[状态机] 在S2点，等待绿色箭头方向识别")
        # 记录绿色箭头识别开始时间
        self.green_arrow_detection_start_time = time.time()
        # 等待绿色箭头识别结果，在green_arrow_callback中处理状态转换
    
    def start_s2_to_l1_sequence(self):
        """s2_to_l1状态：左转，检查RGB位置，前进到红色限高杆"""
        print("[状态机] 开始s2_to_l1运动序列")
        self.movement_sequence = [
            ('left', {'count': 21}),                           # 左转21步
            # ('check_rgb_position', None),                      # 检查RGB位置
            ('forward_until_red_barrier', None)                # 前进直到红色限高杆
        ]
        self.movement_step = 0
        self.execute_next_movement()
    
    def start_s2_to_r1_sequence(self):
        """s2_to_r1状态：右转，检查RGB位置，前进到黄色标志物"""
        print("[状态机] 开始s2_to_r1运动序列")
        self.movement_sequence = [
            ('right', {'count': 21}),                          # 右转21步
            # ('check_rgb_position', None),                      # 检查RGB位置
            ('forward_until_yellow_marker', None)              # 前进直到黄色标志物
        ]
        self.movement_step = 0
        self.execute_next_movement()
    
    def start_in_l1_sequence(self):
        """in_l1状态：根据前一状态决定下一状态，执行特殊步态"""
        print(f"[状态机] 在L1点，执行特殊步态 (前一状态: {self.previous_state.name})")
        
        # 根据前一状态决定下一状态
        if self.previous_state == State.S2_TO_L1:
            self.l1_next_state = State.L1_TO_QRB
            print("[状态机] 前一状态为S2_TO_L1，下一状态将为L1_TO_QRB")
        elif self.previous_state in [State.B1_TO_L1, State.B2_TO_L1]:
            self.l1_next_state = State.L1_TO_S2  # 返程状态
            print(f"[状态机] 前一状态为{self.previous_state.name}，下一状态将为L1_TO_S2")
        else:
            # 默认情况（异常情况）
            self.l1_next_state = State.L1_TO_QRB
            print(f"[状态机] 前一状态为{self.previous_state.name}，使用默认下一状态L1_TO_QRB")
        
        # 预留特殊步态实现位置
        # TODO: 实现特殊步态
        
        # 暂时用普通前进代替特殊步态
        self.movement_sequence = [
            ('lie_down', None),                                # 趴下
        
        ]
        self.movement_step = 0
        self.execute_next_movement()
    
    def start_in_r1_sequence(self):
        """in_r1状态：发布信息触发voice_node开始倒计时，等待voice_node的完成信号，根据前一状态决定下一状态"""
        print(f"[状态机] 在R1点，发布倒计时触发信号 (前一状态: {self.previous_state.name})")
        
        # 发布信息触发voice_node开始倒计时
        self.publish_r1_countdown_trigger()
        
        # 设置等待voice_node完成信号的状态
        self.waiting_for_voice = True
        self.voice_start_time = time.time()
        
        # 根据前一状态决定下一状态
        if self.previous_state == State.S2_TO_R1:
            self.r1_next_state = State.R1_TO_QRB
            print("[状态机] 前一状态为S2_TO_R1，下一状态将为R1_TO_QRB")
        elif self.previous_state in [State.B1_TO_R1, State.B2_TO_R1]:
            self.r1_next_state = State.R1_TO_S2  # 返程状态
            print(f"[状态机] 前一状态为{self.previous_state.name}，下一状态将为R1_TO_S2")
        else:
            # 默认情况（异常情况）
            self.r1_next_state = State.R1_TO_QRB
            print(f"[状态机] 前一状态为{self.previous_state.name}，使用默认下一状态R1_TO_QRB")
        
        # 启动超时定时器（10秒）以防voice_node无响应
        def r1_voice_timeout():
            if self.waiting_for_voice:
                print("[状态机] R1状态等待voice_node超时，强制转到下一状态")
                self.waiting_for_voice = False
                self._handle_r1_voice_complete()
        
        self.create_state_timer('r1_voice_timeout', 10.0, r1_voice_timeout, State.IN_R1)
    
    def _handle_r1_voice_complete(self):
        """处理R1状态语音播报完成"""
        print("[状态机] 处理R1状态语音播报完成")
        
        # 关闭黄色标志物识别节点
        if 'yellow_marker_detector' in self.managed_nodes:
            print("[状态机] 关闭黄色标志物识别节点")
            self.stop_node('yellow_marker_detector')
        
        # 在转到下一状态前，先执行12步前进，速度为0.2
        print("[状态机] R1状态播报完成后，向前走12步，速度0.2")
        next_state = getattr(self, 'r1_next_state', State.R1_TO_QRB)
        
        # 执行12步前进运动，使用自定义参数设置速度和步数
        self._execute_motion_command('forward', {
            'count': 12,
            'vel_des': [0.2, 0.0, 0.0]
        })
        
        # 设置一个定时器，在运动完成后转到下一状态
        def r1_forward_complete():
            print(f"[状态机] R1状态前进12步完成，转到下一状态: {next_state.name}")
            self.transition_to_state(next_state)
        
        # 创建定时器，等待前进运动完成（12步 * 0.2秒/步 = 2.4秒 + 缓冲时间）
        self.create_state_timer('r1_forward_complete', 3.0, r1_forward_complete, State.IN_R1)
    
    def _handle_l1_sequence_complete(self):
        """处理L1状态序列完成"""
        print("[状态机] 处理L1状态序列完成")
        
        # 关闭红色限高杆识别节点
        if 'red_barrier_detector' in self.managed_nodes:
            print("[状态机] 关闭红色限高杆识别节点")
            self.stop_node('red_barrier_detector')
        
        # 转到下一状态（根据前一状态决定）
        next_state = getattr(self, 'l1_next_state', State.L1_TO_QRB)
        print(f"[状态机] L1状态转到下一状态: {next_state.name}")
        self.transition_to_state(next_state)
    
    def _handle_a1_sequence_complete(self):
        """处理A1状态序列完成"""
        print("[状态机] 处理A1状态序列完成")
        
        # A1状态不需要关闭特定识别节点
        
        # 转到下一状态（根据前一状态决定）
        next_state = getattr(self, 'a1_next_state', State.A1_TO_S1)
        print(f"[状态机] A1状态转到下一状态: {next_state.name}")
        self.transition_to_state(next_state)
    
    def _handle_a2_sequence_complete(self):
        """处理A2状态序列完成"""
        print("[状态机] 处理A2状态序列完成")
        
        # A2状态不需要关闭特定识别节点
        
        # 转到下一状态（根据前一状态决定）
        next_state = getattr(self, 'a2_next_state', State.A2_TO_S1)
        print(f"[状态机] A2状态转到下一状态: {next_state.name}")
        self.transition_to_state(next_state)
    
    def start_l1_to_qrb_sequence(self):
        """l1_to_qrb状态：前进(150-x1)步，检查RGB距离，右转，检查右鱼眼距离"""
        print("[状态机] 开始l1_to_qrb运动序列")
        forward_steps = max(150 - self.x1_steps, 0)
        self.movement_sequence = [
            ('forward', {'count': forward_steps}),             # 前进(150-x1)步
            # ('check_rgb_distance', None),                      # 检查RGB距离
            ('right', {'count': 21}),                          # 右转21步  
            # ('check_right_fisheye_distance', None),                      # 检查右鱼眼距离
        ]
        self.movement_step = 0
        self.execute_next_movement()
    
    def start_r1_to_qrb_sequence(self):
        """r1_to_qrb状态：前进(150-y1)步，检查RGB距离，左转，检查左鱼眼距离"""
        print("[状态机] 开始r1_to_qrb运动序列")
        forward_steps = max(150 - self.y1_steps, 0)
        self.movement_sequence = [
            ('forward', {'count': forward_steps}),             # 前进(150-y1)步
            # ('check_rgb_distance', None),                      # 检查RGB距离
            ('left', {'count': 21}),                           # 左转21步
            # ('check_left_fisheye_distance', None),                      # 检查左鱼眼距离
        ]
        self.movement_step = 0
        self.execute_next_movement()
    
    def start_qrb_to_b1_sequence(self):
        """qrb_to_b1状态：左21,检查-rgb-ycy，前9,检查-rgb-dy"""
        print("[状态机] 开始qrb_to_b1运动序列")
        self.movement_sequence = [
            ('left', {'count': 21}),                           # 左转21步
            # ('check_rgb_position', None),                      # 检查RGB位置
            ('forward', {'count': 9}),                         # 前进9步
            # ('check_rgb_distance', None),                      # 检查RGB距离
        ]
        self.movement_step = 0
        self.execute_next_movement()
    
    def start_qrb_to_b2_sequence(self):
        """qrb_to_b2状态：右21,检查-rgb-ycy，前9,检查-rgb-dy"""
        print("[状态机] 开始qrb_to_b2运动序列")
        self.movement_sequence = [
            ('right', {'count': 21}),                          # 右转21步
            # ('check_rgb_position', None),                      # 检查RGB位置
            ('forward', {'count': 9}),                         # 前进9步
            # ('check_rgb_distance', None),                      # 检查RGB距离
        ]
        self.movement_step = 0
        self.execute_next_movement()
    
    def start_in_b1_sequence(self):
        """in_b1状态：趴下，语音交互（预留位置）"""
        print("[状态机] 开始in_b1序列：在B1库位趴下等待交互")
        self.movement_sequence = [
            ('lie_down', None),                                # 趴下
            # 语音交互预留位置
        ]
        self.movement_step = 0
        self.execute_next_movement()
    
    def start_in_b2_sequence(self):
        """in_b2状态：趴下，语音交互（预留位置）"""
        print("[状态机] 开始in_b2序列：在B2库位趴下等待交互")
        self.movement_sequence = [
            ('lie_down', None),                                # 趴下
            # 语音交互预留位置
        ]
        self.movement_step = 0
        self.execute_next_movement()
    
    def start_b1_to_b2_sequence(self):
        """b1_to_b2状态：站起，后9,检查-rgb-dy，右50,检查-右鱼眼-dy，前9,检查-rgb-dy"""
        print("[状态机] 开始b1_to_b2运动序列")
        self.movement_sequence = [
            ('stand', None),                                   # 站立
            ('backward', {'count': 9}),                        # 后退9步
            # ('check_rgb_distance', None),                      # 检查RGB距离
            ('right', {'count': 50}),                          # 右移50步
            # ('check_right_fisheye_distance', None),                      # 检查右鱼眼距离
            ('forward', {'count': 9}),                         # 前进9步
            # ('check_rgb_distance', None),                      # 检查RGB距离
        ]
        self.movement_step = 0
        self.execute_next_movement()
    
    def start_b2_to_b1_sequence(self):
        """b2_to_b1状态：站起，后9,检查-rgb-dy,左50,检查-左鱼眼-dy,前9,检查-rgb-dy"""
        print("[状态机] 开始b2_to_b1运动序列")
        self.movement_sequence = [
            ('stand', None),                                   # 站立
            ('backward', {'count': 9}),                        # 后退9步
            # ('check_rgb_distance', None),                      # 检查RGB距离
            ('left', {'count': 50}),                           # 左移50步
            # ('check_left_fisheye_distance', None),                      # 检查左鱼眼距离
            ('forward', {'count': 9}),                         # 前进9步
            # ('check_rgb_distance', None),                      # 检查RGB距离
        ]
        self.movement_step = 0
        self.execute_next_movement()
    
    def start_b1_to_l1_sequence(self):
        """b1_to_l1状态：右转18,检查-rgb-ycy,前进一直到rgb识别到红色限高杆，记录步数为x2"""
        print("[状态机] 开始b1_to_l1运动序列")
        self.movement_sequence = [
            ('turn_right', {'count': 18}),                     # 右转18步
            # ('check_rgb_position', None),                      # 检查RGB位置
            ('forward_until_red_barrier', None)                # 前进直到红色限高杆
        ]
        self.movement_step = 0
        self.execute_next_movement()
    
    def start_b1_to_r1_sequence(self):
        """b1_to_r1状态：右转18,检查-rgb-ycy,前9,检查-左鱼眼-ycy,左50,检查-左鱼眼-dy,前进一直到黄色标志物，记录步数为y2"""
        print("[状态机] 开始b1_to_r1运动序列")
        self.movement_sequence = [
            ('turn_right', {'count': 18}),                     # 右转18步
            # ('check_rgb_position', None),                      # 检查RGB位置
            ('forward', {'count': 9}),                         # 前进9步
            # ('check_left_fisheye_ycy_position', None),                      # 检查左鱼眼YCY位置
            ('left', {'count': 50}),                           # 左移50步
            # ('check_left_fisheye_distance', None),                      # 检查左鱼眼距离
            ('forward_until_yellow_marker', None)              # 前进直到黄色标志物
        ]
        self.movement_step = 0
        self.execute_next_movement()
    
    def start_b2_to_l1_sequence(self):
        """b2_to_l1状态：右转18,检查-rgb-ycy,前9,检查-右鱼眼-ycy,右50,检查-右鱼眼-dy,前进一直到红色限高杆，记录步数为x2"""
        print("[状态机] 开始b2_to_l1运动序列")
        self.movement_sequence = [
            ('turn_right', {'count': 18}),                     # 右转18步
            # ('check_rgb_position', None),                      # 检查RGB位置
            ('forward', {'count': 9}),                         # 前进9步
            # ('check_right_fisheye_ycy_position', None),                      # 检查右鱼眼YCY位置
            ('right', {'count': 50}),                          # 右移50步
            # ('check_right_fisheye_distance', None),                      # 检查右鱼眼距离
            ('forward_until_red_barrier', None)                # 前进直到红色限高杆
        ]
        self.movement_step = 0
        self.execute_next_movement()
    
    def start_b2_to_r1_sequence(self):
        """b2_to_r1状态：右转18,检查-rgb-ycy,前进一直到黄色标志物，记录步数为y2"""
        print("[状态机] 开始b2_to_r1运动序列")
        self.movement_sequence = [
            ('turn_right', {'count': 18}),                     # 右转18步
            # ('check_rgb_position', None),                      # 检查RGB位置
            ('forward_until_yellow_marker', None)              # 前进直到黄色标志物
        ]
        self.movement_step = 0
        self.execute_next_movement()
    
    def start_l1_to_s2_sequence(self):
        """l1_to_s2状态：前150-x2,检查-rgb-dy,右21,检查-rgb-ycy"""
        print("[状态机] 开始l1_to_s2运动序列（返程）")
        forward_steps = max(150 - self.x2_steps, 0)
        self.movement_sequence = [
            ('forward', {'count': forward_steps}),             # 前进(150-x2)步
            # ('check_rgb_distance', None),                      # 检查RGB距离
            ('right', {'count': 21}),                          # 右转21步
            # ('check_rgb_position', None),                      # 检查RGB位置
        ]
        self.movement_step = 0
        self.execute_next_movement()
    
    def start_r1_to_s2_sequence(self):
        """r1_to_s2状态：前150-y2,检查-rgb-dy,左21,检查-rgb-ycy"""
        print("[状态机] 开始r1_to_s2运动序列（返程）")
        forward_steps = max(150 - self.y2_steps, 0)
        self.movement_sequence = [
            ('forward', {'count': forward_steps}),             # 前进(150-y2)步
            # ('check_rgb_distance', None),                      # 检查RGB距离
            ('left', {'count': 21}),                           # 左转21步
            # ('check_rgb_position', None),                      # 检查RGB位置
        ]
        self.movement_step = 0
        self.execute_next_movement()
    
    def start_in_s2_r_sequence(self):
        """in_s2_R状态：等待一分钟"""
        print("[状态机] 在S2点（返程），等待一分钟")
        # 创建60秒定时器
        def goto_s1():
            self.transition_to_state(State.S2_TO_S1)
        self.create_state_timer('s2_r_wait_complete', 60.0, goto_s1, State.IN_S2_R)
    
    def start_s2_to_s1_sequence(self):
        """s2_to_s1状态：调用两条黄线走中间模块（返程）"""
        print("[状态机] 开始s2_to_s1序列：调用两条黄线走中间模块（返程）")
        # 发送开始命令给黄线走中间控制模块
        command_msg = String()
        command_msg.data = "START"
        self.yellow_line_control_publisher.publish(command_msg)
        
        # 等待一段时间后自动进入下一状态
        def goto_s1_r():
            self.transition_to_state(State.IN_S1_R)
        self.create_state_timer('s2_to_s1_complete', 5.0, goto_s1_r, State.S2_TO_S1)
    
    def start_in_s1_r_sequence(self):
        """in_s1_R状态：等待一分钟，根据初始二维码结果决定下一状态"""
        print("[状态机] 在S1点（返程），等待一分钟")
        print(f"[状态机] 初始二维码结果: {self.initial_qr_result}")
        
        # 根据初始二维码结果决定下一状态
        if self.initial_qr_result == "A-2":
            next_state = State.S1_TO_A1
            print("[状态机] 初始为A-2，返程到A1")
        elif self.initial_qr_result == "A-1":
            next_state = State.S1_TO_A2
            print("[状态机] 初始为A-1，返程到A2")
        else:
            print(f"[状态机] 未知的初始二维码结果: {self.initial_qr_result}，默认到A1")
            next_state = State.S1_TO_A1
        
        # 创建60秒定时器
        def goto_next_state():
            self.transition_to_state(next_state)
        self.create_state_timer('s1_r_wait_complete', 60.0, goto_next_state, State.IN_S1_R)
    
    def start_s1_to_a1_sequence(self):
        """s1_to_a1状态：左转9，检查-rgb-ycy,前23,检查-右鱼眼-ycy,右21,检查-右鱼眼-dy,后9,检查-rgb-dy"""
        print("[状态机] 开始s1_to_a1运动序列（返程）")
        self.movement_sequence = [
            ('turn_left', {'count': 9}),                       # 左转9步
            # ('check_rgb_position', None),                      # 检查RGB位置
            ('forward', {'count': 23}),                        # 前进23步
            # ('check_right_fisheye_ycy_position', None),                      # 检查右鱼眼YCY位置
            ('right', {'count': 21}),                          # 右移21步
            # ('check_right_fisheye_distance', None),                      # 检查右鱼眼距离
            ('backward', {'count': 9}),                        # 后退9步
            # ('check_rgb_distance', None),                      # 检查RGB距离
        ]
        self.movement_step = 0
        self.execute_next_movement()
    
    def start_s1_to_a2_sequence(self):
        """s1_to_a2状态：左转9，检查-rgb-ycy,前23,检查-左鱼眼-ycy,左21,检查-左鱼眼-dy,后9,检查-rgb-dy"""
        print("[状态机] 开始s1_to_a2运动序列（返程）")
        self.movement_sequence = [
            ('turn_left', {'count': 9}),                       # 左转9步
            # ('check_rgb_position', None),                      # 检查RGB位置
            ('forward', {'count': 23}),                        # 前进23步
            # ('check_left_fisheye_ycy_position', None),                      # 检查左鱼眼YCY位置
            ('left', {'count': 21}),                           # 左移21步
            # ('check_left_fisheye_distance', None),                      # 检查左鱼眼距离
            ('backward', {'count': 9}),                        # 后退9步
            # ('check_rgb_distance', None),                      # 检查RGB距离
        ]
        self.movement_step = 0
        self.execute_next_movement()
    
    def start_a1_to_start_sequence(self):
        """a1_to_start状态：站起，前9,检查-左鱼眼-ycy,左21,检查-右鱼眼-dy,后23,检查-右鱼眼-ycy，左转9，检查-右鱼眼-ycy，后9,检查-rgb-dy，趴下"""
        print("[状态机] 开始a1_to_start运动序列（返回充电站）")
        self.movement_sequence = [
            ('stand', None),                                   # 站立
            ('forward', {'count': 9}),                         # 前进9步
            # ('check_left_fisheye_ycy_position', None),                      # 检查左鱼眼YCY位置
            ('left', {'count': 21}),                           # 左移21步
            # ('check_right_fisheye_distance', None),                      # 检查右鱼眼距离
            ('backward', {'count': 23}),                       # 后退23步
            # ('check_right_fisheye_ycy_position', None),                      # 检查右鱼眼YCY位置
            ('turn_left', {'count': 9}),                       # 左转9步
            # ('check_right_fisheye_ycy_position', None),                      # 检查右鱼眼YCY位置
            ('backward', {'count': 9}),                        # 后退9步
            # ('check_rgb_distance', None),                      # 检查RGB距离
            ('lie_down', None)                                 # 趴下
        ]
        self.movement_step = 0
        self.execute_next_movement()
    
    def start_a2_to_start_sequence(self):
        """a2_to_start状态：站起，前9,检查-右鱼眼-ycy,右21,检查-左鱼眼-dy,后23,检查-右鱼眼-ycy，左转9，检查-右鱼眼-ycy，后9,检查-rgb-dy，趴下"""
        print("[状态机] 开始a2_to_start运动序列（返回充电站）")
        self.movement_sequence = [
            ('stand', None),                                   # 站立
            ('forward', {'count': 9}),                         # 前进9步
            # ('check_right_fisheye_ycy_position', None),                      # 检查右鱼眼YCY位置
            ('right', {'count': 21}),                          # 右移21步
            # ('check_left_fisheye_distance', None),                      # 检查左鱼眼距离
            ('backward', {'count': 23}),                       # 后退23步
            # ('check_right_fisheye_ycy_position', None),                      # 检查右鱼眼YCY位置
            ('turn_left', {'count': 9}),                       # 左转9步
            # ('check_right_fisheye_ycy_position', None),                      # 检查右鱼眼YCY位置
            ('backward', {'count': 9}),                        # 后退9步
            # ('check_rgb_distance', None),                      # 检查RGB距离
            ('lie_down', None)                                 # 趴下
        ]
        self.movement_step = 0
        self.execute_next_movement()
    
    def start_qrb_detection(self):
        """开始在qrb点进行二维码识别"""
        print("[状态机] 到达二维码B点，开始二维码识别")
        self.qr_detection_start_time = time.time()
        # 停止运动，准备识别
        self.stop_motion_timer()
    
    def execute_next_movement(self):
        """执行下一个运动"""
        if self.movement_step < len(self.movement_sequence):
            # 处理新的元组格式: (运动类型, 自定义参数)
            movement_item = self.movement_sequence[self.movement_step]
            if isinstance(movement_item, tuple):
                movement_type, custom_params = movement_item
            else:
                # 兼容旧格式
                movement_type = movement_item
                custom_params = None
                
            print(f"[状态机] 执行运动类型: {movement_type} (步骤 {self.movement_step + 1}/{len(self.movement_sequence)})")
            if custom_params:
                print(f"[状态机] 使用自定义参数: {custom_params}")
            
            self.send_motion_command(movement_type, custom_params)
        else:
            # 运动序列完成，转到下一个状态
            self.on_movement_sequence_completed()
    
    def on_movement_sequence_completed(self):
        """运动序列完成处理"""
        if self.current_state == State.START_TO_QRA:
            print("[状态机] start_to_qra运动序列完成，进入in_qr_a状态")
            self.transition_to_state(State.IN_QR_A)
        elif self.current_state == State.QRA_TO_A1:
            print("[状态机] qra_to_a1运动序列完成，进入in_a1状态")
            self.transition_to_state(State.IN_A1)
        elif self.current_state == State.QRA_TO_A2:
            print("[状态机] qra_to_a2运动序列完成，进入in_a2状态")
            self.transition_to_state(State.IN_A2)
        elif self.current_state == State.IN_A1:
            print("[状态机] in_a1运动序列完成，处理状态转换")
            self._handle_a1_sequence_complete()
        elif self.current_state == State.IN_A2:
            print("[状态机] in_a2运动序列完成，处理状态转换")
            self._handle_a2_sequence_complete()
        elif self.current_state == State.A1_TO_S1:
            print("[状态机] a1_to_s1运动序列完成，进入in_s1状态")
            self.transition_to_state(State.IN_S1)
        elif self.current_state == State.A2_TO_S1:
            print("[状态机] a2_to_s1运动序列完成，进入in_s1状态")
            self.transition_to_state(State.IN_S1)
        # QRB到B区状态
        elif self.current_state == State.QRB_TO_B1:
            print("[状态机] qrb_to_b1运动序列完成，进入in_b1状态")
            self.b_zone_task_count = 0  # 重置B区任务计数器
            self.transition_to_state(State.IN_B1)
        elif self.current_state == State.QRB_TO_B2:
            print("[状态机] qrb_to_b2运动序列完成，进入in_b2状态")
            self.b_zone_task_count = 0  # 重置B区任务计数器
            self.transition_to_state(State.IN_B2)
        elif self.current_state == State.IN_B1:
            self.b_zone_task_count += 1
            print(f"[状态机] in_b1运动序列完成，B区任务计数: {self.b_zone_task_count}/{self.max_b_zone_tasks}")
            if self.b_zone_task_count >= self.max_b_zone_tasks:
                print("[状态机] B区任务完成，开始返程")
                # 根据绿色箭头方向决定返程路径（相反方向）
                if self.green_arrow_result == "left":
                    # 来时向左，返程向右
                    print("[状态机] 来时向左，从B1返程到R1")
                    self.transition_to_state(State.B1_TO_R1)
                elif self.green_arrow_result == "right":
                    # 来时向右，返程向左
                    print("[状态机] 来时向右，从B1返程到L1")
                    self.transition_to_state(State.B1_TO_L1)
                else:
                    # 如果没有绿色箭头记录，使用默认路径
                    print(f"[状态机] 没有绿色箭头记录，使用默认返程路径，从B1返程到L1")
                    self.transition_to_state(State.B1_TO_L1)
            else:
                print("[状态机] 继续B区任务，进入b1_to_b2状态")
                self.transition_to_state(State.B1_TO_B2)
        elif self.current_state == State.IN_B2:
            self.b_zone_task_count += 1
            print(f"[状态机] in_b2运动序列完成，B区任务计数: {self.b_zone_task_count}/{self.max_b_zone_tasks}")
            if self.b_zone_task_count >= self.max_b_zone_tasks:
                print("[状态机] B区任务完成，开始返程")
                # 根据绿色箭头方向决定返程路径（相反方向）
                if self.green_arrow_result == "left":
                    # 来时向左，返程向右
                    print("[状态机] 来时向左，从B2返程到R1")
                    self.transition_to_state(State.B2_TO_R1)
                elif self.green_arrow_result == "right":
                    # 来时向右，返程向左
                    print("[状态机] 来时向右，从B2返程到L1")
                    self.transition_to_state(State.B2_TO_L1)
                else:
                    # 如果没有绿色箭头记录，使用默认路径
                    print(f"[状态机] 没有绿色箭头记录，使用默认返程路径，从B2返程到L1")
                    self.transition_to_state(State.B2_TO_L1)
            else:
                print("[状态机] 继续B区任务，进入b2_to_b1状态")
                self.transition_to_state(State.B2_TO_B1)
        elif self.current_state == State.B1_TO_B2:
            print("[状态机] b1_to_b2运动序列完成，进入in_b2状态")
            self.transition_to_state(State.IN_B2)
        elif self.current_state == State.B2_TO_B1:
            print("[状态机] b2_to_b1运动序列完成，进入in_b1状态")
            self.transition_to_state(State.IN_B1)
        # B区返程状态
        elif self.current_state == State.B1_TO_L1:
            print("[状态机] b1_to_l1运动序列完成，进入in_l1状态")
            self.transition_to_state(State.IN_L1)
        elif self.current_state == State.B1_TO_R1:
            print("[状态机] b1_to_r1运动序列完成，进入in_r1状态")
            self.transition_to_state(State.IN_R1)
        elif self.current_state == State.B2_TO_L1:
            print("[状态机] b2_to_l1运动序列完成，进入in_l1状态")
            self.transition_to_state(State.IN_L1)
        elif self.current_state == State.B2_TO_R1:
            print("[状态机] b2_to_r1运动序列完成，进入in_r1状态")
            self.transition_to_state(State.IN_R1)
        # L1和R1中转点状态
        elif self.current_state == State.IN_L1:
            print("[状态机] in_l1运动序列完成，处理状态转换")
            self._handle_l1_sequence_complete()
        elif self.current_state == State.L1_TO_S2:
            print("[状态机] l1_to_s2运动序列完成，进入in_s2_r状态")
            self.transition_to_state(State.IN_S2_R)
        elif self.current_state == State.R1_TO_S2:
            print("[状态机] r1_to_s2运动序列完成，进入in_s2_r状态")
            self.transition_to_state(State.IN_S2_R)
        # 返程到A区和充电站
        elif self.current_state == State.S1_TO_A1:
            print("[状态机] s1_to_a1运动序列完成，进入in_a1状态")
            self.transition_to_state(State.IN_A1)
        elif self.current_state == State.S1_TO_A2:
            print("[状态机] s1_to_a2运动序列完成，进入in_a2状态")
            self.transition_to_state(State.IN_A2)
        elif self.current_state == State.A1_TO_START:
            print("[状态机] a1_to_start运动序列完成，返回充电站完成")
            self.transition_to_state(State.END)
        elif self.current_state == State.A2_TO_START:
            print("[状态机] a2_to_start运动序列完成，返回充电站完成")
            self.transition_to_state(State.END)
        else:
            print(f"[状态机] 未处理的运动序列完成状态: {self.current_state}")
            self.transition_to_state(State.ERROR)
    
    def start_qr_detection(self):
        """开始在qr_a点进行二维码识别，如果20s没有结果则前进重试"""
        print("[状态机] 到达二维码A点，开始二维码识别")
        self.qr_detection_start_time = time.time()
        self.z1_steps = 0  # 重置前进步数计数器
        self.qr_retry_count = 0  # 重置重试计数器
        # 停止运动，准备识别
        self.stop_motion_timer()
        # 启动20秒定时器检查QR识别结果 - 使用状态绑定定时器
        self.create_state_timer('qr_detection_timeout', 20.0, self.check_qr_detection_timeout, State.IN_QR_A)
    
    def check_qr_detection_timeout(self):
        """检查QR识别20s超时，如果没有结果则前进2步重试"""
        if self.current_state != State.IN_QR_A:
            return  # 如果不在QR识别状态，直接返回
            
        if self.qr_result is not None:
            return  # 如果已经有识别结果，直接返回
            
        # 检查是否已经达到最大重试次数
        if self.qr_retry_count >= self.max_qr_retries:
            print(f"[状态机] QR识别已重试{self.qr_retry_count}次，使用默认值: {self.qr_a_default_value}")
            self.qr_result = self.qr_a_default_value
            self.initial_qr_result = self.qr_a_default_value
            
            # 停止二维码识别节点
            if 'qr_detector_node' in self.managed_nodes:
                self.stop_node('qr_detector_node')
            
            self.process_qr_a_result()
            return
            
        # 前进2步后重试
        self.qr_retry_count += 1
        print(f"[状态机] QR识别20秒内无结果，前进2步重试 (第{self.qr_retry_count}/{self.max_qr_retries}次)")
        self.z1_steps += 2
        
        # 直接执行前进2步（不使用运动序列）
        self._execute_motion_command('forward', {'count': 2})
        
        # 使用状态绑定定时器避免递归问题
        def delayed_retry():
            # 创建下一次重试的定时器
            self.create_state_timer('qr_retry', 20.0, self.check_qr_detection_timeout, State.IN_QR_A)
        
        # 等待前进完成后重新启动20s定时器
        self.create_state_timer('qr_retry_delay', 0.6, delayed_retry, State.IN_QR_A)
    
    def process_qr_a_result(self):
        """处理A点二维码识别结果"""
        if self.qr_result == "A-1":
            print(f"[状态机] 识别到二维码: {self.qr_result}，转到qra_to_a1状态")
            self.transition_to_state(State.QRA_TO_A1)
        elif self.qr_result == "A-2":
            print(f"[状态机] 识别到二维码: {self.qr_result}，转到qra_to_a2状态")
            self.transition_to_state(State.QRA_TO_A2)
        else:
            print(f"[状态机] A点未知二维码: {self.qr_result}")
            self.transition_to_state(State.ERROR)
    
    def process_qr_b_result(self):
        """处理B点二维码识别结果"""
        if self.qrb_result == "B-1":
            print(f"[状态机] 识别到二维码: {self.qrb_result}，转到qrb_to_b1状态")
            self.transition_to_state(State.QRB_TO_B1)
        elif self.qrb_result == "B-2":
            print(f"[状态机] 识别到二维码: {self.qrb_result}，转到qrb_to_b2状态")
            self.transition_to_state(State.QRB_TO_B2)
        else:
            print(f"[状态机] B点未知二维码: {self.qrb_result}")
            self.transition_to_state(State.ERROR)
    
    def on_task_completed(self):
        """任务完成处理"""
        self.task_completed = True
        print("[状态机] 任务完成！清理，关闭")
        
        # 停止运动定时器
        self.stop_motion_timer()
        
        # 清理所有节点
        self.cleanup_all_nodes()
    
    def on_error_state(self):
        """错误状态处理"""
        print("[状态机] 进入错误状态，急停运动")
        
        # 停止运动定时器
        self.stop_motion_timer()
        
        # 急停
        self.send_motion_command('lie_down')
        
        # 清理所有节点
        self.cleanup_all_nodes()
    
    def send_motion_command(self, movement_type, custom_params=None):
        """发送运动指令 - 支持持续发布（类似ros2 topic pub -r 频率 -t 次数）和服务调用"""
        try:
            print(f"[调试] send_motion_command 调用 - movement_type: {movement_type}, custom_params: {custom_params}")
            
            # 处理特殊的检查命令
            if movement_type == 'check_rgb_position':
                print("[状态机] 开始RGB相机位置检查")
                print(f"[调试] 原始custom_params: {custom_params}, 类型: {type(custom_params)}")
                self.position_check_type = 'rgb'
                config_name = custom_params if isinstance(custom_params, str) else (custom_params.get('config_name') if custom_params else None)
                print(f"[调试] 解析出的config_name: {config_name}")
                self.check_rgb_position(config_name)
                return
            elif movement_type == 'check_rgb_distance':
                print("[状态机] 开始RGB相机距离检查")
                self.position_check_type = 'rgb'  # 修复：设置position_check_type
                config_name = custom_params if isinstance(custom_params, str) else (custom_params.get('config_name') if custom_params else None)
                self.check_rgb_distance(config_name)
                return
            elif movement_type == 'check_right_fisheye_distance':
                print("[状态机] 开始右鱼眼相机距离检查")
                self.position_check_type = 'right_fisheye'  # 修复：设置position_check_type
                config_name = custom_params if isinstance(custom_params, str) else (custom_params.get('config_name') if custom_params else None)
                self.check_right_fisheye_distance(config_name)
                return
            elif movement_type == 'check_left_fisheye_distance':
                print("[状态机] 开始左鱼眼相机距离检查")
                self.position_check_type = 'left_fisheye'  # 修复：设置position_check_type
                config_name = custom_params if isinstance(custom_params, str) else (custom_params.get('config_name') if custom_params else None)
                self.check_left_fisheye_distance(config_name)
                return
            elif movement_type == 'check_left_fisheye_ycy_position':
                print("[状态机] 开始左鱼眼YCY位置检查")
                self.position_check_type = 'left_fisheye'  # 设置具体的鱼眼类型
                config_name = custom_params if isinstance(custom_params, str) else (custom_params.get('config_name') if custom_params else None)
                self.check_left_fisheye_ycy_position(config_name)
                return
            elif movement_type == 'check_right_fisheye_ycy_position':
                print("[状态机] 开始右鱼眼YCY位置检查")
                self.position_check_type = 'right_fisheye'  # 设置具体的鱼眼类型
                if isinstance(custom_params, str):
                    config_name = custom_params
                elif isinstance(custom_params, dict):
                    config_name = custom_params.get('config_name')
                else:
                    config_name = None
                self.check_right_fisheye_ycy_position(config_name)
                return
            elif movement_type == 'forward_until_red_barrier':
                print("[状态机] 开始前进直到检测到红色限高杆")
                self.forward_until_red_barrier()
                return
            elif movement_type == 'forward_until_yellow_marker':
                print("[状态机] 开始前进直到检测到黄色标志物")
                self.forward_until_yellow_marker()
                return
            
            # 检查是否是lie_down动作，如果是则先等待5秒
            if movement_type == 'lie_down':
                print("[状态机] 准备执行lie_down，先等待5秒让机器人稳定...")
                self.lie_down_wait_timer = self.create_state_timer('lie_down_wait', 5.0, self._execute_lie_down_after_wait_callback, self.current_state)
                self.pending_lie_down_params = custom_params
                return
            
            # 执行其他运动指令
            self._execute_motion_command(movement_type, custom_params)
            
        except Exception as e:
            self.get_logger().error(f'发送运动指令时出错: {str(e)}')
    
    def _execute_lie_down_after_wait_callback(self):
        """等待后执行lie_down的回调"""
        print("[状态机] 等待完成，现在执行lie_down")
        # 定时器会通过状态绑定自动清理
        self.lie_down_wait_timer = None
        
        # 检查是否已经有lie_down在执行中
        if hasattr(self, 'lie_down_final_timer') and self.lie_down_final_timer is not None:
            print("[状态机] lie_down已在执行或等待中，跳过重复执行")
            return
        
        # 执行lie_down
        self._execute_motion_command('lie_down', self.pending_lie_down_params)
        self.pending_lie_down_params = None
    
    def _execute_motion_command(self, movement_type, custom_params=None):
        """实际执行运动指令"""
        try:
            # 停止之前的运动定时器
            if self.motion_timer is not None:
                self.motion_timer.cancel()
                self.motion_timer = None
            
            # 检查运动类型是否存在
            if movement_type not in self.basic_movements:
                self.get_logger().error(f'未知的运动类型: {movement_type}')
                return
            
            config = self.basic_movements[movement_type]
            # 复制配置以避免修改原始配置
            config = config.copy()
            
            # 如果有自定义参数，覆盖默认配置
            if custom_params:
                for key, value in custom_params.items():
                    if key in config:
                        config[key] = value
                        self.get_logger().info(f'使用自定义参数 {key}={value}')
            
            self.current_motion_config = config
            self.motion_count = 0
            
            # 检查是否为服务调用类型
            if config.get('is_service_call', False):
                print(f"[状态机] 开始服务调用运动: {movement_type}")
                self._call_motion_service(config)
                return
            
            # 获取发布频率和次数
            frequency = config['frequency']
            total_count = config['count']
            
            # 参数验证
            if frequency <= 0:
                self.get_logger().error(f'无效的频率: {frequency}Hz')
                return
            if total_count <= 0:
                self.get_logger().error(f'无效的次数: {total_count}')
                return
            
            # MotionServoCmd接口约束验证
            vel_des = config['vel_des']
            if abs(vel_des[1]) > 1.5:  # y方向速度限制
                self.get_logger().error(f'y方向速度超限: {vel_des[1]} > 1.5 m/s')
                return
            if abs(vel_des[2]) > 2.0:  # yaw角速度限制
                self.get_logger().error(f'yaw角速度超限: {vel_des[2]} > 2.0 rad/s')
                return
            
            print(f"[状态机] 开始运动: {movement_type}")
            print(f"[状态机] 参数: 频率={frequency}Hz, 总次数={total_count}, 类似: ros2 topic pub -r {frequency} -t {total_count}")
            
            # 立即发布第一次
            self._publish_motion_message(config)
            self.motion_count += 1
            print(f"[状态机] 发送运动指令: {movement_type} (第 {self.motion_count}/{total_count} 次)")
            
            # 如果需要发送多次，启动定时器 - 使用状态绑定确保状态切换时自动清理
            if total_count > 1 and frequency > 0:
                timer_period = 1.0 / frequency  # 计算定时器周期
                self.motion_timer = self.create_state_timer('motion_control', timer_period, self._motion_timer_callback, self.current_state)
                self.get_logger().info(f'启动运动控制定时器: 周期={timer_period:.3f}s')
            elif total_count == 1:
                # 单次发送完成，进入下一个运动
                print(f"[状态机] 运动 {movement_type} 单次发送完成")
                self.on_single_movement_completed()
            
        except Exception as e:
            self.get_logger().error(f'发送运动指令时出错: {str(e)}')
    
    def _publish_motion_message(self, config):
        """发布运动消息到ROS2话题"""
        try:
            # 创建MotionServoCmd消息
            msg = MotionServoCmd()
            msg.motion_id = config['motion_id']
            msg.cmd_type = config['cmd_type']
            msg.cmd_source = config['cmd_source']
            msg.value = config['value']
            msg.vel_des = [float(x) for x in config['vel_des']]
            msg.rpy_des = [float(x) for x in config['rpy_des']]
            msg.pos_des = [float(x) for x in config['pos_des']]
            msg.acc_des = [float(x) for x in config['acc_des']]
            msg.ctrl_point = [float(x) for x in config['ctrl_point']]
            msg.foot_pose = [float(x) for x in config['foot_pose']]
            msg.step_height = [float(x) for x in config['step_height']]

            # 发布指令到话题
            self.motion_cmd_publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f'发布运动消息时出错: {str(e)}')
    
    def _call_motion_service(self, config):
        """调用运动控制服务"""
        try:
            # 创建服务请求
            request = MotionResultCmd.Request()
            request.motion_id = config['motion_id']
            request.cmd_source = config['cmd_source']
            request.vel_des = config['vel_des']
            request.rpy_des = config['rpy_des']
            request.pos_des = config['pos_des']
            request.acc_des = config['acc_des']
            request.ctrl_point = config['ctrl_point']
            request.foot_pose = config['foot_pose']
            request.step_height = config['step_height']
            request.duration = config['duration']
            
            print(f"[状态机] 调用运动服务: motion_id={config['motion_id']}, duration={config['duration']}s")
            
            # 异步调用服务
            future = self.motion_service_client.call_async(request)
            future.add_done_callback(self._service_callback)
            
        except Exception as e:
            self.get_logger().error(f'调用运动服务时出错: {str(e)}')
            # 服务调用失败，直接进入下一个运动
            self.on_single_movement_completed()
    
    def _service_callback(self, future):
        """服务调用回调"""
        try:
            response = future.result()
            print(f"[状态机] 运动服务调用完成: {response}")
            
            # 检查是否是lie_down动作
            if (hasattr(self, 'current_motion_config') and 
                self.current_motion_config and 
                self.current_motion_config.get('motion_id') == 101):
                print("[状态机] lie_down动作完成，等待10秒...")
                # 创建10秒延时定时器 - 使用状态绑定定时器
                self.lie_down_final_timer = self.create_state_timer('lie_down_final', 10.0, self.on_lie_down_wait_completed_callback, self.current_state)
                return  # 重要：阻止继续执行其他逻辑
            else:
                # 其他服务调用完成，直接进入下一个运动
                self.on_single_movement_completed()
                
        except Exception as e:
            self.get_logger().error(f'运动服务调用失败: {str(e)}')
            # 服务调用失败，直接进入下一个运动
            self.on_single_movement_completed()
    
    def on_lie_down_wait_completed_callback(self):
        """lie_down最终等待完成回调"""
        # 定时器会通过状态绑定自动清理
        self.lie_down_final_timer = None
        
        print("[状态机] lie_down等待10秒完成，继续执行")
        # 检查是否是运动序列中的最后一个动作
        if hasattr(self, 'movement_step') and hasattr(self, 'movement_sequence'):
            if self.movement_step + 1 >= len(self.movement_sequence):
                # 这是运动序列的最后一个动作，直接完成序列
                self.movement_step += 1  # 确保step正确递增
                self.on_movement_sequence_completed()
            else:
                # 不是最后一个动作，继续执行下一个
                self.on_single_movement_completed()
        else:
            # 没有运动序列信息，使用原来的逻辑
            self.on_single_movement_completed()
        
    def _motion_timer_callback(self):
        """运动控制定时器回调 - 按频率持续发布指令"""
        if self.current_motion_config is not None:
            total_count = self.current_motion_config['count']
            
            # 检查是否已达到指定次数
            if self.motion_count < total_count:
                self._publish_motion_message(self.current_motion_config)
                self.motion_count += 1
                
                # 获取当前运动类型名称（用于日志）
                movement_type = "unknown"
                if hasattr(self, 'movement_sequence') and self.movement_step < len(self.movement_sequence):
                    movement_item = self.movement_sequence[self.movement_step]
                    if isinstance(movement_item, tuple):
                        movement_type = movement_item[0]
                    else:
                        movement_type = movement_item
                
                print(f"[状态机] 发送运动指令: {movement_type} (第 {self.motion_count}/{total_count} 次)")
                
                # 如果这是最后一次发送
                if self.motion_count >= total_count:
                    print(f"[状态机] 运动 {movement_type} 完成 (共发送 {total_count} 次)")
                    # 运动完成
                    self.stop_motion_timer()
                    self.on_single_movement_completed()
            else:
                # 运动完成（防止意外情况）
                self.stop_motion_timer()
                self.on_single_movement_completed()
    
    def on_single_movement_completed(self):
        """单个运动完成"""
        import time
        self.last_movement_completion_time = time.time()  # 记录运动完成时间
        
        if self.movement_step < len(self.movement_sequence):
            movement_item = self.movement_sequence[self.movement_step]
            if isinstance(movement_item, tuple):
                movement_type = movement_item[0]
            else:
                movement_type = movement_item
        else:
            movement_type = "unknown"
        print(f"[状态机] 运动 {movement_type} 完成")
        
        # 特殊状态处理：QR识别状态下的运动完成
        if self.current_state in [State.IN_QR_A, State.IN_QRB]:
            print(f"[状态机] 在{self.current_state.name}状态下运动完成，等待识别结果")
            return
        
        # 如果有待处理的位置重新检查，则执行重新检查
        if hasattr(self, 'pending_position_recheck') and self.pending_position_recheck:
            print("[状态机] 位置修正完成，重新进行位置检查")
            self.pending_position_recheck = False
            
            # 使用保存的检查上下文重新执行检查
            if hasattr(self, 'position_check_context') and self.position_check_context:
                context = self.position_check_context
                check_type = context.get('check_type')
                config_name = context.get('config_name')
                
                print(f"[调试] 重新执行检查: {check_type}, 配置: {config_name}")
                
                if check_type == 'rgb_position':
                    self.check_rgb_position(config_name)
                elif check_type == 'rgb_distance':
                    self.check_rgb_distance(config_name)
                elif check_type == 'right_fisheye_ycy_position':
                    self.check_right_fisheye_ycy_position(config_name)
                elif check_type == 'left_fisheye_ycy_position':
                    self.check_left_fisheye_ycy_position(config_name)
                elif check_type == 'right_fisheye_distance':
                    self.check_right_fisheye_distance(config_name)
                elif check_type == 'left_fisheye_distance':
                    self.check_left_fisheye_distance(config_name)
                else:
                    print(f"[状态机] 未知的检查类型: {check_type}")
                    print("[调试] 可能是检查上下文被意外重置，尝试进入下一阶段")
                    self.proceed_to_next_stage()
            else:
                print(f"[状态机] 没有保存的检查上下文，进入下一阶段")
                self.proceed_to_next_stage()
        else:
            # 进入下一个运动
            self.movement_step += 1
            self.execute_next_movement()
    
    def stop_motion_timer(self):
        """停止运动控制定时器"""
        if self.motion_timer is not None:
            self.motion_timer.cancel()
            self.motion_timer = None
            self.current_motion_config = None
            self.motion_count = 0
            self.get_logger().info('运动控制定时器已停止')
    
    def reset_state_machine(self):
        """重置状态机"""
        self.current_state = State.START
        self.previous_state = State.START
        self.qr_result = None
        self.initial_qr_result = None
        self.qrb_result = None
        self.green_arrow_result = None
        self.qr_detection_start_time = None
        self.green_arrow_detection_start_time = None
        self.movement_start_time = None
        self.task_completed = False
        self.movement_step = 0
        self.movement_sequence = []
        self.motion_count = 0
        self.r1_next_state = None  # 重置R1状态的下一状态
        self.l1_next_state = None  # 重置L1状态的下一状态
        self.a1_next_state = None  # 重置A1状态的下一状态
        self.a2_next_state = None  # 重置A2状态的下一状态
        
        # 重置QR识别重试计数器
        self.qr_retry_count = 0
        
        # 重置B区任务相关变量
        self.b_zone_task_count = 0
        
        # 停止运动定时器
        self.stop_motion_timer()
        
        print("[状态机] 已重置，可以重新开始")
    
    def check_timeouts(self):
        """检查超时"""
        current_time = time.time()
        
        # 检查A点二维码识别总超时（保留原有的总超时逻辑）
        if (self.current_state == State.IN_QR_A and 
            self.qr_detection_start_time and 
            current_time - self.qr_detection_start_time > self.qr_detection_timeout):
            print(f"[状态机] A点二维码识别总超时，使用默认值: {self.qr_a_default_value}，关闭识别节点")
            self.qr_result = self.qr_a_default_value
            self.initial_qr_result = self.qr_a_default_value
            
            # 停止二维码识别节点
            if 'qr_detector_node' in self.managed_nodes:
                self.stop_node('qr_detector_node')
            
            self.process_qr_a_result()
            
        # 检查B点二维码识别超时
        if (self.current_state == State.IN_QRB and 
            self.qr_detection_start_time and 
            current_time - self.qr_detection_start_time > self.qr_detection_timeout):
            print(f"[状态机] B点二维码识别超时，使用默认值: {self.qr_b_default_value}，关闭识别节点")
            self.qrb_result = self.qr_b_default_value
            
            # 停止二维码识别节点
            if 'qr_detector_node' in self.managed_nodes:
                self.stop_node('qr_detector_node')
            
            self.process_qr_b_result()
            
        # 检查绿色箭头识别超时
        if (self.current_state == State.IN_S2 and 
            self.green_arrow_detection_start_time and 
            self.green_arrow_result is None and  # 只有在没有识别结果时才处理超时
            current_time - self.green_arrow_detection_start_time > self.green_arrow_detection_timeout):
            print(f"[状态机] 绿色箭头识别超时，使用默认值: {self.green_arrow_default_value}，关闭识别节点")
            self.green_arrow_result = self.green_arrow_default_value
            
            # 重置绿色箭头识别开始时间，防止重复触发
            self.green_arrow_detection_start_time = None
            
            # 停止绿色箭头识别节点
            if 'green_arrow_detector' in self.managed_nodes:
                self.stop_node('green_arrow_detector')
            
            self._process_green_arrow_result(self.green_arrow_default_value)
    
    def state_machine_loop(self):
        """状态机主循环"""
        # 检查超时
        self.check_timeouts()
        
        # 发布当前状态信息
        state_msg = String()
        state_msg.data = f"CurrentState:{self.current_state.value},QRResult:{self.qr_result or 'None'},TaskCompleted:{self.task_completed}"
        self.state_publisher.publish(state_msg)

    def destroy_node(self):
        """销毁节点时清理资源"""
        print("[状态机] 正在关闭，清理所有资源...")
        
        # 停止运动控制定时器
        self.stop_motion_timer()
        
        # 清理所有状态绑定定时器和全局定时器
        self.cleanup_all_timers()
        
        # 清理所有管理的节点
        self.cleanup_all_nodes()
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    state_machine_node = StateMachineNode()
    
    try:
        rclpy.spin(state_machine_node)
    except KeyboardInterrupt:
        pass
    finally:
        state_machine_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 