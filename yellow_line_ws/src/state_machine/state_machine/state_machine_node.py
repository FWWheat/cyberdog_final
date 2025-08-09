#!/usr/bin/env python3

"""
状态机节点 - 机器人运动控制状态机

功能描述：
    - 控制机器人从起点运动到qr_a点进行二维码识别
    - 根据识别结果运动到a1点或a2点
    - 支持持续发布运动控制指令（类似ros2 topic pub -r 频率 -t 次数）
    
运动控制特性：
    - 每种基础运动类型都有独立的频率(Hz)和次数配置
    - 类似ROS2命令: ros2 topic pub -r <频率> -t <次数> <话题> <消息>
    - 支持精确的时序控制，确保运动指令按预设频率和次数发送
    - 自动管理运动序列，完成一个运动后自动进入下一个运动或状态

启动方法：
    1. 构建工作空间：
       cd ~/yellow_line_ws
       colcon build --packages-select state_machine
       source install/setup.bash

    2. 启动节点：
       ros2 run state_machine state_machine_node

使用方法：
    1. 启动任务：
       ros2 topic pub -1 /state_machine/start_command std_msgs/msg/String "data: 'START'"

    2. 停止任务：
       ros2 topic pub -1 /state_machine/start_command std_msgs/msg/String "data: 'STOP'"

    3. 重置状态机：
       ros2 topic pub -1 /state_machine/start_command std_msgs/msg/String "data: 'RESET'"

    4. 监控状态：
       ros2 topic echo /state_machine/state_info

    5. 查看运动控制指令：
       ros2 topic echo /mi_desktop_48_b0_2d_7b_03_d0/motion_servo_cmd


状态流程：
    start -> start_to_qra -> in_qr_a -> qra_to_a1/qra_to_a2 -> in_a1/in_a2 -> a1_to_s1/a2_to_s1 -> end

运动类型说明：
    1. forward:    向前运动 - 5Hz频率发送15次  vel_des=[0.5, 0.0, 0.0]
    2. backward:   向后运动 - 5Hz频率发送15次  vel_des=[-0.5, 0.0, 0.0]
    3. right:      向右运动 - 5Hz频率发送10次  vel_des=[0.0, -0.3, 0.0]
    4. left:       向左运动 - 5Hz频率发送10次  vel_des=[0.0, 0.3, 0.0]
    5. turn_right: 右转90度 - 5Hz频率发送9次   vel_des=[0.0, 0.0, -1.0]
    6. turn_left:  左转90度 - 5Hz频率发送9次   vel_des=[0.0, 0.0, 1.0]
    7. emergency_stop: 急停 - 服务调用  motion_id=0
    8. stand:      站立 - 服务调用  motion_id=111
    9. lie_down:   高阻尼趴下 - 服务调用  motion_id=101

MotionServoCmd接口规范：
    - 接口名字: "motion_servo_cmd"
    - 消息文件: protocol/msg/MotionServoCmd.msg
    - motion_id: 303 (机器人运控姿态)
    - cmd_type: 1 (Data帧), 2 (End帧)
    - cmd_source: 4 (Algo算法来源)
    - value: 0 (内八步态), 2 (垂直步态)
    - vel_des: [x, y(≤1.5), yaw(≤2.0)] 速度约束 m/s
    - step_height: [0.05, 0.05] 抬腿高度，默认0.05m
    - 其他字段: rpy_des, pos_des, acc_des, ctrl_point, foot_pose 当前暂不开放

示例命令：
    ros2 topic pub -r 5 -t 15 /mi_desktop_48_b0_2d_7b_03_d0/motion_servo_cmd protocol/msg/MotionServoCmd "{ 
        motion_id: 303, cmd_type: 1, cmd_source: 4, value: 0, 
        vel_des: [0.0, 0.3, 0.0], rpy_des: [0.0, 0.0, 0.0], 
        pos_des: [0.0, 0.0, 0.0], acc_des: [0.0, 0.0, 0.0], 
        ctrl_point: [0.0, 0.0, 0.0], foot_pose: [0.0, 0.0, 0.0], 
        step_height: [0.05, 0.05] }"

"""

from itertools import count
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
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
    END = 9              # 任务结束
    ERROR = 10           # 错误状态

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')
        
        # 状态机状态
        self.current_state = State.START
        self.previous_state = State.START

        # 任务完成标志
        self.task_completed = False
        
        # 二维码识别结果
        self.qr_result = None
        self.qr_detection_timeout = 10.0  # 二维码识别超时时间（秒）
        self.qr_detection_start_time = None
        
        # 图像处理相关
        self.bridge = CvBridge()
        # 使用统一的黄线检测器处理所有图像
        self.yellow_line_detector = YellowLineDetector()
        self.fisheye_image = None
        self.rgb_image = None
        self.position_check_pending = False
        self.position_check_type = None  # 'fisheye' or 'rgb'
        
        # 运动控制参数
        self.movement_timeout = 30.0  # 运动超时时间（秒）
        self.movement_start_time = None
        self.movement_step = 0  # 运动步骤计数器
        self.movement_total_steps = 0  # 总步骤数
        self.movement_sequence = []  # 运动序列
        
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
        
        # 创建发布者和订阅者
        self.setup_communication()
        
        # 创建定时器
        self.create_timer(0.1, self.state_machine_loop)  # 10Hz状态机循环
        
        print("[状态机] 节点已启动，等待命令...")

        
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
        
        # 鱼眼相机图像订阅者 (1000x800 RG10)
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
        
    def start_command_callback(self, msg):
        """处理启动命令"""
        if msg.data == "START" and self.current_state == State.START and not self.task_completed:
            print("[状态机] 收到启动命令")
            self.transition_to_state(State.START_TO_QRA)
        elif msg.data == "START" and self.task_completed:
            print("[状态机] 任务已完成，请发送RESET重置")
        elif msg.data == "STOP":
            print("[状态机] 收到停止命令")
            # 停止运动定时器
            self.stop_motion_timer()
            self.transition_to_state(State.START)
        elif msg.data == "RESET":
            print("[状态机] 收到重置命令")
            self.reset_state_machine()
        else:
            print("[状态机] 收到未知命令")
    
    def qr_info_callback(self, msg):
        """处理二维码识别结果"""
        try:
            # 解析二维码信息
            # 格式: "QR:A-1,Distance:1.5" 或 "QR:A-2,Distance:1.2"
            if msg.data.startswith("QR:"):
                parts = msg.data.split(",")
                qr_code = parts[0].split(":")[1]
                distance_str = parts[1].split(":")[1]
                distance = float(distance_str)
                
                self.get_logger().info(f'收到二维码识别结果: {qr_code}, 距离: {distance}m')
                
                if self.current_state == State.IN_QR_A:
                    self.qr_result = qr_code
                    self.process_qr_result()
                    
        except Exception as e:
            self.get_logger().error(f'处理二维码信息时出错: {str(e)}')
    
    def fisheye_image_callback(self, msg):
        """鱼眼相机图像回调"""
        try:
            self.fisheye_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 如果正在等待鱼眼图像位置检查
            if self.position_check_pending and self.position_check_type == 'fisheye':
                self.check_fisheye_position()
                
        except Exception as e:
            self.get_logger().error(f'处理鱼眼相机图像时出错: {str(e)}')
    
    def rgb_image_callback(self, msg):
        """RGB相机图像回调"""
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 如果正在等待RGB图像位置检查
            if self.position_check_pending and self.position_check_type == 'rgb':
                self.check_rgb_position()
                
        except Exception as e:
            self.get_logger().error(f'处理RGB相机图像时出错: {str(e)}')
    
    def check_fisheye_position(self):
        """检查鱼眼相机中的位置"""
        if self.fisheye_image is None:
            print("[状态机] 鱼眼图像未就绪，等待...")
            return
        
        # 保存关键鱼眼图像用于调试
        import cv2
        import time
        timestamp = int(time.time())
        debug_image_path = f"/tmp/fisheye_debug_{timestamp}.jpg"
        cv2.imwrite(debug_image_path, self.fisheye_image)
        print(f"[状态机] 已保存鱼眼调试图像: {debug_image_path}")
        
        position = self.yellow_line_detector.detect_position(self.fisheye_image)
        print(f"[状态机] 鱼眼相机检测位置: {position}")
        
        self.position_check_pending = False
        
        if position == 'left':
            print("[状态机] 靠近左线，向后走一步")
            self.execute_position_correction('backward',{'count': 1})
        elif position == 'right':
            print("[状态机] 靠近右线，向前走一步")
            self.execute_position_correction('forward',{'count': 1})
        else:
            print("[状态机] 位置正确，进入下一阶段")
            self.proceed_to_next_stage()
    
    def check_rgb_position(self):
        """检查RGB相机中的位置"""
        if self.rgb_image is None:
            print("[状态机] RGB图像未就绪，等待...")
            return
        
        position = self.yellow_line_detector.detect_position(self.rgb_image)
        print(f"[状态机] RGB相机检测位置: {position}")
        
        self.position_check_pending = False
        
        if position == 'left':
            print("[状态机] 靠近左线，向右转一步")
            self.execute_position_correction('turn_right', {'count': 1})
        elif position == 'right':
            print("[状态机] 靠近右线，向左转一步")
            self.execute_position_correction('turn_left', {'count': 1})
        else:
            print("[状态机] 位置正确，进入下一阶段")
            self.proceed_to_next_stage()
    
    def execute_position_correction(self, movement_type, custom_params=None):
        """执行位置修正"""
        print(f"[状态机] 执行位置修正: {movement_type}")
        if custom_params is None:
            custom_params = {'count': 1}  # 只执行一步
        self.send_motion_command(movement_type, custom_params)
        
        # 修正完成后，重新检查位置
        self.position_check_restart_timer = self.create_timer(3.0, self.restart_position_check_callback)
    
    def restart_position_check_callback(self):
        """重新开始位置检查的回调"""
        # 取消定时器
        if hasattr(self, 'position_check_restart_timer'):
            self.position_check_restart_timer.cancel()
            self.position_check_restart_timer = None
        
        self.restart_position_check()
    
    def restart_position_check(self):
        """重新开始位置检查"""
        self.position_check_pending = True
        if self.position_check_type == 'fisheye':
            self.check_fisheye_position()
        else:
            self.check_rgb_position()
    
    def proceed_to_next_stage(self):
        """进入下一阶段"""
        # 继续执行运动序列的下一个动作
        self.movement_step += 1
        self.execute_next_movement()
    
    def transition_to_state(self, new_state):
        """状态转换"""
        if new_state != self.current_state:
            self.previous_state = self.current_state
            self.current_state = new_state
            
            # 简约的状态输出
            self.print_status()
            
            # 发布状态信息
            state_msg = String()
            state_msg.data = f"State:{self.current_state.value}"
            self.state_publisher.publish(state_msg)
            
            # 执行状态进入动作
            self.on_state_entered(new_state)
    
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
            ('check_fisheye_position', None),              # 检查鱼眼相机位置
            ('turn_right', {'count': 11}),                 # 运动类型5: 右转
            ('check_rgb_position', None),                  # 检查RGB相机位置
        ]
        self.movement_step = 0
        self.execute_next_movement()
    
    def qra_to_a1_sequence(self):
        """qra_to_a1状态：执行运动类型3,然后执行运动类型2"""
        print("[状态机] 开始qra_to_a1运动序列")
        self.movement_sequence = [
            ('forward', {'count': 23}),                     # 运动类型1: 向前
            ('right', {'count': 21}), 
            ('backward', {'count': 9})
        ]
        self.movement_step = 0
        self.execute_next_movement()
    
    def qra_to_a2_sequence(self):
        """qra_to_a2状态：执行运动类型4,然后执行运动类型2"""
        print("[状态机] 开始qra_to_a2运动序列")
        self.movement_sequence = [
            ('forward', {'count': 23}),                     # 运动类型1: 向前
            ('left', {'count': 21}), 
            ('backward', {'count': 9})
        ]
        self.movement_step = 0
        self.execute_next_movement()
    
    def start_in_a1_sequence(self):
        """in_a1状态：执行运动类型9"""
        print("[状态机] 开始in_a1运动序列")
        self.movement_sequence = ['lie_down']

        self.movement_step = 0
        self.execute_next_movement()
    
    def start_in_a2_sequence(self):
        """in_a2状态：执行运动类型9"""
        print("[状态机] 开始in_a2运动序列")
        self.movement_sequence = ['lie_down']
        self.movement_step = 0
        self.execute_next_movement()
    
    def start_a1_to_s1_sequence(self):
        """a1_to_s1状态：执行运动类型8,然后执行任务1,然后执行任务4,然后执行任务2,然后执行任务6"""
        print("[状态机] 开始a1_to_s1运动序列")
        self.movement_sequence = [
            ('stand', None),                               # 运动类型8: 站立
            ('forward', {'count': 9}),                    # 任务1: 向前
            ('left', {'count': 21}),                       # 任务4: 向左
            ('backward', {'count': 24}),                    # 任务2: 向后
            ('turn_left', {'count': 11})                   # 任务6: 左转
        ]
        self.movement_step = 0
        self.execute_next_movement()
    
    def start_a2_to_s1_sequence(self):
        """a2_to_s1状态：执行运动类型8,然后执行任务1,然后执行任务3,然后执行任务2,然后执行任务6"""
        print("[状态机] 开始a2_to_s1运动序列")
        self.movement_sequence = [
            ('stand', None),                               # 运动类型8: 站立
            ('forward', {'count': 9}),                    # 任务1: 向前
            ('right', {'count': 21}),                      # 任务3: 向右
            ('backward', {'count': 24}),                    # 任务2: 向后
            ('turn_left', {'count': 11})                   # 任务6: 左转
        ]
        self.movement_step = 0
        self.execute_next_movement()
    
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
            print("[状态机] in_a1运动序列完成，进入a1_to_s1状态")
            self.transition_to_state(State.A1_TO_S1)
        elif self.current_state == State.IN_A2:
            print("[状态机] in_a2运动序列完成，进入a2_to_s1状态")
            self.transition_to_state(State.A2_TO_S1)
        elif self.current_state == State.A1_TO_S1:
            print("[状态机] a1_to_s1运动序列完成，进入end状态")
            self.transition_to_state(State.END)
        elif self.current_state == State.A2_TO_S1:
            print("[状态机] a2_to_s1运动序列完成，进入end状态")
            self.transition_to_state(State.END)
    
    def start_qr_detection(self):
        """开始在qr_a点进行二维码识别"""
        print("[状态机] 到达二维码A点，开始二维码识别")
        self.qr_detection_start_time = time.time()
        # 停止运动，准备识别
        self.stop_motion_timer()
    
    def process_qr_result(self):
        """处理二维码识别结果"""
        if self.qr_result == "A-1":
            print(f"[状态机] 识别到二维码: {self.qr_result}，转到qra_to_a1状态")
            self.transition_to_state(State.QRA_TO_A1)
        elif self.qr_result == "A-2":
            print(f"[状态机] 识别到二维码: {self.qr_result}，转到qra_to_a2状态")
            self.transition_to_state(State.QRA_TO_A2)
        else:
            print(f"[状态机] 未知二维码: {self.qr_result}")
            self.transition_to_state(State.ERROR)
    
    def on_task_completed(self):
        """任务完成处理"""
        self.task_completed = True
        print("[状态机] 任务完成！清理，关闭")
        
        # 停止运动定时器
        self.stop_motion_timer()
    
    def on_error_state(self):
        """错误状态处理"""
        print("[状态机] 进入错误状态，急停运动")
        
        # 停止运动定时器
        self.stop_motion_timer()
        
        # 急停
        self.send_motion_command('lie_down')
    
    def send_motion_command(self, movement_type, custom_params=None):
        """发送运动指令 - 支持持续发布（类似ros2 topic pub -r 频率 -t 次数）和服务调用"""
        try:
            # 处理特殊的检查命令
            if movement_type == 'check_fisheye_position':
                print("[状态机] 开始鱼眼相机位置检查")
                self.position_check_pending = True
                self.position_check_type = 'fisheye'
                self.check_fisheye_position()
                return
            elif movement_type == 'check_rgb_position':
                print("[状态机] 开始RGB相机位置检查")
                self.position_check_pending = True
                self.position_check_type = 'rgb'
                self.check_rgb_position()
                return
            
            # 检查是否是lie_down动作，如果是则先等待5秒
            if movement_type == 'lie_down':
                print("[状态机] 准备执行lie_down，先等待5秒让机器人稳定...")
                self.lie_down_wait_timer = self.create_timer(5.0, self._execute_lie_down_after_wait_callback)
                self.pending_lie_down_params = custom_params
                return
            
            # 执行其他运动指令
            self._execute_motion_command(movement_type, custom_params)
            
        except Exception as e:
            self.get_logger().error(f'发送运动指令时出错: {str(e)}')
    
    def _execute_lie_down_after_wait_callback(self):
        """等待后执行lie_down的回调"""
        print("[状态机] 等待完成，现在执行lie_down")
        # 取消定时器
        if hasattr(self, 'lie_down_wait_timer'):
            self.lie_down_wait_timer.cancel()
            self.lie_down_wait_timer = None
        
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
            
            # 如果需要发送多次，启动定时器
            if total_count > 1 and frequency > 0:
                timer_period = 1.0 / frequency  # 计算定时器周期
                self.motion_timer = self.create_timer(timer_period, self._motion_timer_callback)
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
            msg.vel_des = config['vel_des']
            msg.rpy_des = config['rpy_des']
            msg.pos_des = config['pos_des']
            msg.acc_des = config['acc_des']
            msg.ctrl_point = config['ctrl_point']
            msg.foot_pose = config['foot_pose']
            msg.step_height = config['step_height']

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
                # 创建10秒延时定时器
                self.lie_down_final_timer = self.create_timer(10.0, self.on_lie_down_wait_completed_callback)
            else:
                # 其他服务调用完成，直接进入下一个运动
                self.on_single_movement_completed()
                
        except Exception as e:
            self.get_logger().error(f'运动服务调用失败: {str(e)}')
            # 服务调用失败，直接进入下一个运动
            self.on_single_movement_completed()
    
    def on_lie_down_wait_completed_callback(self):
        """lie_down最终等待完成回调"""
        # 取消定时器
        if hasattr(self, 'lie_down_final_timer'):
            self.lie_down_final_timer.cancel()
            self.lie_down_final_timer = None
        
        print("[状态机] lie_down等待10秒完成，继续执行")
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
                    movement_type = self.movement_sequence[self.movement_step]
                
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
        movement_type = self.movement_sequence[self.movement_step] if self.movement_step < len(self.movement_sequence) else "unknown"
        print(f"[状态机] 运动 {movement_type} 完成")
        
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
        self.qr_detection_start_time = None
        self.movement_start_time = None
        self.task_completed = False
        self.movement_step = 0
        self.movement_sequence = []
        self.motion_count = 0
        
        # 停止运动定时器
        self.stop_motion_timer()
        
        print("[状态机] 已重置，可以重新开始")
    
    def check_timeouts(self):
        """检查超时"""
        current_time = time.time()
        
        # 检查二维码识别超时
        if (self.current_state == State.IN_QR_A and 
            self.qr_detection_start_time and 
            current_time - self.qr_detection_start_time > self.qr_detection_timeout):
            print("[状态机] 二维码识别超时")
            self.transition_to_state(State.ERROR)
    
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
        # 停止运动控制定时器
        self.stop_motion_timer()
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