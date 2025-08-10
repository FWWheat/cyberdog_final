#!/usr/bin/env python3

"""
两条黄线走中间控制模块 - S形路径控制

功能描述：
    - 在S形路径中保持两条黄线中间行走
    - 使用左鱼眼相机作为主要参考，右鱼眼和RGB相机作为辅助
    - 保证机器人不出界（赛道宽83cm，机器人宽33.9cm）
    - 接收开始/结束控制命令
    
使用方法：
    ros2 run state_machine yellow_line_walker
    
控制命令：
    ros2 topic pub -1 /yellow_line_walker/command std_msgs/msg/String "data: START"
    ros2 topic pub -1 /yellow_line_walker/command std_msgs/msg/String "data: STOP"
    
发布话题：
    /mi_desktop_48_b0_2d_7b_03_d0/motion_servo_cmd - 运动控制指令
    /yellow_line_walker/status - 当前状态信息
    
订阅话题：
    /image_left - 左鱼眼相机图像（主要参考）
    /image_right - 右鱼眼相机图像（辅助）
    /image_rgb - RGB相机图像（辅助）
    /yellow_line_walker/command - 控制命令
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from protocol.msg import MotionServoCmd
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
import numpy as np
from cv_bridge import CvBridge
from state_machine.yellow_line_detector import YellowLineDetector
import time
from enum import Enum


class WalkerState(Enum):
    IDLE = 0      # 空闲状态
    WALKING = 1   # 行走状态
    ADJUSTING = 2 # 调整状态


class YellowLineWalker(Node):
    def __init__(self):
        super().__init__('yellow_line_walker')
        
        # CV桥接器
        self.bridge = CvBridge()
        
        # 黄线检测器
        self.yellow_detector = YellowLineDetector()
        
        # 状态管理
        self.current_state = WalkerState.IDLE
        self.walking_active = False
        self.last_command_time = time.time()
        
        # 图像存储
        self.left_fisheye_image = None
        self.right_fisheye_image = None
        self.rgb_image = None
        
        # 控制参数
        self.forward_speed = 0.3      # 前进速度 (m/s)
        self.turn_speed = 0.6         # 转向速度 (rad/s)
        self.position_tolerance = 0.1 # 位置容忍度
        self.control_frequency = 10   # 控制频率 (Hz)
        
        # 赛道参数
        self.track_width = 0.83       # 赛道宽度 (m)
        self.robot_width = 0.339      # 机器人宽度 (m)
        self.safe_margin = 0.05       # 安全边距 (m)
        
        # ROI参数（针对不同相机）
        self.left_roi_params = {'top_ratio': 0.6, 'bottom_ratio': 1.0}
        self.right_roi_params = {'top_ratio': 0.6, 'bottom_ratio': 1.0}
        self.rgb_roi_params = {'top_ratio': 0.7, 'bottom_ratio': 1.0}
        
        # 阈值参数
        self.detection_thresholds = {
            'position_threshold': 0.08,  # 位置检测阈值
            'area_threshold': 300        # 面积阈值
        }
        
        # 配置QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # 订阅图像话题
        self.left_image_sub = self.create_subscription(
            Image, '/image_left', self.left_image_callback, qos_profile
        )
        self.right_image_sub = self.create_subscription(
            Image, '/image_right', self.right_image_callback, qos_profile
        )
        self.rgb_image_sub = self.create_subscription(
            Image, '/image_rgb', self.rgb_image_callback, qos_profile
        )
        
        # 订阅控制命令
        self.command_sub = self.create_subscription(
            String, '/yellow_line_walker/command', self.command_callback, 10
        )
        
        # 发布运动控制指令
        self.motion_pub = self.create_publisher(
            MotionServoCmd, '/mi_desktop_48_b0_2d_7b_03_d0/motion_servo_cmd', 10
        )
        
        # 发布状态信息
        self.status_pub = self.create_publisher(
            String, '/yellow_line_walker/status', 10
        )
        
        # 控制定时器
        self.control_timer = self.create_timer(
            1.0 / self.control_frequency, self.control_loop
        )
        
        self.get_logger().info('两条黄线走中间控制节点已启动')
        self.get_logger().info(f'控制频率: {self.control_frequency}Hz')
        self.get_logger().info(f'前进速度: {self.forward_speed}m/s, 转向速度: {self.turn_speed}rad/s')
        self.get_logger().info(f'赛道宽度: {self.track_width}m, 机器人宽度: {self.robot_width}m')
        self.get_logger().info('发送START命令开始行走: ros2 topic pub -1 /yellow_line_walker/command std_msgs/msg/String "data: START"')
        self.get_logger().info('发送STOP命令停止行走: ros2 topic pub -1 /yellow_line_walker/command std_msgs/msg/String "data: STOP"')
    
    def left_image_callback(self, msg):
        """左鱼眼相机图像回调"""
        try:
            self.left_fisheye_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'处理左鱼眼图像时出错: {str(e)}')
    
    def right_image_callback(self, msg):
        """右鱼眼相机图像回调"""
        try:
            self.right_fisheye_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'处理右鱼眼图像时出错: {str(e)}')
    
    def rgb_image_callback(self, msg):
        """RGB相机图像回调"""
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'处理RGB图像时出错: {str(e)}')
    
    def command_callback(self, msg):
        """控制命令回调"""
        command = msg.data.strip().upper()
        self.get_logger().info(f'收到控制命令: {command}')
        
        if command == "START":
            if not self.walking_active:
                self.walking_active = True
                self.current_state = WalkerState.WALKING
                self.last_command_time = time.time()
                self.get_logger().info('开始黄线中间行走')
            else:
                self.get_logger().info('已经在行走中')
                
        elif command == "STOP":
            if self.walking_active:
                self.walking_active = False
                self.current_state = WalkerState.IDLE
                self.send_stop_command()
                self.get_logger().info('停止黄线中间行走')
            else:
                self.get_logger().info('已经处于停止状态')
                
        else:
            self.get_logger().warn(f'未知控制命令: {command}')
    
    def control_loop(self):
        """主控制循环"""
        try:
            # 发布状态信息
            self.publish_status()
            
            # 如果不在行走状态，直接返回
            if not self.walking_active or self.current_state == WalkerState.IDLE:
                return
            
            # 检查图像是否就绪
            if not self.images_ready():
                return
            
            # 执行黄线跟随控制
            self.execute_line_following()
            
        except Exception as e:
            self.get_logger().error(f'控制循环出错: {str(e)}')
    
    def images_ready(self) -> bool:
        """检查图像是否就绪"""
        if self.left_fisheye_image is None:
            return False
        # 右鱼眼和RGB作为辅助，不是必需的
        return True
    
    def execute_line_following(self):
        """执行黄线跟随控制"""
        try:
            # 1. 使用左鱼眼相机进行主要位置检测
            left_position = self.yellow_detector.detect_position(
                self.left_fisheye_image, 
                detection_type='ycy',
                roi_params=self.left_roi_params,
                camera_type='fisheye_left',
                threshold_params=self.detection_thresholds
            )
            
            # 2. 使用右鱼眼相机作为辅助参考
            right_position = None
            if self.right_fisheye_image is not None:
                right_position = self.yellow_detector.detect_position(
                    self.right_fisheye_image,
                    detection_type='ycy', 
                    roi_params=self.right_roi_params,
                    camera_type='fisheye_right',
                    threshold_params=self.detection_thresholds
                )
            
            # 3. 使用RGB相机作为辅助参考
            rgb_position = None
            if self.rgb_image is not None:
                rgb_position = self.yellow_detector.detect_position(
                    self.rgb_image,
                    detection_type='ycy',
                    roi_params=self.rgb_roi_params, 
                    camera_type='rgb',
                    threshold_params=self.detection_thresholds
                )
            
            # 4. 融合多个相机的检测结果
            final_decision = self.fuse_detection_results(left_position, right_position, rgb_position)
            
            # 5. 根据融合结果生成运动控制指令
            self.generate_motion_command(final_decision)
            
        except Exception as e:
            self.get_logger().error(f'执行黄线跟随控制时出错: {str(e)}')
    
    def fuse_detection_results(self, left_pos: str, right_pos: str, rgb_pos: str) -> str:
        """
        融合多个相机的检测结果
        
        Args:
            left_pos: 左鱼眼检测结果
            right_pos: 右鱼眼检测结果  
            rgb_pos: RGB相机检测结果
            
        Returns:
            融合后的位置决策
        """
        # 以左鱼眼为主要参考
        primary_result = left_pos
        
        # 如果有辅助相机的结果，进行一致性检查
        consistent_results = [primary_result]
        
        if right_pos:
            consistent_results.append(right_pos)
        if rgb_pos:
            consistent_results.append(rgb_pos)
        
        # 统计各种结果的出现次数
        result_counts = {'left': 0, 'right': 0, 'center': 0}
        for result in consistent_results:
            if result in result_counts:
                result_counts[result] += 1
        
        # 选择出现次数最多的结果
        final_result = max(result_counts, key=result_counts.get)
        
        # 如果主要参考（左鱼眼）与其他结果差异较大，优先相信左鱼眼
        if len(consistent_results) > 1 and primary_result != final_result:
            self.get_logger().info(f'检测结果不一致: 左鱼眼={left_pos}, 右鱼眼={right_pos}, RGB={rgb_pos}, 采用左鱼眼结果')
            final_result = primary_result
        
        return final_result
    
    def generate_motion_command(self, position_decision: str):
        """
        根据位置决策生成运动控制指令
        
        Args:
            position_decision: 位置决策结果 ('left', 'right', 'center')
        """
        try:
            # 创建运动控制消息
            motion_cmd = MotionServoCmd()
            motion_cmd.motion_id = 303        # 机器人运控姿态
            motion_cmd.cmd_type = 1           # Data帧
            motion_cmd.cmd_source = 4         # Algo算法来源
            motion_cmd.value = 0              # 内八步态
            motion_cmd.rpy_des = [0.0, 0.0, 0.0]
            motion_cmd.pos_des = [0.0, 0.0, 0.0]
            motion_cmd.acc_des = [0.0, 0.0, 0.0]
            motion_cmd.ctrl_point = [0.0, 0.0, 0.0]
            motion_cmd.foot_pose = [0.0, 0.0, 0.0]
            motion_cmd.step_height = [0.05, 0.05]
            
            # 根据位置决策设置速度
            if position_decision == 'left':
                # 机器人偏左，需要向右调整
                motion_cmd.vel_des = [self.forward_speed, -0.1, 0.1]  # 前进+右移+右转
                self.current_state = WalkerState.ADJUSTING
                
            elif position_decision == 'right':
                # 机器人偏右，需要向左调整  
                motion_cmd.vel_des = [self.forward_speed, 0.1, -0.1]  # 前进+左移+左转
                self.current_state = WalkerState.ADJUSTING
                
            else:  # center
                # 机器人在中间，直线前进
                motion_cmd.vel_des = [self.forward_speed, 0.0, 0.0]  # 直线前进
                self.current_state = WalkerState.WALKING
            
            # 发布运动指令
            self.motion_pub.publish(motion_cmd)
            
            # 记录控制信息
            if position_decision != 'center':
                self.get_logger().info(f'位置调整: {position_decision} -> 速度设置: {motion_cmd.vel_des}')
            
        except Exception as e:
            self.get_logger().error(f'生成运动指令时出错: {str(e)}')
    
    def send_stop_command(self):
        """发送停止指令"""
        try:
            motion_cmd = MotionServoCmd()
            motion_cmd.motion_id = 303
            motion_cmd.cmd_type = 1
            motion_cmd.cmd_source = 4
            motion_cmd.value = 0
            motion_cmd.vel_des = [0.0, 0.0, 0.0]  # 停止
            motion_cmd.rpy_des = [0.0, 0.0, 0.0]
            motion_cmd.pos_des = [0.0, 0.0, 0.0]
            motion_cmd.acc_des = [0.0, 0.0, 0.0]
            motion_cmd.ctrl_point = [0.0, 0.0, 0.0]
            motion_cmd.foot_pose = [0.0, 0.0, 0.0]
            motion_cmd.step_height = [0.05, 0.05]
            
            self.motion_pub.publish(motion_cmd)
            
        except Exception as e:
            self.get_logger().error(f'发送停止指令时出错: {str(e)}')
    
    def publish_status(self):
        """发布状态信息"""
        try:
            status_msg = String()
            status_data = {
                'state': self.current_state.name,
                'walking_active': self.walking_active,
                'images_ready': self.images_ready(),
                'time': time.time() - self.last_command_time
            }
            status_msg.data = f"State:{status_data['state']},Active:{status_data['walking_active']},ImagesReady:{status_data['images_ready']},Runtime:{status_data['time']:.1f}s"
            self.status_pub.publish(status_msg)
            
        except Exception as e:
            self.get_logger().error(f'发布状态信息时出错: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    yellow_line_walker = YellowLineWalker()
    
    try:
        rclpy.spin(yellow_line_walker)
    except KeyboardInterrupt:
        pass
    finally:
        yellow_line_walker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()