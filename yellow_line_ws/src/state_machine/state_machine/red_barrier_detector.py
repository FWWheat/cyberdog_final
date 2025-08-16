#!/usr/bin/env python3

"""
红色限高杆识别模块 - 改进版
针对实际比赛环境优化，包含状态跟踪逻辑

功能描述：
    - 检测图像中的红色限高杆区域
    - 状态跟踪：检测到红色时不发布，红色消失后发布固定距离"50"
    - 多ROI策略适应不同距离的限高杆检测
    
状态逻辑：
    1. 检测红线位置，当红线到达图片顶部时发布话题
    2. 监控红线垂直位置变化，到达顶部区域时触发
    
使用方法：
    ros2 run state_machine red_barrier_detector
    
发布话题：
    /red_barrier_detector/distance - 红线到达顶部时的距离信息
    /red_barrier_detector/debug_image - 调试图像（可选）
    
订阅话题：
    /image_rgb - RGB相机图像
"""

from traceback import print_tb
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
import numpy as np
from cv_bridge import CvBridge
from typing import Optional, Tuple, Dict, Any


class RedBarrierDetector(Node):
    def __init__(self):
        super().__init__('red_barrier_detector')
        
        # CV桥接器
        self.bridge = CvBridge()
        
        # 改进的红色检测参数 - 基于实际测试优化
        self.lower_red1 = np.array([0, 50, 50])      # 主要红色范围
        self.upper_red1 = np.array([10, 255, 255])   
        self.lower_red2 = np.array([170, 50, 50])    # 深红色范围
        self.upper_red2 = np.array([180, 255, 255])  
        
        # 橙红色范围（室内LED灯光下的红色）
        self.lower_orange_red = np.array([0, 80, 80])
        self.upper_orange_red = np.array([8, 255, 255])
        
        # 深红色范围（阴影中的红色）
        self.lower_dark_red = np.array([160, 40, 40])
        self.upper_dark_red = np.array([180, 255, 200])
        
        # 检测参数 - 针对实际限高杆优化
        self.min_contour_area = 100      # 最小轮廓面积
        self.min_aspect_ratio = 0.05     # 最小高宽比
        self.max_aspect_ratio = 20.0     # 最大高宽比
        self.detection_threshold = 0.2   # 检测置信度阈值
        
        # 多层ROI策略 - 距离分层策略
        self.roi_configs = {
            'very_near': {'top': 0.0, 'bottom': 1.0},    # 极近距离：全图检测
            'near': {'top': 0.0, 'bottom': 0.8},         # 近距离：大部分区域
            'medium': {'top': 0.1, 'bottom': 0.7},       # 中距离：稍微上移
            'far': {'top': 0.2, 'bottom': 0.8},          # 远距离：中部区域
        }
        
        # 距离估算参数
        self.reference_height_pixels = 60
        self.reference_distance = 2.0
        
        # 形状验证参数
        self.min_width = 3
        self.max_width = 800
        self.min_height = 3
        self.max_height = 800
        
        # 状态跟踪变量 
        self.red_at_top = False               # 红线是否已到达顶部
        self.top_zone_threshold = 0.1        # 顶部区域阈值
        self.published_top_arrival = False    # 是否已发布到达顶部消息
        self.publish_triggered = False        # 是否已触发持续发布模式
        
        # 调试统计变量
        self.frame_count = 0                  # 总帧数
        self.detection_count = 0              # 检测到的帧数
        self.publish_count = 0                # 发布的帧数
        self.last_detection_time = None       # 最后检测时间
        self.state_change_time = None         # 状态改变时间
        
        # 调试模式
        self.debug_mode = self.declare_parameter('debug_mode', True).value
        
        # 配置QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        
        # 订阅RGB图像
        self.image_subscription = self.create_subscription(
            Image,
            '/image_rgb',
            self.image_callback,
            qos_profile
        )
        
        # 发布距离信息
        self.distance_publisher = self.create_publisher(
            String,
            '/red_barrier_detector/distance',
            10
        )
        
        # 发布调试图像
        if self.debug_mode:
            self.debug_image_publisher = self.create_publisher(
                Image,
                '/red_barrier_detector/debug_image',
                qos_profile
            )
        
        self.get_logger().info('红色限高杆识别节点已启动 (改进版)')
        self.get_logger().info(f'参数配置: debug_mode={self.debug_mode}')
        self.get_logger().info('订阅话题: /image_rgb')
        self.get_logger().info('发布话题: /red_barrier_detector/distance (红线到达顶部时发布)')
        if self.debug_mode:
            self.get_logger().info('发布调试话题: /red_barrier_detector/debug_image')
        self.get_logger().info('状态逻辑: 检测红线位置，到达图片顶部时发布话题')
    
    def image_callback(self, msg):
        """图像回调函数"""
        try:
            print("*********")
            # 更新帧计数
            self.frame_count += 1
            current_time = self.get_clock().now().nanoseconds / 1e9
            
            # 转换图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 使用状态逻辑检测
            should_publish, distance_value, detection_result = self.get_barrier_distance(cv_image)
            
            # 更新统计信息
            if detection_result:
                self.detection_count += 1
                self.last_detection_time = current_time
            
            if should_publish:
                self.publish_count += 1
                
                # 检查是否是第一次发布到达顶部消息
                if not self.published_top_arrival:
                    self.published_top_arrival = True
                    self.get_logger().info(f'红线到达顶部，开始发布距离"50"! 帧#{self.frame_count}')
            
            # 根据状态逻辑决定是否发布
            if should_publish:
                self.publish_distance(distance_value)
                if self.frame_count % 10 == 0:  # 每10帧记录一次，避免日志过多
                    self.get_logger().info(f'红线在顶部，发布距离: {distance_value}米 (帧#{self.frame_count})')
            elif detection_result:
                bbox = detection_result['bbox']
                red_top_y = bbox[1]
                image_height = cv_image.shape[0]
                top_zone_height = int(image_height * self.top_zone_threshold)
                self.get_logger().info(
                    f'检测到红线 - y={red_top_y}, 阈值={top_zone_height}, '
                    f'距离:{detection_result["distance"]:.2f}m, '
                    f'置信度:{detection_result["confidence"]:.3f}, '
                    f'帧#{self.frame_count} (未到达顶部)')
            
            # 每100帧输出统计信息
            if self.frame_count % 100 == 0:
                detection_rate = (self.detection_count / self.frame_count) * 100
                publish_rate = (self.publish_count / self.frame_count) * 100
                self.get_logger().info(
                    f'统计信息 - 总帧数:{self.frame_count}, 检测率:{detection_rate:.1f}%, '
                    f'发布率:{publish_rate:.1f}%, 状态:{"红线在顶部" if self.red_at_top else "红线未在顶部"}'
                )
            
            # 发布调试图像
            if self.debug_mode:
                self.publish_debug_image(cv_image, should_publish, distance_value, detection_result, msg.header)
                
        except Exception as e:
            self.get_logger().error(f'处理图像时出错 (帧#{self.frame_count}): {str(e)}', throttle_duration_sec=1.0)
    
    def get_barrier_distance(self, image: np.ndarray) -> Tuple[bool, Optional[str], Optional[Dict[str, Any]]]:
        """
        检测红线位置并判断是否到达顶部
        
        Args:
            image: 输入图像 (BGR格式)
            
        Returns:
            (是否发布距离, 距离值, 检测结果)
        """
        # 如果已经触发发布模式，直接返回发布状态，不再进行检测
        if self.publish_triggered:
            return True, "50", None
        
        # 执行检测
        detection_result = self.detect_red_barrier(image)
        
        if detection_result:
            # 检测到红色线，判断位置
            bbox = detection_result['bbox']
            red_top_y = bbox[1]  # 红线的顶部y坐标
            image_height = image.shape[0]
            print(f"检测框位置: {detection_result['bbox']}")
            print(f"红线高度: y={red_top_y} (图片高度: {image_height})")
            # 判断红线是否在顶部区域
            top_zone_height = int(image_height * self.top_zone_threshold)
            
            if red_top_y <= top_zone_height:
                # 红线已到达顶部区域，触发持续发布模式
                if not self.publish_triggered:
                    # 第一次到达顶部，记录状态变更并触发持续发布
                    self.get_logger().info(f'🎯 红线首次到达顶部! y={red_top_y}, 阈值={top_zone_height}')
                    self.get_logger().info(f'🔥 触发持续发布模式，停止检测')
                    self.red_at_top = True
                    self.published_top_arrival = True
                    self.publish_triggered = True
                
                # 发布固定距离"50"
                return True, "50", detection_result
            else:
                # 红线还未到达顶部
                self.red_at_top = False
                return False, None, detection_result
        else:
            # 未检测到红线
            self.red_at_top = False
            return False, None, None
    
    def detect_red_barrier(self, image: np.ndarray) -> Optional[Dict[str, Any]]:
        """
        改进的红色限高杆检测 - 多ROI策略
        
        Args:
            image: 输入图像 (BGR格式)
            
        Returns:
            检测结果字典或None
        """
        try:
            height, width = image.shape[:2]
            
            # 按优先级尝试不同ROI配置 - 极近距离优先
            roi_priority = ['very_near', 'near', 'medium', 'far']
            best_detection = None
            best_roi_config = None
            
            for roi_name in roi_priority:
                roi_config = self.roi_configs[roi_name]
                detection = self._detect_in_roi(image, roi_config, roi_name)
                
                if detection and (best_detection is None or detection['confidence'] > best_detection['confidence']):
                    best_detection = detection
                    best_roi_config = roi_name
                    
                    # 如果检测置信度足够高，直接返回
                    if detection['confidence'] > 0.7:
                        break
            
            if best_detection:
                best_detection['roi_name'] = best_roi_config
                return best_detection
            
            return None
            
        except Exception as e:
            self.get_logger().error(f'检测红色限高杆时出错: {str(e)}')
            return None
    
    def _detect_in_roi(self, image: np.ndarray, roi_config: Dict[str, float], roi_name: str) -> Optional[Dict[str, Any]]:
        """在指定ROI中检测限高杆"""
        try:
            height, width = image.shape[:2]
            
            # 计算ROI区域
            roi_top = int(height * roi_config['top'])
            roi_bottom = int(height * roi_config['bottom'])
            roi_image = image[roi_top:roi_bottom, :]
            
            if roi_image.size == 0:
                return None
            
            # HSV转换
            hsv = cv2.cvtColor(roi_image, cv2.COLOR_BGR2HSV)
            
            # 多重红色掩码组合
            mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
            mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
            mask3 = cv2.inRange(hsv, self.lower_orange_red, self.upper_orange_red)
            mask4 = cv2.inRange(hsv, self.lower_dark_red, self.upper_dark_red)
            
            # 组合所有掩码
            mask = cv2.bitwise_or(mask1, mask2)
            mask = cv2.bitwise_or(mask, mask3)
            mask = cv2.bitwise_or(mask, mask4)
            
            # 形态学操作
            kernel_small = np.ones((2, 2), np.uint8)
            kernel_large = np.ones((3, 3), np.uint8)
            
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_small)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_large)
            
            # 查找轮廓
            contours_result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours_result) == 3:
                _, contours, _ = contours_result
            else:
                contours, _ = contours_result
            
            # 过滤轮廓
            valid_contours = []
            for c in contours:
                if c is not None and len(c) > 4:
                    valid_contours.append(c)
            contours = valid_contours
            
            if not contours:
                return None
            
            # 根据ROI调整检测参数
            min_area = self.min_contour_area
            if roi_name == 'very_near':
                min_area = self.min_contour_area * 2     # 极近距离需要更大面积避免误检
            elif roi_name == 'near':
                min_area = self.min_contour_area * 1.5   # 近距离稍微提高要求
            elif roi_name == 'medium':
                min_area = self.min_contour_area
            elif roi_name == 'far':
                min_area = self.min_contour_area * 0.5   # 远距离放宽要求
            
            # 轮廓分析
            best_contour = None
            best_score = 0
            best_bbox = None
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < min_area:
                    continue
                
                x, y, w, h = cv2.boundingRect(contour)
                
                # 尺寸过滤
                if (w < self.min_width or w > self.max_width or 
                    h < self.min_height or h > self.max_height):
                    continue
                
                aspect_ratio = h / w if w > 0 else 0
                
                # 根据ROI调整高宽比要求
                min_ratio, max_ratio = self.min_aspect_ratio, self.max_aspect_ratio
                if roi_name == 'very_near':
                    # 极近距离时限高杆可能呈现各种形状，但不能太极端
                    min_ratio = 0.02
                    max_ratio = 50.0
                elif roi_name == 'near':
                    # 近距离时限高杆可能很矮很宽
                    min_ratio = 0.05
                    max_ratio = 20.0
                elif roi_name == 'medium':
                    min_ratio = 0.05
                    max_ratio = 20.0
                elif roi_name == 'far':
                    min_ratio = 0.05
                    max_ratio = 20.0
                
                if aspect_ratio < min_ratio or aspect_ratio > max_ratio:
                    continue
                
                # 计算多维度分数
                bbox_area = w * h
                fill_ratio = area / bbox_area if bbox_area > 0 else 0
                
                # 面积分数
                area_score = min(area / 1000, 1.0)
                
                # 高宽比分数 - 根据ROI调整理想比例
                if roi_name == 'very_near':
                    ideal_ratio = 1.0  # 极近距离任意形状都可接受
                    ratio_score = 0.8  # 固定给高分
                elif roi_name == 'near':
                    ideal_ratio = 0.8
                    ratio_score = max(0, 1.0 - abs(aspect_ratio - ideal_ratio) / ideal_ratio)
                elif roi_name == 'medium':
                    ideal_ratio = 1.5
                    ratio_score = max(0, 1.0 - abs(aspect_ratio - ideal_ratio) / ideal_ratio)
                else:
                    ideal_ratio = 1.5
                    ratio_score = max(0, 1.0 - abs(aspect_ratio - ideal_ratio) / ideal_ratio)
                
                # 填充度分数
                fill_score = min(fill_ratio * 2, 1.0)
                
                # 位置分数
                center_x = x + w // 2
                roi_center = roi_image.shape[1] // 2
                position_score = 1.0 - abs(center_x - roi_center) / roi_center
                
                # ROI奖励分数 - 极近距离获得最高奖励
                roi_bonus = {
                    'very_near': 1.5,    # 极近距离最高优先级
                    'near': 1.3,         # 近距离高优先级
                    'medium': 1.1,       # 中距离正常优先级
                    'far': 1.0           # 远距离基础优先级
                }.get(roi_name, 1.0)
                
                # 综合评分
                total_score = (area_score * 0.25 + ratio_score * 0.2 + fill_score * 0.25 + 
                              position_score * 0.15 + 0.15) * roi_bonus
                
                if total_score > best_score and total_score > self.detection_threshold:
                    best_score = total_score
                    best_contour = contour
                    best_bbox = (x, y + roi_top, w, h)  # 转换到原图坐标
            
            if best_contour is None:
                return None
            
            # 估算距离
            distance = self.estimate_distance(best_bbox[3])
            
            return {
                'distance': distance,
                'confidence': best_score,
                'bbox': best_bbox,
                'contour': best_contour,
                'roi_top': roi_top,
                'roi_name': roi_name
            }
            
        except Exception as e:
            self.get_logger().error(f'在ROI中检测时出错: {str(e)}')
            return None
    
    def estimate_distance(self, object_height_pixels: int) -> float:
        """根据物体像素高度估算距离"""
        try:
            if object_height_pixels <= 0:
                return 10.0
            
            distance = (self.reference_height_pixels * self.reference_distance) / object_height_pixels
            return max(0.3, min(distance, 15.0))  # 0.3米到15米
            
        except Exception as e:
            self.get_logger().error(f'估算距离时出错: {str(e)}')
            return 5.0
    
    def publish_distance(self, distance: str):
        """发布距离信息"""
        msg = String()
        msg.data = distance
        self.distance_publisher.publish(msg)
    
    def publish_debug_image(self, image: np.ndarray, should_publish: bool, distance_value: Optional[str], 
                           detection_result: Optional[Dict[str, Any]], header):
        """发布调试图像"""
        if not self.debug_mode:
            return
        
        try:
            debug_image = image.copy()
            height, width = image.shape[:2]
            
            # 显示所有ROI配置区域
            colors = {
                'very_near': (0, 255, 0),     # 绿色 - 极近距离
                'near': (0, 255, 255),        # 黄色 - 近距离
                'medium': (255, 255, 0),      # 青色 - 中距离  
                'far': (255, 0, 255),         # 紫色 - 远距离
            }
            
            for roi_name, roi_config in self.roi_configs.items():
                roi_top = int(height * roi_config['top'])
                roi_bottom = int(height * roi_config['bottom'])
                color = colors.get(roi_name, (255, 255, 255))
                cv2.rectangle(debug_image, (0, roi_top), (width, roi_bottom), color, 1)
                cv2.putText(debug_image, roi_name, (10, roi_top + 15), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            
            # 高亮当前使用的ROI
            if detection_result and 'roi_name' in detection_result:
                roi_name = detection_result['roi_name']
                roi_config = self.roi_configs[roi_name]
                roi_top = int(height * roi_config['top'])
                roi_bottom = int(height * roi_config['bottom'])
                color = colors.get(roi_name, (255, 255, 255))
                cv2.rectangle(debug_image, (0, roi_top), (width, roi_bottom), color, 3)
                cv2.putText(debug_image, f"ACTIVE: {roi_name}", (width - 150, roi_top + 20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            
            # 绘制检测结果
            if detection_result:
                x, y, w, h = detection_result['bbox']
                cv2.rectangle(debug_image, (x, y), (x + w, y + h), (0, 255, 0), 3)
                
                # 绘制十字中心
                center_x, center_y = x + w//2, y + h//2
                cv2.line(debug_image, (center_x-15, center_y), (center_x+15, center_y), (0, 255, 0), 2)
                cv2.line(debug_image, (center_x, center_y-15), (center_x, center_y+15), (0, 255, 0), 2)
                
                distance = detection_result['distance']
                confidence = detection_result['confidence']
                roi_name = detection_result.get('roi_name', 'unknown')
                
                cv2.putText(debug_image, f"DETECTED ({roi_name})!", (x, y - 80), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.putText(debug_image, f"Distance: {distance:.2f}m", (x, y - 55), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(debug_image, f"Confidence: {confidence:.2f}", (x, y - 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(debug_image, f"Size: {w}x{h}", (x, y - 5), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # 状态信息
            if self.publish_triggered:
                result_text = f"TRIGGERED MODE - PUBLISHING: {distance_value}m"
                color = (0, 255, 0)  # 绿色
            elif detection_result:
                bbox = detection_result['bbox']
                red_top_y = bbox[1]
                top_zone_height = int(height * self.top_zone_threshold)
                
                if red_top_y <= top_zone_height:
                    result_text = f"RED AT TOP - PUBLISHING: {distance_value}m"
                    color = (0, 255, 0)  # 绿色
                else:
                    result_text = f"RED DETECTED - NOT AT TOP (y={red_top_y}, threshold={top_zone_height})"
                    color = (0, 255, 255)  # 黄色
            else:
                result_text = "NO RED LINE DETECTED"
                color = (0, 0, 255)  # 红色
            
            cv2.putText(debug_image, result_text, (10, height - 80), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            
            # 状态信息
            state_text = f"Frame#{self.frame_count} | Red_at_top={self.red_at_top} | Triggered={self.publish_triggered}"
            if self.detection_count > 0:
                state_text += f" | Det:{self.detection_count}/{self.frame_count}({(self.detection_count/self.frame_count)*100:.1f}%)"
            if self.publish_count > 0:
                state_text += f" | Pub:{self.publish_count}"
            cv2.putText(debug_image, state_text, (10, height - 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # 参数信息
            param_text = f"Area:{self.min_contour_area}+ Ratio:{self.min_aspect_ratio:.2f}-{self.max_aspect_ratio:.1f}"
            cv2.putText(debug_image, param_text, (10, height - 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # 发布调试图像
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            debug_msg.header = header
            self.debug_image_publisher.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f'发布调试图像时出错: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    red_barrier_detector = RedBarrierDetector()
    
    try:
        rclpy.spin(red_barrier_detector)
    except KeyboardInterrupt:
        pass
    finally:
        red_barrier_detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()