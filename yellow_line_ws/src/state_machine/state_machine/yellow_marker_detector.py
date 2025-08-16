#!/usr/bin/env python3

"""
黄色标志物识别模块 - 识别RGB图片中的黄色圆形/方形标志物并发布距离信息

功能描述：
    - 检测图像中的黄色区域
    - 识别黄色圆形或方形标志物（挂在高处）
    - 状态跟踪：检测到黄色时不发布，黄色消失后持续发布固定距离"50"
    
使用方法：
    ros2 run state_machine yellow_marker_detector
    
发布话题：
    /yellow_marker_detector/distance - 黄色消失后持续发布的距离信息
    /yellow_marker_detector/debug_image - 调试图像（可选）
    
订阅话题：
    /image_rgb - RGB相机图像
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
import numpy as np
from cv_bridge import CvBridge
from typing import Optional, Tuple, List


class YellowMarkerDetector(Node):
    def __init__(self):
        super().__init__('yellow_marker_detector')
        
        # CV桥接器
        self.bridge = CvBridge()
        
        # 黄色检测参数 - HSV色彩空间（优化后的精确范围）
        self.lower_yellow = np.array([15, 50, 50])     # 放宽的黄色下限
        self.upper_yellow = np.array([35, 255, 255])   # 放宽的黄色上限
        
        # 检测参数（整合精确检测逻辑）
        self.min_contour_area = 200       # 降低最小轮廓面积
        self.max_contour_area = 15000     # 增加最大轮廓面积以适应更宽的HSV范围
        self.roi_top_ratio = 0.0          # ROI顶部比例（图片上半部分）
        self.roi_bottom_ratio = 0.6       # ROI底部比例（扩大到60%覆盖更多天花板区域）
        self.valid_circle_y_ratio = 0.5   # 只接受ROI前50%区域内的圆形（真正的天花板区域）
        
        # 形状检测参数（整合精确霍夫圆参数）
        self.circle_min_radius = 5         # 减小最小半径
        self.circle_max_radius = 150       # 增加最大半径
        self.hough_min_dist = 20           # 圆心间最小距离
        self.hough_param1 = 20             # 边缘检测阈值
        self.hough_param2 = 10             # 累积阈值
        self.rect_aspect_ratio_tolerance = 0.3  # 方形长宽比容忍度
        
        # 距离估算参数（基于物体大小）
        self.reference_area_pixels = 2000  # 参考面积像素（1米距离时的像素面积）
        self.reference_distance = 1.0      # 参考距离（米）
        
        # 黄灯检测特定参数
        self.yellow_light_detection_mode = False  # 黄灯检测模式开关
        self.yellow_light_target_distance = 0.5   # 黄灯停止目标距离（米）
        self.yellow_light_distance_tolerance = 0.2 # 距离容忍度（米）
        self.yellow_light_min_radius = 20         # 黄灯最小半径（像素）
        self.yellow_light_max_radius = 150        # 黄灯最大半径（像素）
        self.yellow_light_real_diameter = 0.2     # 黄灯实际直径（米）
        
        # 消失检测状态机参数
        self.detection_history = []               # 检测历史记录
        self.history_max_length = 10              # 历史记录最大长度
        self.disappeared_threshold = 5            # 连续未检测到的帧数阈值
        self.is_marker_disappeared = False        # 标志物是否已消失
        self.last_detection_distance = None       # 最后检测到的距离
        self.disappear_distance = "50"            # 消失后发布的距离
        self.detection_state = "SEARCHING"        # SEARCHING, TRACKING, DISAPPEARED, PUBLISHING
        self.stop_detection = False               # 是否停止检测（进入PUBLISHING状态后为True）
        self.first_publish_done = False           # 是否已完成第一次发布
        
        # 调试统计变量
        self.frame_count = 0                      # 总帧数
        self.detection_count = 0                  # 检测到的帧数
        self.publish_count = 0                    # 发布的帧数
        self.last_detection_time = None           # 最后检测时间
        self.state_change_time = None             # 状态改变时间
        self.tracking_frames = 0                  # 跟踪状态的帧数
        
        # 调试模式
        self.debug_mode = self.declare_parameter('debug_mode', True).value
        self.detection_threshold = 0.4  # 检测置信度阈值
        
        # 配置QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
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
            '/yellow_marker_detector/distance',
            10
        )
        
        # 发布调试图像
        if self.debug_mode:
            self.debug_image_publisher = self.create_publisher(
                Image,
                '/yellow_marker_detector/debug_image',
                qos_profile
            )
        
        # 定时器：用于定时发布距离（在PUBLISHING状态）
        self.publish_timer = self.create_timer(0.1, self.timer_publish_callback)  # 100ms间隔
        
        self.get_logger().info('黄色标志物识别节点已启动（改进发布逻辑版）')
        self.get_logger().info(f'参数配置: debug_mode={self.debug_mode}')
        self.get_logger().info(f'状态跟踪: threshold={self.disappeared_threshold}, state={self.detection_state}')
        self.get_logger().info('订阅话题: /image_rgb')
        self.get_logger().info('发布话题: /yellow_marker_detector/distance (满足第一发布条件后定时发布)')
        if self.debug_mode:
            self.get_logger().info('发布调试话题: /yellow_marker_detector/debug_image')
        self.get_logger().info('新逻辑: 检测到黄色时不发布，消失后第一次发布，之后停止检测并定时发布50')
        self.get_logger().info('HSV黄色范围: {} - {}'.format(self.lower_yellow, self.upper_yellow))
    
    def image_callback(self, msg):
        """图像回调函数"""
        try:
            # 更新帧计数
            self.frame_count += 1
            current_time = self.get_clock().now().nanoseconds / 1e9
            
            # 调试：确认回调被调用
            if self.frame_count % 30 == 1:  # 每30帧输出一次，避免日志过多
                self.get_logger().info(f'📷 图像回调被调用 - 帧#{self.frame_count}, 状态:{self.detection_state}')
            
            # 如果已进入PUBLISHING状态，停止检测
            if self.detection_state == "PUBLISHING":
                if self.frame_count % 100 == 1:  # 减少日志频率
                    self.get_logger().info(f'⏸️  已停止检测 - 状态:PUBLISHING, 帧#{self.frame_count}')
                return
            
            # 转换图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 检测黄色标志物
            detection_result = self.detect_yellow_light_improved(cv_image)
            
            # 更新统计信息
            if detection_result:
                self.detection_count += 1
                self.last_detection_time = current_time
                distance, circle = detection_result
                x, y, radius = circle
                
                if self.frame_count % 10 == 1:  # 减少检测到时的日志频率
                    self.get_logger().info(
                        f'检测到黄色标志物 - 位置:({x},{y}), 半径:{radius}, 距离:{distance:.2f}m, '\
                        f'帧#{self.frame_count}, 状态:{self.detection_state}'
                    )
            
            # 更新状态机并获取应发布的距离
            old_state = self.detection_state
            published_distance = self.update_detection_state(detection_result)
            
            # 检查状态变化
            if old_state != self.detection_state:
                self.state_change_time = current_time
                self.get_logger().info(f'🔄 状态变更: {old_state} -> {self.detection_state} (帧#{self.frame_count})')
                
                if self.detection_state == "TRACKING":
                    self.tracking_frames = 0
                elif self.detection_state == "DISAPPEARED":
                    self.get_logger().info(f'📍 黄色标志物消失! 连续{self.disappeared_threshold}帧未检测到')
            
            # 更新跟踪帧数
            if self.detection_state == "TRACKING":
                self.tracking_frames += 1
            
            # 发布距离（仅在DISAPPEARED状态第一次发布）
            if published_distance is not None and not self.first_publish_done:
                self.publish_count += 1
                self.publish_distance(published_distance)
                self.first_publish_done = True
                
                elapsed = current_time - self.state_change_time if self.state_change_time else 0
                self.get_logger().info(f'📤 完成第一次发布距离"50"! 帧#{self.frame_count}, 消失后{elapsed:.1f}秒')
                self.get_logger().info(f'🔄 状态转换: {self.detection_state} -> PUBLISHING')
                self.detection_state = "PUBLISHING"
                self.stop_detection = True
            elif detection_result:
                distance, circle = detection_result
                if self.frame_count % 10 == 1:  # 减少检测日志频率
                    self.get_logger().info(f'检测到黄色标志物，距离: {distance:.2f}米，不发布距离 (状态:{self.detection_state})')
            
            # 每100帧输出统计信息
            if self.frame_count % 100 == 0:
                detection_rate = (self.detection_count / self.frame_count) * 100
                publish_rate = (self.publish_count / self.frame_count) * 100
                history_summary = "".join(["1" if x else "0" for x in self.detection_history[-10:]])
                
                self.get_logger().info(
                    f'📊 统计信息 - 总帧数:{self.frame_count}, 检测率:{detection_rate:.1f}%, '\
                    f'发布率:{publish_rate:.1f}%, 状态:{self.detection_state}, 历史:{history_summary}'
                )
                
                if self.detection_state == "TRACKING":
                    self.get_logger().info(f'🎯 跟踪中: 已跟踪{self.tracking_frames}帧')
            
            # 发布调试图像
            if self.debug_mode:
                try:
                    self.publish_debug_image_improved(cv_image, detection_result, msg.header)
                except Exception as debug_e:
                    self.get_logger().error(f'发布调试图像失败 (帧#{self.frame_count}): {str(debug_e)}')
                
        except Exception as e:
            self.get_logger().error(f'处理图像时出错 (帧#{self.frame_count}): {str(e)}', throttle_duration_sec=1.0)
            import traceback
            self.get_logger().error(f'详细错误信息: {traceback.format_exc()}')
    
    def detect_yellow_light_improved(self, image: np.ndarray) -> Optional[Tuple[float, Tuple[int, int, int]]]:
        """
        精确的黄色标志物检测（整合test_yellow_marker_detector_accurate.py的逻辑）
        
        Args:
            image: 输入图像 (BGR格式)
            
        Returns:
            (距离, (x, y, radius)) 或 None
        """
        try:
            height, width = image.shape[:2]
            
            # 限制ROI区域到天花板部分
            roi_top = int(height * self.roi_top_ratio)
            roi_bottom = int(height * self.roi_bottom_ratio)
            roi_image = image[roi_top:roi_bottom, :]
            
            # HSV转换
            hsv = cv2.cvtColor(roi_image, cv2.COLOR_BGR2HSV)
            
            # 创建精确的黄色掩码
            mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
            yellow_pixels = np.sum(mask > 0)
            
            # 如果黄色像素太少，直接返回
            if yellow_pixels < self.min_contour_area:
                return None
            
            # 如果黄色像素太多，可能是误检黄线
            if yellow_pixels > self.max_contour_area:
                return None
            
            # 形态学操作 - 去噪声
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # 使用精确的霍夫圆检测参数
            circles = cv2.HoughCircles(
                mask,
                cv2.HOUGH_GRADIENT,
                dp=1,
                minDist=self.hough_min_dist,
                param1=self.hough_param1,
                param2=self.hough_param2, 
                minRadius=self.circle_min_radius,
                maxRadius=self.circle_max_radius
            )
            
            if circles is not None:
                circles = np.uint16(np.around(circles))
                
                # 只返回最佳的一个圆（最大的或最圆的）
                best_circle = None
                best_score = 0
                
                for circle in circles[0, :]:
                    x, y, radius = circle
                    
                    # 验证圆形的有效性（包括位置过滤）
                    roi_height = roi_image.shape[0]
                    if not self.validate_circle_accurate(mask, x, y, radius, roi_height):
                        continue
                    
                    # 计算圆形质量分数（半径 + 圆形度）
                    circularity = self.calculate_circularity(mask, x, y, radius)
                    score = radius * circularity  # 半径越大、越圆的分数越高
                    
                    if score > best_score:
                        best_score = score
                        best_circle = circle
                
                if best_circle is not None:
                    x, y, radius = best_circle
                    distance = self.calculate_distance_from_circle(radius)
                    return (distance, (x, y, radius))
            
            return None
            
        except Exception as e:
            self.get_logger().error(f'精确检测过程出错: {str(e)}')
            return None
    
    def validate_circle_accurate(self, mask: np.ndarray, x: int, y: int, radius: int, roi_height: int) -> bool:
        """验证圆形的有效性 - 包括位置和填充度验证（整合精确检测逻辑）"""
        try:
            # 添加Y坐标位置过滤：只接受ROI前50%区域内的圆形
            max_valid_y = int(roi_height * self.valid_circle_y_ratio)
            if y > max_valid_y:
                self.get_logger().info(f'圆形Y坐标{y}超过有效区域{max_valid_y}，跳过')
                return False
            
            # 检查圆形是否在掩码范围内
            h, w = mask.shape
            x, y, radius = int(x), int(y), int(radius)  # 确保为整数类型
            if x - radius < 0 or x + radius >= w or y - radius < 0 or y + radius >= h:
                return False
            
            # 检查圆形区域内的像素填充度
            circle_mask = np.zeros_like(mask)
            cv2.circle(circle_mask, (x, y), radius, 255, -1)
            
            intersection = cv2.bitwise_and(mask, circle_mask)
            circle_area = np.pi * radius * radius
            filled_area = np.sum(intersection > 0)
            fill_ratio = filled_area / circle_area
            
            # 要求至少50%的填充度
            return fill_ratio > 0.5
            
        except:
            return False
    
    def calculate_circularity(self, mask: np.ndarray, x: int, y: int, radius: int) -> float:
        """计算圆形的圆形度（0-1，1为完美圆形）"""
        try:
            # 创建圆形模板
            circle_mask = np.zeros_like(mask)
            cv2.circle(circle_mask, (x, y), radius, 255, -1)
            
            # 计算重叠度
            intersection = cv2.bitwise_and(mask, circle_mask)
            union = cv2.bitwise_or(mask, circle_mask)
            
            intersection_area = np.sum(intersection > 0)
            union_area = np.sum(union > 0)
            
            if union_area == 0:
                return 0.0
            
            # IoU作为圆形度指标
            return intersection_area / union_area
            
        except:
            return 0.0
    
    def update_detection_state(self, detection_result) -> Optional[str]:
        """更新检测状态机并返回应该发布的距离"""
        # 如果已进入PUBLISHING状态，不再更新状态
        if self.detection_state == "PUBLISHING":
            return None
            
        # 添加当前检测结果到历史
        self.detection_history.append(detection_result is not None)
        
        # 保持历史记录长度
        if len(self.detection_history) > self.history_max_length:
            self.detection_history.pop(0)
        
        # 如果检测到黄灯，更新最后检测距离
        if detection_result is not None:
            distance, circle = detection_result
            self.last_detection_distance = distance
            
            # 状态转换：SEARCHING -> TRACKING
            if self.detection_state == "SEARCHING":
                self.detection_state = "TRACKING"
                self.get_logger().info(f'🎯 开始跟踪黄色标志物 (SEARCHING -> TRACKING)')
                return None  # 跟踪状态不发布距离
        
        # 检查是否连续多帧未检测到（仅在TRACKING状态下）
        if self.detection_state == "TRACKING" and len(self.detection_history) >= self.disappeared_threshold:
            recent_detections = self.detection_history[-self.disappeared_threshold:]
            if not any(recent_detections):  # 连续未检测到
                self.is_marker_disappeared = True
                self.detection_state = "DISAPPEARED"
                self.get_logger().info(f'📍 黄色标志物已消失! (TRACKING -> DISAPPEARED, 连续{self.disappeared_threshold}帧未检测到)')
        
        # 返回应该发布的距离（仅在DISAPPEARED状态首次发布）
        if self.detection_state == "DISAPPEARED" and not self.first_publish_done:
            return self.disappear_distance
        else:
            return None
    
    def analyze_shape(self, contour: np.ndarray) -> Optional[Tuple[str, float]]:
        """
        分析轮廓形状（圆形或方形）
        
        Args:
            contour: 输入轮廓
            
        Returns:
            (形状类型, 形状分数) 或 None
        """
        try:
            # 检测圆形
            (x, y), radius = cv2.minEnclosingCircle(contour)
            if self.circle_min_radius <= radius <= self.circle_max_radius:
                # 计算轮廓面积与外接圆面积的比值
                contour_area = cv2.contourArea(contour)
                circle_area = np.pi * radius * radius
                circle_ratio = contour_area / circle_area if circle_area > 0 else 0
                
                # 如果比值接近1，说明是圆形
                if circle_ratio > 0.7:  # 70%以上相似度认为是圆形
                    return ("圆形", circle_ratio)
            
            # 检测方形（矩形）
            # 使用多边形逼近
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # 如果逼近后的多边形有4个顶点，可能是方形
            if len(approx) == 4:
                # 计算边界框
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = float(w) / h if h > 0 else 0
                
                # 检查长宽比是否接近1（正方形）或在合理范围内（矩形）
                if 0.5 <= aspect_ratio <= 2.0:  # 长宽比在合理范围内
                    # 计算轮廓面积与边界框面积的比值
                    contour_area = cv2.contourArea(contour)
                    bbox_area = w * h
                    rect_ratio = contour_area / bbox_area if bbox_area > 0 else 0
                    
                    # 如果填充度足够高，认为是方形
                    if rect_ratio > 0.8:  # 80%以上填充度
                        return ("方形", rect_ratio)
            
            return None
            
        except Exception as e:
            self.get_logger().error(f'分析轮廓形状时出错: {str(e)}')
            return None
    
    def estimate_distance(self, object_area_pixels: float) -> float:
        """
        根据物体像素面积估算距离
        
        Args:
            object_area_pixels: 物体在图像中的像素面积
            
        Returns:
            估算距离（米）
        """
        try:
            # 使用面积反比关系：distance = sqrt((reference_area * reference_distance²) / current_area)
            if object_area_pixels <= 0:
                return 10.0  # 默认远距离
            
            distance_squared = (self.reference_area_pixels * self.reference_distance * self.reference_distance) / object_area_pixels
            distance = np.sqrt(distance_squared)
            
            # 限制距离范围
            distance = max(0.5, min(distance, 10.0))  # 0.5米到10米
            
            return distance
            
        except Exception as e:
            self.get_logger().error(f'估算距离时出错: {str(e)}')
            return 5.0  # 默认距离
    
    def set_yellow_light_mode(self, enable: bool):
        """设置黄灯检测模式"""
        self.yellow_light_detection_mode = enable
        mode_text = "启用" if enable else "禁用"
        self.get_logger().info(f'黄灯检测模式已{mode_text}')
    
    def detect_yellow_light(self, image: np.ndarray) -> Optional[Tuple[float, bool, str]]:
        """
        专门检测黄灯（圆形，直径20cm，RGB: 255,255,0）
        
        Args:
            image: 输入图像 (BGR格式)
            
        Returns:
            (距离(米), 是否应该停止, 检测状态) 或 None
        """
        try:
            height, width = image.shape[:2]
            
            # 黄灯通常在赛道中间稍高的位置
            roi_top = int(height * 0.3)      # 中上部分
            roi_bottom = int(height * 0.7)   # 中下部分
            roi_image = image[roi_top:roi_bottom, :]
            
            # 转换为HSV色彩空间
            hsv = cv2.cvtColor(roi_image, cv2.COLOR_BGR2HSV)
            
            # 创建黄色掩码（更严格的黄色范围）
            # RGB(255,255,0) 对应的HSV约为 (60, 255, 255)
            lower_yellow_light = np.array([20, 100, 100])
            upper_yellow_light = np.array([30, 255, 255])
            mask = cv2.inRange(hsv, lower_yellow_light, upper_yellow_light)
            
            # 形态学操作
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # 使用霍夫圆变换检测圆形
            circles = cv2.HoughCircles(
                mask,
                cv2.HOUGH_GRADIENT,
                dp=1,
                minDist=50,
                param1=50,
                param2=30,
                minRadius=self.yellow_light_min_radius,
                maxRadius=self.yellow_light_max_radius
            )
            
            if circles is not None:
                circles = np.uint16(np.around(circles))
                
                # 找到最大的圆形（假设是黄灯）
                largest_circle = None
                max_radius = 0
                
                for circle in circles[0, :]:
                    x, y, radius = circle
                    if radius > max_radius:
                        max_radius = radius
                        largest_circle = circle
                
                if largest_circle is not None:
                    x, y, radius = largest_circle
                    
                    # 计算距离（基于圆形直径）
                    distance = self.calculate_distance_from_circle(radius)
                    
                    # 判断是否应该停止
                    should_stop = self.should_stop_at_yellow_light(distance)
                    
                    # 判断检测状态
                    if distance < (self.yellow_light_target_distance - self.yellow_light_distance_tolerance):
                        status = "too_close"  # 太近了
                    elif should_stop:
                        status = "stop_zone"  # 停止区域
                    else:
                        status = "approaching" # 接近中
                    
                    self.get_logger().info(f'检测到黄灯: 位置({x}, {y}), 半径{radius}, 距离{distance:.2f}m, 状态{status}')
                    
                    return (distance, should_stop, status)
            
            return None
            
        except Exception as e:
            self.get_logger().error(f'检测黄灯时出错: {str(e)}')
            return None
    
    def calculate_distance_from_circle(self, pixel_radius: int) -> float:
        """
        根据圆形像素半径计算实际距离（专门用于黄灯）
        
        Args:
            pixel_radius: 像素半径
            
        Returns:
            实际距离（米）
        """
        # 使用相似三角形原理: distance = (real_diameter * focal_length) / (2 * pixel_radius)
        # 假设相机焦距为600像素（需要根据实际相机调整）
        focal_length_pixels = 600
        
        if pixel_radius > 0:
            distance = (self.yellow_light_real_diameter * focal_length_pixels) / (2 * pixel_radius)
            return max(0.1, min(distance, 10.0))  # 限制在0.1-10米范围内
        return 10.0  # 默认远距离
    
    def should_stop_at_yellow_light(self, distance: float) -> bool:
        """
        判断是否应该在黄灯前停止
        
        Args:
            distance: 与黄灯的距离（米）
            
        Returns:
            是否应该停止
        """
        # 在目标距离±容忍度范围内停止
        target_min = self.yellow_light_target_distance - self.yellow_light_distance_tolerance
        target_max = self.yellow_light_target_distance + self.yellow_light_distance_tolerance
        
        return target_min <= distance <= target_max
    
    def timer_publish_callback(self):
        """定时器回调函数：在PUBLISHING状态定时发布距离"""
        if self.detection_state == "PUBLISHING":
            self.publish_distance(self.disappear_distance)
            self.publish_count += 1
            
            # 减少日志频率
            if self.publish_count % 10 == 1:  # 每10次发布记录一次
                self.get_logger().info(f'⏰ 定时发布距离: {self.disappear_distance} (第{self.publish_count}次)')
    
    def publish_distance(self, distance: str):
        """发布距离信息（字符串格式）"""
        msg = String()
        msg.data = distance
        self.distance_publisher.publish(msg)
    
    def publish_debug_image_improved(self, image: np.ndarray, detection_result, header):
        """发布改进的调试图像（带状态机信息）"""
        if not self.debug_mode:
            return
        
        try:
            debug_image = image.copy()
            height, width = image.shape[:2]
            
            # 绘制ROI区域
            roi_top = int(height * self.roi_top_ratio)
            roi_bottom = int(height * self.roi_bottom_ratio)
            cv2.rectangle(debug_image, (0, roi_top), (width, roi_bottom), (255, 255, 0), 2)
            cv2.putText(debug_image, "ROI (Upper Half)", (10, roi_top + 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            
            # 在ROI区域检测黄色
            roi_image = image[roi_top:roi_bottom, :]
            hsv = cv2.cvtColor(roi_image, cv2.COLOR_BGR2HSV)
            
            # 使用精确的黄色范围（与检测函数一致）
            mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
            
            # 形态学操作
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # 在原图上绘制黄色区域
            yellow_overlay = debug_image.copy()
            yellow_overlay[roi_top:roi_bottom, :][mask > 0] = [0, 255, 255]
            debug_image = cv2.addWeighted(debug_image, 0.7, yellow_overlay, 0.3, 0)
            
            # 绘制检测结果
            if detection_result:
                distance, circle = detection_result
                x, y, radius = circle
                y_adjusted = y + roi_top
                
                # 绘制圆形
                cv2.circle(debug_image, (x, y_adjusted), radius, (0, 0, 255), 3)
                cv2.circle(debug_image, (x, y_adjusted), 5, (0, 0, 255), -1)
                
                # 显示信息
                cv2.putText(debug_image, f"Distance: {distance:.2f}m", 
                           (x + 10, y_adjusted - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            # 显示状态信息
            state_colors = {
                "SEARCHING": (255, 255, 0),    # 黄色
                "TRACKING": (0, 255, 0),       # 绿色
                "DISAPPEARED": (0, 0, 255),    # 红色
                "PUBLISHING": (255, 0, 255)    # 紫色
            }
            
            cv2.putText(debug_image, f"State: {self.detection_state}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, state_colors.get(self.detection_state, (255, 255, 255)), 2)
            
            # 显示检测历史和统计信息
            history_text = f"Frame#{self.frame_count} | History: " + "".join(["1" if x else "0" for x in self.detection_history[-10:]])
            if self.detection_count > 0:
                detection_rate = (self.detection_count / self.frame_count) * 100
                history_text += f" | Det:{self.detection_count}/{self.frame_count}({detection_rate:.1f}%)"
            if self.publish_count > 0:
                history_text += f" | Pub:{self.publish_count}"
            if self.detection_state == "TRACKING":
                history_text += f" | Tracking:{self.tracking_frames}frames"
            cv2.putText(debug_image, history_text, (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # 显示发布状态
            if self.detection_state == "PUBLISHING":
                publish_text = f"TIMER Publishing: {self.disappear_distance} (Count: {self.publish_count})"
                color = (255, 0, 255)  # 紫色
                cv2.putText(debug_image, publish_text, (10, height - 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            elif self.detection_state == "DISAPPEARED":
                publish_text = f"FIRST Publishing: {self.disappear_distance}"
                color = (0, 255, 0)  # 绿色
                cv2.putText(debug_image, publish_text, (10, height - 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            elif self.detection_state == "TRACKING":
                cv2.putText(debug_image, "DETECTED - NOT Publishing", (10, height - 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)  # 黄色
            else:
                cv2.putText(debug_image, f"No Publish ({self.detection_state})", (10, height - 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (128, 128, 128), 2)
            
            # 显示消失状态
            if self.detection_state == "PUBLISHING":
                cv2.putText(debug_image, "DETECTION STOPPED - TIMER PUBLISHING!", (10, height - 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2)  # 紫色
            elif self.is_marker_disappeared:
                cv2.putText(debug_image, "MARKER DISAPPEARED - FIRST PUBLISHING!", (10, height - 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)  # 绿色
            
            # 发布调试图像
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            debug_msg.header = header
            self.debug_image_publisher.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f'发布改进调试图像时出错: {str(e)}')

    def publish_debug_image(self, image: np.ndarray, detection_result: Optional[Tuple[float, float, str]], header):
        """发布调试图像"""
        if not self.debug_mode:
            return
        
        try:
            debug_image = image.copy()
            height, width = image.shape[:2]
            
            # 绘制ROI区域
            roi_top = int(height * self.roi_top_ratio)
            roi_bottom = int(height * self.roi_bottom_ratio)
            cv2.rectangle(debug_image, (0, roi_top), (width, roi_bottom), (255, 255, 0), 2)
            cv2.putText(debug_image, "ROI (High Area)", (10, roi_top + 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            
            # 在ROI区域检测黄色
            roi_image = image[roi_top:roi_bottom, :]
            hsv = cv2.cvtColor(roi_image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
            
            # 形态学操作
            kernel = np.ones((3, 3), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # 在原图上绘制黄色区域
            yellow_overlay = debug_image.copy()
            yellow_overlay[roi_top:roi_bottom, :][mask > 0] = [0, 255, 255]
            debug_image = cv2.addWeighted(debug_image, 0.7, yellow_overlay, 0.3, 0)
            
            # 查找并绘制轮廓
            contours_result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours_result) == 3:
                _, contours, hierarchy  = contours_result
            else:
                contours, hierarchy = contours_result
            
            # 过滤有效轮廓
            valid_contours = []
            for c in contours:
                if c is not None and len(c) > 0 and c.dtype in [np.int32, np.float32]:
                    valid_contours.append(c)
            contours = valid_contours
            
            for i, contour in enumerate(contours):
                area = cv2.contourArea(contour)
                if self.min_contour_area <= area <= self.max_contour_area:
                    # 调整轮廓坐标到原图
                    adjusted_contour = contour.copy()
                    adjusted_contour[:, :, 1] += roi_top
                    
                    # 绘制轮廓
                    cv2.drawContours(debug_image, [adjusted_contour], -1, (0, 255, 255), 2)
                    
                    # 分析形状
                    shape_result = self.analyze_shape(contour)
                    shape_info = shape_result[0] if shape_result else "未知"
                    
                    # 计算质心
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"]) + roi_top
                        cv2.circle(debug_image, (cx, cy), 5, (0, 0, 255), -1)
                        
                        # 显示信息
                        cv2.putText(debug_image, f"#{i+1} {shape_info} A:{area:.0f}", 
                                   (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # 显示检测结果
            if detection_result:
                distance, confidence, shape_type = detection_result
                result_text = f"DETECTED: {shape_type} {distance:.2f}m (conf: {confidence:.2f})"
                color = (0, 255, 0)
            else:
                result_text = "NO MARKER DETECTED"
                color = (0, 0, 255)
            
            cv2.putText(debug_image, result_text, (10, height - 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
            
            # 显示参数信息
            param_text = f"Area:{self.min_contour_area}-{self.max_contour_area}, Shapes:Circle+Rectangle"
            cv2.putText(debug_image, param_text, (10, height - 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # 发布调试图像
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            debug_msg.header = header
            self.debug_image_publisher.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f'发布调试图像时出错: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    yellow_marker_detector = YellowMarkerDetector()
    
    try:
        rclpy.spin(yellow_marker_detector)
    except KeyboardInterrupt:
        pass
    finally:
        yellow_marker_detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()