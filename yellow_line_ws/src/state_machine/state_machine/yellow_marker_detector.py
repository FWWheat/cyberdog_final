#!/usr/bin/env python3

"""
黄色标志物识别模块 - 识别RGB图片中的黄色圆形/方形标志物并发布距离信息

功能描述：
    - 检测图像中的黄色区域
    - 识别黄色圆形或方形标志物（挂在高处）
    - 估算距离并发布到ROS话题
    
使用方法：
    ros2 run state_machine yellow_marker_detector
    
发布话题：
    /yellow_marker_detector/distance - 检测到标志物时的距离信息
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
        
        # 黄色检测参数 - HSV色彩空间
        self.lower_yellow = np.array([15, 80, 80])    # 黄色下限
        self.upper_yellow = np.array([35, 255, 255])  # 黄色上限
        
        # 检测参数
        self.min_contour_area = 500    # 最小轮廓面积
        self.max_contour_area = 10000  # 最大轮廓面积（避免检测到黄线）
        self.roi_top_ratio = 0.1       # ROI顶部比例（高处标志物）
        self.roi_bottom_ratio = 0.6    # ROI底部比例
        
        # 形状检测参数
        self.circle_min_radius = 10    # 圆形最小半径
        self.circle_max_radius = 100   # 圆形最大半径
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
        
        self.get_logger().info('黄色标志物识别节点已启动')
        self.get_logger().info(f'参数配置: debug_mode={self.debug_mode}')
        self.get_logger().info('订阅话题: /image_rgb')
        self.get_logger().info('发布话题: /yellow_marker_detector/distance')
        if self.debug_mode:
            self.get_logger().info('发布调试话题: /yellow_marker_detector/debug_image')
        self.get_logger().info('HSV黄色范围: {} - {}'.format(self.lower_yellow, self.upper_yellow))
    
    def image_callback(self, msg):
        """图像回调函数"""
        try:
            # 转换图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 检测黄色标志物
            detection_result = self.detect_yellow_marker(cv_image)
            
            # 发布检测结果
            if detection_result:
                distance, confidence, shape_type = detection_result
                self.publish_distance(distance)
                self.get_logger().info(f'检测到黄色{shape_type}标志物，距离: {distance:.2f}米, 置信度: {confidence:.2f}')
            
            # 发布调试图像
            if self.debug_mode:
                self.publish_debug_image(cv_image, detection_result, msg.header)
                
        except Exception as e:
            self.get_logger().error(f'处理图像时出错: {str(e)}')
    
    def detect_yellow_marker(self, image: np.ndarray) -> Optional[Tuple[float, float, str]]:
        """
        检测黄色标志物
        
        Args:
            image: 输入图像 (BGR格式)
            
        Returns:
            (距离, 置信度, 形状类型) 或 None
        """
        try:
            height, width = image.shape[:2]
            
            # 提取ROI区域（上半部分，标志物挂在高处）
            roi_top = int(height * self.roi_top_ratio)
            roi_bottom = int(height * self.roi_bottom_ratio)
            roi_image = image[roi_top:roi_bottom, :]
            
            # 转换为HSV色彩空间
            hsv = cv2.cvtColor(roi_image, cv2.COLOR_BGR2HSV)
            
            # 创建黄色掩码
            mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
            
            # 形态学操作
            kernel = np.ones((3, 3), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # 查找轮廓
            contours_result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours_result) == 3:
                contours, hierarchy, _ = contours_result
            else:
                contours, hierarchy = contours_result
            
            # 过滤有效轮廓
            valid_contours = []
            for c in contours:
                if c is not None and len(c) > 0 and c.dtype in [np.int32, np.float32]:
                    valid_contours.append(c)
            contours = valid_contours
            
            if not contours:
                return None
            
            # 分析轮廓，找到最可能是标志物的轮廓
            best_contour = None
            best_score = 0
            best_shape = "unknown"
            
            for contour in contours:
                # 检查轮廓面积
                area = cv2.contourArea(contour)
                if area < self.min_contour_area or area > self.max_contour_area:
                    continue
                
                # 分析形状
                shape_result = self.analyze_shape(contour)
                if shape_result is None:
                    continue
                
                shape_type, shape_score = shape_result
                
                # 计算轮廓的紧凑性
                perimeter = cv2.arcLength(contour, True)
                if perimeter == 0:
                    continue
                compactness = 4 * np.pi * area / (perimeter * perimeter)
                
                # 计算总分数
                area_score = min(area / 1000, 1.0)  # 面积分数
                compact_score = compactness  # 紧凑性分数
                
                total_score = (area_score + shape_score + compact_score) / 3.0
                
                if total_score > best_score and total_score > self.detection_threshold:
                    best_score = total_score
                    best_contour = contour
                    best_shape = shape_type
            
            if best_contour is None:
                return None
            
            # 估算距离
            area = cv2.contourArea(best_contour)
            distance = self.estimate_distance(area)
            
            return (distance, best_score, best_shape)
            
        except Exception as e:
            self.get_logger().error(f'检测黄色标志物时出错: {str(e)}')
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
    
    def publish_distance(self, distance: float):
        """发布距离信息"""
        msg = String()
        msg.data = f"{distance:.2f}"
        self.distance_publisher.publish(msg)
    
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
                contours, hierarchy, _ = contours_result
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