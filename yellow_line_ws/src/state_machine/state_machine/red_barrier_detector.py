#!/usr/bin/env python3

"""
红色限高杆识别模块 - 识别RGB图片中的红色限高杆并发布距离信息

功能描述：
    - 检测图像中的红色区域
    - 识别红色限高杆（垂直的红色物体）
    - 估算距离并发布到ROS话题
    
使用方法：
    ros2 run state_machine red_barrier_detector
    
发布话题：
    /red_barrier_detector/distance - 检测到限高杆时的距离信息
    /red_barrier_detector/debug_image - 调试图像（可选）
    
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


class RedBarrierDetector(Node):
    def __init__(self):
        super().__init__('red_barrier_detector')
        
        # CV桥接器
        self.bridge = CvBridge()
        
        # 红色检测参数 - HSV色彩空间
        # 红色在HSV中分为两个范围
        self.lower_red1 = np.array([0, 50, 50])      # 红色下限1
        self.upper_red1 = np.array([10, 255, 255])   # 红色上限1
        self.lower_red2 = np.array([160, 50, 50])    # 红色下限2
        self.upper_red2 = np.array([180, 255, 255])  # 红色上限2
        
        # 检测参数
        self.min_contour_area = 800   # 最小轮廓面积
        self.min_aspect_ratio = 1.5   # 最小高宽比（限高杆应该是竖直的）
        self.max_aspect_ratio = 8.0   # 最大高宽比
        self.roi_top_ratio = 0.2      # ROI顶部比例
        self.roi_bottom_ratio = 0.8   # ROI底部比例
        
        # 距离估算参数（基于物体大小）
        self.reference_height_pixels = 200  # 参考高度像素（1米距离时的像素高度）
        self.reference_distance = 1.0       # 参考距离（米）
        
        # 调试模式
        self.debug_mode = self.declare_parameter('debug_mode', True).value
        self.detection_threshold = 0.5  # 检测置信度阈值
        
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
        
        self.get_logger().info('红色限高杆识别节点已启动')
        self.get_logger().info(f'参数配置: debug_mode={self.debug_mode}')
        self.get_logger().info('订阅话题: /image_rgb')
        self.get_logger().info('发布话题: /red_barrier_detector/distance')
        if self.debug_mode:
            self.get_logger().info('发布调试话题: /red_barrier_detector/debug_image')
        self.get_logger().info('HSV红色范围1: {} - {}'.format(self.lower_red1, self.upper_red1))
        self.get_logger().info('HSV红色范围2: {} - {}'.format(self.lower_red2, self.upper_red2))
    
    def image_callback(self, msg):
        """图像回调函数"""
        try:
            # 转换图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 检测红色限高杆
            detection_result = self.detect_red_barrier(cv_image)
            
            # 发布检测结果
            if detection_result:
                distance, confidence = detection_result
                self.publish_distance(distance)
                self.get_logger().info(f'检测到红色限高杆，距离: {distance:.2f}米, 置信度: {confidence:.2f}')
            
            # 发布调试图像
            if self.debug_mode:
                self.publish_debug_image(cv_image, detection_result, msg.header)
                
        except Exception as e:
            self.get_logger().error(f'处理图像时出错: {str(e)}')
    
    def detect_red_barrier(self, image: np.ndarray) -> Optional[Tuple[float, float]]:
        """
        检测红色限高杆
        
        Args:
            image: 输入图像 (BGR格式)
            
        Returns:
            (距离, 置信度) 或 None
        """
        try:
            height, width = image.shape[:2]
            
            # 提取ROI区域（上半部分到中下部分）
            roi_top = int(height * self.roi_top_ratio)
            roi_bottom = int(height * self.roi_bottom_ratio)
            roi_image = image[roi_top:roi_bottom, :]
            
            # 转换为HSV色彩空间
            hsv = cv2.cvtColor(roi_image, cv2.COLOR_BGR2HSV)
            
            # 创建红色掩码（两个范围）
            mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
            mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
            mask = cv2.bitwise_or(mask1, mask2)
            
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
            
            # 分析轮廓，找到最可能是限高杆的轮廓
            best_contour = None
            best_score = 0
            
            for contour in contours:
                # 检查轮廓面积
                area = cv2.contourArea(contour)
                if area < self.min_contour_area:
                    continue
                
                # 检查边界框的高宽比
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = h / w if w > 0 else 0
                
                if aspect_ratio < self.min_aspect_ratio or aspect_ratio > self.max_aspect_ratio:
                    continue
                
                # 计算轮廓的填充度（实际面积/边界框面积）
                bbox_area = w * h
                fill_ratio = area / bbox_area if bbox_area > 0 else 0
                
                # 计算分数（综合考虑面积、高宽比、填充度）
                area_score = min(area / 2000, 1.0)  # 面积分数，最大1.0
                ratio_score = 1.0 - abs(aspect_ratio - 3.0) / 3.0  # 高宽比分数，理想比例为3:1
                fill_score = fill_ratio  # 填充度分数
                
                total_score = (area_score + ratio_score + fill_score) / 3.0
                
                if total_score > best_score and total_score > self.detection_threshold:
                    best_score = total_score
                    best_contour = contour
            
            if best_contour is None:
                return None
            
            # 估算距离
            x, y, w, h = cv2.boundingRect(best_contour)
            distance = self.estimate_distance(h)
            
            return (distance, best_score)
            
        except Exception as e:
            self.get_logger().error(f'检测红色限高杆时出错: {str(e)}')
            return None
    
    def estimate_distance(self, object_height_pixels: int) -> float:
        """
        根据物体像素高度估算距离
        
        Args:
            object_height_pixels: 物体在图像中的像素高度
            
        Returns:
            估算距离（米）
        """
        try:
            # 使用简单的反比关系：distance = (reference_height * reference_distance) / current_height
            if object_height_pixels <= 0:
                return 10.0  # 默认远距离
            
            distance = (self.reference_height_pixels * self.reference_distance) / object_height_pixels
            
            # 限制距离范围
            distance = max(0.5, min(distance, 10.0))  # 0.5米到10米
            
            return distance
            
        except Exception as e:
            self.get_logger().error(f'估算距离时出错: {str(e)}')
            return 5.0  # 默认距离
    
    def publish_distance(self, distance: float):
        """发布距离信息"""
        msg = String()
        msg.data = f"{distance:.2f}"
        self.distance_publisher.publish(msg)
    
    def publish_debug_image(self, image: np.ndarray, detection_result: Optional[Tuple[float, float]], header):
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
            cv2.putText(debug_image, "ROI", (10, roi_top + 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            
            # 在ROI区域检测红色
            roi_image = image[roi_top:roi_bottom, :]
            hsv = cv2.cvtColor(roi_image, cv2.COLOR_BGR2HSV)
            
            # 创建红色掩码
            mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
            mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
            mask = cv2.bitwise_or(mask1, mask2)
            
            # 形态学操作
            kernel = np.ones((3, 3), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # 在原图上绘制红色区域
            red_overlay = debug_image.copy()
            red_overlay[roi_top:roi_bottom, :][mask > 0] = [0, 0, 255]
            debug_image = cv2.addWeighted(debug_image, 0.7, red_overlay, 0.3, 0)
            
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
                if area >= self.min_contour_area:
                    # 调整轮廓坐标到原图
                    adjusted_contour = contour.copy()
                    adjusted_contour[:, :, 1] += roi_top
                    
                    # 绘制轮廓
                    cv2.drawContours(debug_image, [adjusted_contour], -1, (0, 255, 255), 2)
                    
                    # 绘制边界框
                    x, y, w, h = cv2.boundingRect(contour)
                    y += roi_top  # 调整到原图坐标
                    cv2.rectangle(debug_image, (x, y), (x + w, y + h), (255, 0, 255), 2)
                    
                    # 显示轮廓信息
                    aspect_ratio = h / w if w > 0 else 0
                    cv2.putText(debug_image, f"#{i+1} A:{area:.0f} R:{aspect_ratio:.1f}", 
                               (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # 显示检测结果
            if detection_result:
                distance, confidence = detection_result
                result_text = f"DETECTED: {distance:.2f}m (conf: {confidence:.2f})"
                color = (0, 255, 0)
            else:
                result_text = "NO BARRIER DETECTED"
                color = (0, 0, 255)
            
            cv2.putText(debug_image, result_text, (10, height - 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
            
            # 显示参数信息
            param_text = f"MinArea:{self.min_contour_area}, AspectRatio:{self.min_aspect_ratio}-{self.max_aspect_ratio}"
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