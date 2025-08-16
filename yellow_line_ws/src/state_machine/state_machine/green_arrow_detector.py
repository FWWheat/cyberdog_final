#!/usr/bin/env python3

"""
绿色箭头识别模块 - 识别RGB图片中的绿色箭头方向

功能描述：
    - 检测图像中的绿色区域
    - 识别绿色区域的箭头方向（左、右）
    - 发布箭头方向信息到ROS话题
    
使用方法：
    ros2 run state_machine green_arrow_detector
    
发布话题：
    /green_arrow_detector/direction - 箭头方向 ("left" 或 "right")
    /green_arrow_detector/debug_image - 调试图像（可选）
    
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
from typing import Optional, Tuple


class GreenArrowDetector(Node):
    def __init__(self):
        super().__init__('green_arrow_detector')
        
        # CV桥接器
        self.bridge = CvBridge()
        
        # 绿色检测参数 - HSV色彩空间
        self.lower_green = np.array([35, 50, 50])   # 绿色下限
        self.upper_green = np.array([85, 255, 255]) # 绿色上限
        
        # 检测参数
        self.min_contour_area = 1000  # 最小轮廓面积
        
        # 调试统计变量
        self.frame_count = 0                      # 总帧数
        self.detection_count = 0                  # 检测到的帧数
        self.left_arrow_count = 0                 # 左箭头检测次数
        self.right_arrow_count = 0                # 右箭头检测次数
        self.last_detection_time = None           # 最后检测时间
        self.consecutive_same_direction = 0       # 连续相同方向计数
        self.direction_history = []               # 方向历史记录
        
        # 调试模式
        self.debug_mode = self.declare_parameter('debug_mode', True).value
        self.last_direction = None  # 记录上次检测的方向，避免重复发布
        
        # 配置QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # 订阅RGB图像
        self.image_subscription = self.create_subscription(
            Image,
            '/mi_desktop_48_b0_2d_7b_03_d0/image',
            self.image_callback,
            qos_profile
        )
        
        # 发布箭头方向
        self.direction_publisher = self.create_publisher(
            String,
            '/green_arrow_detector/direction',
            10
        )
        
        # 发布调试图像
        if self.debug_mode:
            self.debug_image_publisher = self.create_publisher(
                Image,
                '/green_arrow_detector/debug_image',
                qos_profile
            )
        
        self.get_logger().info('绿色箭头识别节点已启动 (增强调试版)')
        self.get_logger().info(f'参数配置: debug_mode={self.debug_mode}')
        self.get_logger().info('发布话题: /green_arrow_detector/direction')
        if self.debug_mode:
            self.get_logger().info('发布调试话题: /green_arrow_detector/debug_image')
        self.get_logger().info('HSV绿色范围: {} - {}'.format(self.lower_green, self.upper_green))
        self.get_logger().info('调试功能: 帧计数, 方向统计, 连续检测, 历史记录')
    
    def image_callback(self, msg):
        """图像回调函数"""
        try:
            # 更新帧计数
            self.frame_count += 1
            current_time = self.get_clock().now().nanoseconds / 1e9
            
            # 转换图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 检测箭头方向
            direction = self.detect_arrow_direction(cv_image)
            
            # 更新统计信息
            if direction:
                self.detection_count += 1
                self.last_detection_time = current_time
                
                # 统计方向
                if direction == "left":
                    self.left_arrow_count += 1
                elif direction == "right":
                    self.right_arrow_count += 1
                
                # 检查连续相同方向
                if self.direction_history and self.direction_history[-1] == direction:
                    self.consecutive_same_direction += 1
                else:
                    self.consecutive_same_direction = 1
                
                # 更新方向历史
                self.direction_history.append(direction)
                if len(self.direction_history) > 20:
                    self.direction_history.pop(0)
                
                self.get_logger().info(
                    f'检测到绿色箭头方向: {direction} - 帧#{self.frame_count}, '
                    f'连续相同方向:{self.consecutive_same_direction}次'
                )
            
            # 发布方向信息
            if direction:
                # 避免重复发布相同方向（可选择启用）
                if direction != self.last_direction:
                    self.publish_direction(direction)
                    if self.last_direction is not None:
                        self.get_logger().info(f'🔄 箭头方向变更: {self.last_direction} -> {direction}')
                    self.last_direction = direction
                elif self.consecutive_same_direction == 1:  # 仅在首次检测时发布
                    self.publish_direction(direction)
            
            # 每100帧输出统计信息
            if self.frame_count % 100 == 0:
                detection_rate = (self.detection_count / self.frame_count) * 100
                direction_summary = "".join(self.direction_history[-10:]) if self.direction_history else "无"
                
                self.get_logger().info(
                    f'📊 统计信息 - 总帧数:{self.frame_count}, 检测率:{detection_rate:.1f}%, '
                    f'左箭头:{self.left_arrow_count}, 右箭头:{self.right_arrow_count}, '
                    f'当前方向:"{self.last_direction}", 近期历史:{direction_summary}'
                )
            
            # 发布调试图像
            if self.debug_mode:
                self.publish_debug_image_enhanced(cv_image, direction, msg.header)
                
        except Exception as e:
            self.get_logger().error(f'处理图像时出错 (帧#{self.frame_count}): {str(e)}', throttle_duration_sec=1.0)
    
    def detect_arrow_direction(self, image: np.ndarray) -> Optional[str]:
        """
        检测绿色箭头方向
        
        Args:
            image: 输入图像 (BGR格式)
            
        Returns:
            箭头方向: 'left', 'right' 或 None
        """
        try:
            height, width = image.shape[:2]
            
            # 提取ROI区域
            roi = {
                'y1': int(height * 0),  # 从高度 35% 开始
                'y2': int(height * 1),  # 到高度 65%
                'x1': int(width * 0),   # 左边裁掉 35%
                'x2': int(width * 1),   # 右边裁掉 35%
            }
            roi_image = image[roi['y1']:roi['y2'], roi['x1']:roi['x2']]
            roi_width = roi_image.shape[1]
            
            # 转换为HSV色彩空间
            hsv = cv2.cvtColor(roi_image, cv2.COLOR_BGR2HSV)
            
            # 创建绿色掩码
            mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
            
            # 形态学操作
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # 查找轮廓
            contours_result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours_result) == 3:
                _, contours, hierarchy = contours_result
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
            
            # 找到最大轮廓
            largest_contour = max(contours, key=cv2.contourArea)
            
            # 检查轮廓面积
            if cv2.contourArea(largest_contour) < self.min_contour_area:
                return None
            
            # 分析箭头方向
            direction = self.analyze_arrow_direction(largest_contour, roi_width)
            
            return direction
            
        except Exception as e:
            self.get_logger().error(f'检测箭头方向时出错: {str(e)}')
            return None
    
    def analyze_arrow_direction(self, contour: np.ndarray, roi_width: int) -> Optional[str]:
        """
        分析轮廓的箭头方向
        
        Args:
            contour: 输入轮廓
            roi_width: ROI区域宽度
            
        Returns:
            箭头方向: 'left', 'right' 或 None
        """
        try:
            # 计算轮廓的质心
            M = cv2.moments(contour)
            if M["m00"] == 0:
                return None
            
            cx = int(M["m10"] / M["m00"])  # 质心X坐标
            
            # 计算轮廓的边界框
            x, y, w, h = cv2.boundingRect(contour)
            
            # 分析轮廓形状特征来判断箭头方向
            # 方法1：分析质心相对于边界框的位置
            bbox_center_x = x + w / 2
            centroid_offset = cx - bbox_center_x
            
            # 方法2：分析轮廓的凸包缺陷
            hull = cv2.convexHull(contour, returnPoints=False)
            defects = cv2.convexityDefects(contour, hull)
            
            direction_score = 0  # 正数表示右箭头，负数表示左箭头
            
            # 基于质心偏移判断
            if abs(centroid_offset) > w * 0.1:  # 至少偏移10%
                if centroid_offset > 0:
                    direction_score += 1  # 质心偏右，可能是右箭头
                else:
                    direction_score -= 1  # 质心偏左，可能是左箭头
            
            # 基于轮廓最左和最右点分析
            leftmost = tuple(contour[contour[:, :, 0].argmin()][0])
            rightmost = tuple(contour[contour[:, :, 0].argmax()][0])
            
            # 分析边缘形状
            left_edge_y = leftmost[1]
            right_edge_y = rightmost[1]
            center_y = y + h / 2
            
            # 如果左边缘更尖锐（Y坐标更接近中心），可能是左箭头
            if abs(left_edge_y - center_y) < abs(right_edge_y - center_y):
                direction_score -= 1
            else:
                direction_score += 1
            
            # 根据分数判断方向
            if direction_score > 0:
                return 'right'
            elif direction_score < 0:
                return 'left'
            else:
                # 如果分数为0，使用简单的质心位置判断
                roi_center = roi_width / 2
                if cx > roi_center:
                    return 'right'
                else:
                    return 'left'
                    
        except Exception as e:
            self.get_logger().error(f'分析箭头方向时出错: {str(e)}')
            return None
    
    def publish_direction(self, direction: str):
        """发布箭头方向"""
        msg = String()
        msg.data = direction
        self.direction_publisher.publish(msg)
    
    def publish_debug_image_enhanced(self, image: np.ndarray, direction: Optional[str], header):
        """发布增强的调试图像（带统计信息）"""
        if not self.debug_mode:
            return
        
        try:
            debug_image = image.copy()
            height, width = image.shape[:2]
            
            # 绘制ROI区域
            roi = {
                'y1': int(height * 0),  # 从高度 0% 开始
                'y2': int(height * 1),  # 到高度 100%
                'x1': int(width * 0),   # 左边 0%
                'x2': int(width * 1),   # 右边 100%
            }
            cv2.rectangle(debug_image, (roi['x1'], roi['y1']), (roi['x2'], roi['y2']), (255, 255, 0), 2)
            cv2.putText(debug_image, "ROI (Full Frame)", (roi['x1'] + 10, roi['y1'] + 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            
            # 在ROI区域检测绿色
            roi_image = image[roi['y1']:roi['y2'], roi['x1']:roi['x2']]
            hsv = cv2.cvtColor(roi_image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
            
            # 形态学操作
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # 在原图上绘制绿色区域
            green_overlay = debug_image.copy()
            green_overlay[roi['y1']:roi['y2'], roi['x1']:roi['x2']][mask > 0] = [0, 255, 0]
            debug_image = cv2.addWeighted(debug_image, 0.7, green_overlay, 0.3, 0)
            
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
            
            largest_contour = None
            contour_info = ""
            
            if contours:
                # 找到最大轮廓
                largest_contour = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest_contour) >= self.min_contour_area:
                    # 调整轮廓坐标到原图
                    adjusted_contour = largest_contour.copy()
                    adjusted_contour[:, :, 0] += roi['x1']
                    adjusted_contour[:, :, 1] += roi['y1']
                    
                    # 绘制轮廓
                    cv2.drawContours(debug_image, [adjusted_contour], -1, (0, 255, 255), 3)
                    
                    # 绘制质心
                    M = cv2.moments(largest_contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"]) + roi['x1']
                        cy = int(M["m01"] / M["m00"]) + roi['y1']
                        cv2.circle(debug_image, (cx, cy), 8, (0, 0, 255), -1)
                        
                        # 分析轮廓特征
                        x, y, w, h = cv2.boundingRect(largest_contour)
                        area = cv2.contourArea(largest_contour)
                        contour_info = f"Area:{area:.0f}, Size:{w}x{h}, Center:({cx},{cy})"
            
            # 显示检测结果
            if direction:
                direction_text = f"Arrow: {direction.upper()}"
                color = (0, 255, 0)
                
                # 绘制箭头指向
                center_x, center_y = width // 2, height // 2
                if direction == "left":
                    cv2.arrowedLine(debug_image, (center_x + 50, center_y), (center_x - 50, center_y), (0, 255, 0), 8, tipLength=0.3)
                elif direction == "right":
                    cv2.arrowedLine(debug_image, (center_x - 50, center_y), (center_x + 50, center_y), (0, 255, 0), 8, tipLength=0.3)
            else:
                direction_text = "No Arrow Detected"
                color = (0, 0, 255)
            
            cv2.putText(debug_image, direction_text, (10, height - 120), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 3)
            
            # 显示统计信息
            stats_text = f"Frame#{self.frame_count} | Dir:\"{self.last_direction}\" Cons:{self.consecutive_same_direction}"
            if self.detection_count > 0:
                detection_rate = (self.detection_count / self.frame_count) * 100
                stats_text += f" | Det:{self.detection_count}/{self.frame_count}({detection_rate:.1f}%)"
            stats_text += f" | L:{self.left_arrow_count} R:{self.right_arrow_count}"
            cv2.putText(debug_image, stats_text, (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # 显示轮廓信息
            if contour_info:
                cv2.putText(debug_image, contour_info, (10, 90), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # 显示方向历史
            if self.direction_history:
                history_text = "History: " + "".join([d[0].upper() for d in self.direction_history[-10:]])
                cv2.putText(debug_image, history_text, (10, height - 90), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # 显示参数信息
            param_text = f"HSV: {self.lower_green}-{self.upper_green}, Min Area: {self.min_contour_area}"
            cv2.putText(debug_image, param_text, (10, height - 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # 显示发布状态
            if direction:
                if direction != self.last_direction or self.consecutive_same_direction == 1:
                    publish_status = "PUBLISHING: Direction Change or First Detection"
                    status_color = (0, 255, 0)
                else:
                    publish_status = "NOT PUBLISHING: Same Direction (Filtered)"
                    status_color = (0, 255, 255)
                cv2.putText(debug_image, publish_status, (10, height - 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)
            
            # 发布调试图像
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            debug_msg.header = header
            self.debug_image_publisher.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f'发布增强调试图像时出错: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    green_arrow_detector = GreenArrowDetector()
    
    try:
        rclpy.spin(green_arrow_detector)
    except KeyboardInterrupt:
        pass
    finally:
        green_arrow_detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()