#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
import numpy as np
from cv_bridge import CvBridge
from pyzbar import pyzbar
import math

class QRDetectorNode(Node):
    def __init__(self):
        super().__init__('qr_detector_node')
        
        # 创建CV桥接器
        self.bridge = CvBridge()
        
        # 二维码的实际大小（米）- 可通过参数配置
        self.qr_real_size = self.declare_parameter('qr_real_size', 0.20).value  # 20cm
        
        # 相机内参（需要根据实际相机调整）- 可通过参数配置
        self.focal_length = self.declare_parameter('focal_length', 1144.0).value  # 焦距
        
        # 图像话题名称 - 可通过参数配置
        self.image_topic = self.declare_parameter(
            'image_topic', '/image_rgb').value
        
        
        # 调试模式 - 可通过参数配置
        self.debug_mode = self.declare_parameter('debug_mode', True).value
        
        # 配置QoS设置
        # BEST_EFFORT: 调试图像不需要100%可靠传递，如果偶尔丢失一帧也没关系
        # KEEP_LAST: 只保留最新的10帧调试图像
        # depth=10: 可以缓冲10帧图像
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 订阅图像话题
        self.image_subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile
        )
        
        # 发布二维码信息
        self.qr_info_publisher = self.create_publisher(
            String,
            '/qr_detector/qr_info',
            10
        )
        
        # 发布检测结果图像（用于调试）
        self.debug_image_publisher = self.create_publisher(
            Image,
            '/qr_detector/debug_image',
            qos_profile
        )
        
        self.get_logger().info('二维码检测节点已启动')
        self.get_logger().info(f'参数配置: qr_real_size={self.qr_real_size}m, focal_length={self.focal_length}, image_topic={self.image_topic}, debug_mode={self.debug_mode}')
        self.get_logger().info(f'订阅图像话题: {self.image_topic}')
        self.get_logger().info(f'发布二维码信息话题: /qr_detector/qr_info')
        self.get_logger().info(f'发布调试图像话题: /qr_detector/debug_image')
        self.get_logger().info('节点初始化完成，等待图像消息...')
        
    def image_callback(self, msg):
        try:
            # self.get_logger().info(f'收到图像消息: 宽度={msg.width}, 高度={msg.height}, 编码={msg.encoding}')
            
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # self.get_logger().info(f'图像转换成功: OpenCV图像形状={cv_image.shape}')
            
            # 检测二维码
            qr_codes = self.detect_qr_codes(cv_image)
            # self.get_logger().info(f'二维码检测完成: 检测到 {len(qr_codes)} 个二维码')
            
            # 处理检测到的二维码
            for qr_code in qr_codes:
                self.process_qr_code(qr_code, cv_image, msg.header)
            
            # 发布调试图像（无论是否检测到二维码都发布）
            self.publish_debug_image(cv_image, qr_codes, msg.header)
            
        except Exception as e:
            self.get_logger().error(f'处理图像时出错: {str(e)}')
            import traceback
            self.get_logger().error(f'详细错误信息: {traceback.format_exc()}')
    
    def detect_qr_codes(self, image):
        """检测图像中的二维码"""
        # 使用pyzbar检测二维码
        # 转换为灰度图像
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # 使用pyzbar检测二维码
        qr_codes = pyzbar.decode(gray)
        
        return qr_codes
    
    def process_qr_code(self, qr_code, image, header):
        """处理检测到的二维码"""
        try:
            # 获取二维码数据
            qr_data = qr_code.data.decode('utf-8')
            
            # 检查是否是预期的二维码类型
            expected_codes = ['A-1', 'A-2', 'B-1', 'B-2']
            if qr_data in expected_codes:
                # 计算距离
                distance = self.calculate_distance(qr_code, image)
                
                # 发布二维码信息
                self.publish_qr_info(qr_data, distance)
                
                self.get_logger().info(f'检测到二维码: {qr_data}, 距离: {distance:.2f}m')
            else:
                self.get_logger().warn(f'检测到未知二维码: {qr_data}')
        except Exception as e:
            self.get_logger().error(f'处理二维码时出错: {str(e)}')
    
    def calculate_distance(self, qr_code, image):
        """根据二维码大小计算距离"""
        try:
            # 获取二维码的边界框
            points = qr_code.polygon
            if len(points) >= 4:
                # 计算二维码在图像中的像素大小
                width_pixels = self.calculate_width(points)
                
                # 防止除零错误
                if width_pixels <= 0:
                    self.get_logger().warn('二维码像素宽度为0，无法计算距离')
                    return 0.0
                
                # 使用相似三角形原理计算距离
                # distance = (real_size * focal_length) / pixel_size
                distance = (self.qr_real_size * self.focal_length) / width_pixels
                
                # 添加合理性检查
                if distance < 0.1 or distance > 10.0:  # 距离范围0.1-10米
                    self.get_logger().warn(f'计算的距离 {distance:.2f}m 可能不准确')
                
                return distance
            else:
                self.get_logger().warn('二维码边界框点数不足，无法计算距离')
                return 0.0
        except Exception as e:
            self.get_logger().error(f'计算距离时出错: {str(e)}')
            return 0.0
    
    def calculate_width(self, points):
        """计算二维码的像素宽度"""
        try:
            if len(points) >= 4:
                # 计算边界框的宽度和高度
                x_coords = [point.x for point in points]
                y_coords = [point.y for point in points]
                width = max(x_coords) - min(x_coords)
                height = max(y_coords) - min(y_coords)
                
                # 返回较大的尺寸作为二维码的"宽度"
                return max(width, height)
            else:
                self.get_logger().warn(f'边界框点数不足: {len(points)}')
                return 0.0
        except Exception as e:
            self.get_logger().error(f'计算宽度时出错: {str(e)}')
            return 0.0
    
    def get_qr_center(self, qr_code):
        """获取二维码中心点"""
        try:
            points = qr_code.polygon
            if len(points) >= 4:
                x_coords = [point.x for point in points]
                y_coords = [point.y for point in points]
                center_x = sum(x_coords) / len(x_coords)
                center_y = sum(y_coords) / len(y_coords)
                return (center_x, center_y)
            else:
                self.get_logger().warn(f'无法计算中心点，边界框点数不足: {len(points)}')
                return (0, 0)
        except Exception as e:
            self.get_logger().error(f'计算中心点时出错: {str(e)}')
            return (0, 0)
    
    def publish_qr_info(self, qr_data, distance):
        """发布二维码信息"""
        info_msg = String()
        info_msg.data = f"QR:{qr_data},Distance:{distance:.2f}"
        self.qr_info_publisher.publish(info_msg)
    

    
    def publish_debug_image(self, image, qr_codes, header):
        """发布调试图像"""
        if not self.debug_mode:
            self.get_logger().warn('调试模式已关闭，跳过调试图像发布')
            return
            
        try:
            # self.get_logger().info(f'开始处理调试图像: 图像形状={image.shape}, 二维码数量={len(qr_codes)}')
            
            debug_image = image.copy()
            
            # 在图像上绘制检测到的二维码
            for qr_code in qr_codes:
                points = qr_code.polygon
                if len(points) >= 4:
                    # 绘制边界框
                    pts = np.array([[point.x, point.y] for point in points], np.int32)
                    pts = pts.reshape((-1, 1, 2))
                    cv2.polylines(debug_image, [pts], True, (0, 255, 0), 2)
                    
                    # 添加文本标签
                    try:
                        qr_data = qr_code.data.decode('utf-8')
                        center = self.get_qr_center(qr_code)
                        distance = self.calculate_distance(qr_code, image)
                        
                        # 显示二维码内容和距离
                        text = f"{qr_data} ({distance:.2f}m)"
                        cv2.putText(debug_image, text, 
                                   (int(center[0]), int(center[1]) - 10),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    except Exception as e:
                        self.get_logger().warn(f'绘制二维码标签时出错: {str(e)}')
            
            # 添加调试信息到图像上
            if len(qr_codes) == 0:
                # 如果没有检测到二维码，显示提示信息
                cv2.putText(debug_image, "No QR Code Detected", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            else:
                # 显示检测到的二维码数量
                cv2.putText(debug_image, f"Detected {len(qr_codes)} QR Code(s)", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 发布调试图像
            # self.get_logger().info('准备发布调试图像...')
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            debug_msg.header = header
            self.debug_image_publisher.publish(debug_msg)
            # self.get_logger().info('调试图像发布成功')
            
        except Exception as e:
            self.get_logger().error(f'发布调试图像时出错: {str(e)}')
            import traceback
            self.get_logger().error(f'详细错误信息: {traceback.format_exc()}')

def main(args=None):
    rclpy.init(args=args)
    
    qr_detector_node = QRDetectorNode()
    
    try:
        rclpy.spin(qr_detector_node)
    except KeyboardInterrupt:
        pass
    finally:
        qr_detector_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 