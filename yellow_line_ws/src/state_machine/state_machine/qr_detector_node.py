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
        self.get_logger().info(f'参数配置: image_topic={self.image_topic}, debug_mode={self.debug_mode}')
        self.get_logger().info(f'订阅图像话题: {self.image_topic}')
        self.get_logger().info(f'发布二维码信息话题: /qr_detector/qr_info')
        self.get_logger().info(f'发布调试图像话题: /qr_detector/debug_image')
        self.get_logger().info('节点初始化完成，等待图像消息...')
        self.get_logger().info('注意: 此版本仅发布二维码内容，不计算距离信息')
        
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
            
            # 检查是否是预期的二维码类型（A-1/A-2/B-1/B-2）
            expected_codes = ['A-1', 'A-2', 'B-1', 'B-2']
            if qr_data in expected_codes:
                # 直接发布二维码内容，不包含距离信息
                self.publish_qr_info(qr_data)
                
                self.get_logger().info(f'检测到二维码: {qr_data}')
            else:
                # 尝试文字识别 - 检查是否包含预期的文字内容
                for expected_code in expected_codes:
                    if expected_code in qr_data:
                        # 提取匹配的文字内容
                        self.publish_qr_info(expected_code)
                        self.get_logger().info(f'通过文字识别检测到: {expected_code} (原始内容: {qr_data})')
                        return
                
                self.get_logger().warn(f'检测到未知内容: {qr_data}')
        except Exception as e:
            self.get_logger().error(f'处理二维码时出错: {str(e)}')
    
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
    
    def publish_qr_info(self, qr_data):
        """发布二维码信息 - 仅发布二维码内容，不包含距离"""
        info_msg = String()
        info_msg.data = qr_data  # 直接发布二维码内容，格式如: "A-1", "A-2", "B-1", "B-2"
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
                        
                        # 显示二维码内容（不显示距离）
                        text = f"{qr_data}"
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