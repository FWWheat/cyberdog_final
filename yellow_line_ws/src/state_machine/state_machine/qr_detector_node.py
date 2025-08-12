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
import pytesseract
import re

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
        
        # 添加状态记忆
        self.last_detected_qr = None
        self.detection_confidence = 0
        self.min_confidence = 2  # 需要连续检测3次才确认
        self.publish_interval = 0.1  # 每0.5秒发布一次
        
        # 创建定时器，持续发布检测到的二维码
        self.publish_timer = self.create_timer(
            self.publish_interval, 
            self.publish_timer_callback
        )
        
        self.get_logger().info('二维码检测节点已启动')
        self.get_logger().info(f'参数配置: image_topic={self.image_topic}, debug_mode={self.debug_mode}')
        self.get_logger().info(f'订阅图像话题: {self.image_topic}')
        self.get_logger().info(f'发布二维码信息话题: /qr_detector/qr_info')
        self.get_logger().info(f'发布调试图像话题: /qr_detector/debug_image')
        self.get_logger().info(f'持续发布间隔: {self.publish_interval}秒')
        self.get_logger().info('节点初始化完成，等待图像消息...')
        
    def publish_timer_callback(self):
        """定时器回调，持续发布检测到的二维码信息"""
        if self.last_detected_qr is not None:
            self.publish_qr_info(self.last_detected_qr)
        
    def image_callback(self, msg):
        try:
            # self.get_logger().info(f'收到图像消息: 宽度={msg.width}, 高度={msg.height}, 编码={msg.encoding}')
            
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # self.get_logger().info(f'图像转换成功: OpenCV图像形状={cv_image.shape}')
            
            # 检测二维码
            qr_codes = self.detect_qr_codes(cv_image)
            # self.get_logger().info(f'二维码检测完成: 检测到 {len(qr_codes)} 个二维码')
            
            # 更新状态记忆
            self.update_detection_state(qr_codes)
            
            # 处理检测到的二维码
            for qr_code in qr_codes:
                self.process_qr_code(qr_code, cv_image, msg.header)
            
            # 发布调试图像（无论是否检测到二维码都发布）
            self.publish_debug_image(cv_image, qr_codes, msg.header)
            
        except Exception as e:
            self.get_logger().error(f'处理图像时出错: {str(e)}')
            import traceback
            self.get_logger().error(f'详细错误信息: {traceback.format_exc()}')
    
    def update_detection_state(self, qr_codes):
        """更新检测状态记忆"""
        if qr_codes:
            # 获取检测到的二维码内容
            detected_qr = None
            for qr_code in qr_codes:
                try:
                    qr_data = qr_code.data.decode('utf-8')
                    expected_codes = ['A-1', 'A-2', 'B-1', 'B-2']
                    if qr_data in expected_codes:
                        detected_qr = qr_data
                        break
                    else:
                        # 尝试从文字中提取
                        for expected_code in expected_codes:
                            if expected_code in qr_data:
                                detected_qr = expected_code
                                break
                        if detected_qr:
                            break
                except:
                    continue
            
            if detected_qr:
                if detected_qr == self.last_detected_qr:
                    # 相同的检测结果，增加信心度
                    self.detection_confidence = min(self.detection_confidence + 1, self.min_confidence)
                else:
                    # 新的检测结果，重置信心度
                    self.last_detected_qr = detected_qr
                    self.detection_confidence = 1
                    self.get_logger().info(f'检测到新的二维码: {detected_qr}')
            else:
                # 没有有效检测结果，降低信心度但保持记忆
                self.detection_confidence = max(self.detection_confidence - 1, 0)
        else:
            # 没有检测到任何二维码，降低信心度
            self.detection_confidence = max(self.detection_confidence - 1, 0)
            
        # 如果信心度太低，清除记忆
        if self.detection_confidence <= 0:
            if self.last_detected_qr is not None:
                self.get_logger().info(f'清除二维码记忆: {self.last_detected_qr}')
                self.last_detected_qr = None
    
    def detect_qr_codes(self, image):
        """检测图像中的二维码和文字"""
        # 方法1: 使用pyzbar检测二维码
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        qr_codes = pyzbar.decode(gray)
        
        # 如果pyzbar检测到二维码，直接返回
        if qr_codes:
            return qr_codes
        
        # 方法2: 使用OCR检测文字
        ocr_results = self.detect_text_with_ocr(image)
        
        # 检查OCR结果是否为None
        if ocr_results is None:
            return []
        
        # OCR结果处理 - detect_text_with_ocr返回的是单个字符串或None
        if isinstance(ocr_results, str):
            # 创建Point类
            class Point:
                def __init__(self, x, y):
                    self.x = x
                    self.y = y
            
            # 创建默认的边界框（图像中心区域）
            h, w = image.shape[:2]
            roi_y1 = int(h * 0.1)
            roi_y2 = int(h * 0.25)
            roi_margin_w = int(w * 0.4)
            roi_x1 = roi_margin_w
            roi_x2 = w - roi_margin_w
            
            points = [Point(roi_x1, roi_y1), Point(roi_x2, roi_y1), 
                     Point(roi_x2, roi_y2), Point(roi_x1, roi_y2)]
            
            # 创建一个类似pyzbar.Decoded对象的结构
            class OCRResult:
                def __init__(self, data, polygon):
                    self.data = data.encode('utf-8')
                    self.polygon = polygon
                
            return [OCRResult(ocr_results, points)]
        
        return []
    
    def preprocess_for_large_text(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
        # 多种预处理方法
        methods = {}
        
        # 方法1: 简单灰度
        methods['gray'] = gray
        

        return methods
    
    def detect_code_in_text(self, text):
        """从OCR文本中检测A-1/A-2/B-1/B-2"""
        if not text:
            return None
        
        text_upper = text.upper()
        
        # 精确匹配
        expected_codes = ['A-1', 'A-2', 'B-1', 'B-2']
        for code in expected_codes:
            if code in text_upper:
                return code
        
        # 模糊匹配
        # 检查A-X模式
        has_a_like = any(char in text_upper for char in ['A', 'Α'])
        has_1_like = any(char in text for char in ['1'])
        has_2_like = any(char in text for char in ['2'])
        
        if has_a_like:
            if has_2_like or '2' in text:
                return 'A-2'
            elif has_1_like or '1' in text:
                return 'A-1'
        
        # 检查B-X模式
        has_b_like = any(char in text_upper for char in ['B', 'Β'])
        
        if has_b_like:
            if has_2_like or '2' in text:
                return 'B-2'
            elif has_1_like or '1' in text:
                return 'B-1'
        
        return None

    def detect_text_with_ocr(self, image):
        """使用OCR检测文字"""
        # 先划定感兴趣区域(ROI) - 取图像顶部30%，中间25%部分
        h, w = image.shape[:2]
        
        # 垂直方向：取顶部30%
        roi_y1 = int(h * 0.1)
        roi_y2 = int(h * 0.25)
        
        # 水平方向：取中间25%（左右各37.5%边距）
        roi_margin_w = int(w * 0.4)  # 水平边距37.5%
        roi_x1 = roi_margin_w
        roi_x2 = w - roi_margin_w
        
        # 提取ROI区域
        roi_image = image[roi_y1:roi_y2, roi_x1:roi_x2]
        
        # 获取所有预处理版本
        processed_images = self.preprocess_for_large_text(roi_image)
        
        # OCR配置
        configs = [
            '--oem 3 --psm 6',# 单个文本块
        ]
        
        # 测试所有组合
        for process_name, processed_img in processed_images.items():
            for config in configs:
                try:
                    # 使用image_to_string获取完整文本
                    text = pytesseract.image_to_string(processed_img, config=config).strip()
                    print(text)
                    detected_code = self.detect_code_in_text(text)
                    if detected_code:
                        return detected_code

                except Exception:
                    continue
        
        return None

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