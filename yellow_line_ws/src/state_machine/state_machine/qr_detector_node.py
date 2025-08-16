#!/usr/bin/env python3
# ros2 run state_machine qr_detector_node
# ros2 run state_machine qr_detector_node --ros-args -p image_topic:=/mi_desktop_48_b0_2d_7b_03_d0/image

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
         # 配置QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
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
        self.min_confidence = 1  # 需要连续检测2次才确认，降低要求
        self.publish_interval = 0.05  # 每0.05秒发布一次，提高响应速度
        
        # 调试统计变量
        self.frame_count = 0                      # 总帧数
        self.detection_count = 0                  # 检测到的帧数
        self.publish_count = 0                    # 发布的帧数
        self.last_detection_time = None           # 最后检测时间
        self.state_change_time = None             # 状态改变时间
        self.ocr_attempt_count = 0                # OCR尝试次数
        self.qr_decode_count = 0                  # QR解码成功次数
        self.confidence_history = []              # 置信度历史
        
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
            self.publish_count += 1
            self.publish_qr_info(self.last_detected_qr)
            
            # 每100次发布记录一次
            if self.publish_count % 100 == 0:
                self.get_logger().info(f'持续发布QR码: "{self.last_detected_qr}" (第{self.publish_count}次发布)')
        
    def image_callback(self, msg):
        try:
            # 更新帧计数
            self.frame_count += 1
            current_time = self.get_clock().now().nanoseconds / 1e9
            
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 检测二维码
            qr_codes = self.detect_qr_codes(cv_image)
            
            # 更新统计信息
            if qr_codes:
                self.detection_count += 1
                self.last_detection_time = current_time
                
                # 记录检测方法
                for qr_code in qr_codes:
                    if hasattr(qr_code, 'data'):
                        try:
                            qr_data = qr_code.data.decode('utf-8')
                            if any(code in qr_data for code in ['A-1', 'A-2', 'B-1', 'B-2']):
                                self.qr_decode_count += 1
                                self.get_logger().info(
                                    f'QR码解码成功 - 内容:"{qr_data}", 帧#{self.frame_count}'
                                )
                        except:
                            pass
            
            # 更新状态记忆
            old_qr = self.last_detected_qr
            old_confidence = self.detection_confidence
            self.update_detection_state(qr_codes)
            
            # 检查状态变化
            if old_qr != self.last_detected_qr or old_confidence != self.detection_confidence:
                self.state_change_time = current_time
                if self.last_detected_qr != old_qr:
                    self.get_logger().info(f'🔄 QR码状态变更: "{old_qr}" -> "{self.last_detected_qr}" (帧#{self.frame_count})')
            
            # 记录置信度历史
            self.confidence_history.append(self.detection_confidence)
            if len(self.confidence_history) > 20:
                self.confidence_history.pop(0)
            
            # 处理检测到的二维码
            for qr_code in qr_codes:
                self.process_qr_code(qr_code, cv_image, msg.header)
            
            # 每50帧输出统计信息
            if self.frame_count % 50 == 0:
                detection_rate = (self.detection_count / self.frame_count) * 100
                publish_rate = (self.publish_count / self.frame_count) * 100 if self.publish_count > 0 else 0
                avg_confidence = sum(self.confidence_history) / len(self.confidence_history) if self.confidence_history else 0
                
                self.get_logger().info(
                    f'📊 统计信息 - 总帧数:{self.frame_count}, 检测率:{detection_rate:.1f}%, '
                    f'发布率:{publish_rate:.1f}%, QR解码:{self.qr_decode_count}, OCR尝试:{self.ocr_attempt_count}, '
                    f'当前QR:"{self.last_detected_qr}", 置信度:{self.detection_confidence}/{self.min_confidence}, '
                    f'平均置信度:{avg_confidence:.1f}'
                )
            
            # 发布调试图像（无论是否检测到二维码都发布）
            self.publish_debug_image(cv_image, qr_codes, msg.header)
            
        except Exception as e:
            self.get_logger().error(f'处理图像时出错 (帧#{self.frame_count}): {str(e)}', throttle_duration_sec=1.0)
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
        self.ocr_attempt_count += 1
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
        
        methods = {}
        methods['gray'] = gray
        
        # 添加更多预处理方法提高识别率
        # 二值化
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        methods['binary'] = binary
        
        # 形态学操作去噪
        kernel = np.ones((2,2), np.uint8)
        cleaned = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        methods['cleaned'] = cleaned
        
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
        h, w = image.shape[:2]
        
        # 改进的ROI设置 - 扩大搜索范围，覆盖更多可能的文字位置
        roi_configs = [
            # 原始配置（稍作调整）
            {
                'y1': int(h * 0.05),  # 从更上方开始
                'y2': int(h * 0.30),  # 扩大到30%
                'x1': int(w * 0.35),  # 减少左边距
                'x2': int(w * 0.65),  # 减少右边距
                'name': 'extended_roi'
            },
            # 专门针对文字位置的ROI
            {
                'y1': int(h * 0.08),
                'y2': int(h * 0.25),
                'x1': int(w * 0.40),
                'x2': int(w * 0.60),
                'name': 'focused_roi'
            },
            # 更大范围的ROI
            {
                'y1': int(h * 0.0),
                'y2': int(h * 0.35),
                'x1': int(w * 0.30),
                'x2': int(w * 0.70),
                'name': 'large_roi'
            },

            {
            'y1': int(h * 0.35),  # 从高度 35% 开始
            'y2': int(h * 0.65),  # 到高度 65%
            'x1': int(w * 0.35),  # 左边裁掉 35%
            'x2': int(w * 0.65),  # 右边裁掉 35%
            'name': 'center_roi'
        }
        ]
        
        # OCR配置 - 添加更多配置选项
        configs = [
            '--oem 3 --psm 6',  # 单个文本块
            '--oem 3 --psm 7',  # 单行文本
            '--oem 3 --psm 8',  # 单个单词
            '--oem 3 --psm 13', # 原始行，不做假设
        ]
        
        # 尝试所有ROI配置
        for roi_config in roi_configs:
            # 提取ROI区域
            roi_image = image[roi_config['y1']:roi_config['y2'], 
                             roi_config['x1']:roi_config['x2']]
            
            if roi_image.size == 0:
                continue
                
            # 获取所有预处理版本
            processed_images = self.preprocess_for_large_text(roi_image)
            
            # 测试所有组合
            for process_name, processed_img in processed_images.items():
                for config in configs:
                    try:
                        text = pytesseract.image_to_string(processed_img, config=config).strip()
                        self.get_logger().info(f"ROI: {roi_config['name']}, 预处理: {process_name}, 配置: {config}")
                        self.get_logger().info(f"识别文本: '{text}'")
                        
                        detected_code = self.detect_code_in_text(text)
                        if detected_code:
                            self.get_logger().info(f"检测到代码: {detected_code}")
                            return detected_code

                    except Exception as e:
                        self.get_logger().warn(f"OCR错误: {e}")
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
            
            # 显示统计信息
            stats_text = f"Frame#{self.frame_count} | QR:\"{self.last_detected_qr}\" Conf:{self.detection_confidence}/{self.min_confidence}"
            if self.detection_count > 0:
                detection_rate = (self.detection_count / self.frame_count) * 100
                stats_text += f" | Det:{self.detection_count}/{self.frame_count}({detection_rate:.1f}%)"
            if self.publish_count > 0:
                stats_text += f" | Pub:{self.publish_count}"
            if self.ocr_attempt_count > 0:
                stats_text += f" | OCR:{self.ocr_attempt_count}"
            cv2.putText(debug_image, stats_text, (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # 显示发布状态
            if self.last_detected_qr is not None:
                publish_text = f"CONTINUOUSLY Publishing: \"{self.last_detected_qr}\""
                color = (0, 255, 0)  # 绿色
                cv2.putText(debug_image, publish_text, (10, len(debug_image) - 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            else:
                cv2.putText(debug_image, "No QR Code in Memory", (10, len(debug_image) - 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (128, 128, 128), 2)
            
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