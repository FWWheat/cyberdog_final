#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
import numpy as np

class MultiCameraViewer(Node):
    def __init__(self):
        super().__init__('multi_camera_viewer')
        
        # 创建CV桥接器
        self.bridge = CvBridge()
        
        # 存储每个话题的图像和消息计数
        self.images = {}
        self.message_counts = {}
        
        # 定义要订阅的图像话题列表
        self.topics = [
            # '/mi_desktop_48_b0_2d_7b_03_d0/image',
            # '/image_rgb',
            # '/image_right',
            # '/imge_left',
            '/qr_detector/debug_image',
            '/green_arrow_detector/debug_image',
            '/red_barrier_detector/debug_image',
            '/yellow_marker_detector/debug_image'
        ]
        
        # 配置QoS设置 - 与detector节点保持一致
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 为每个话题创建订阅器
        self._subscriptions = []
        for topic in self.topics:
            subscription = self.create_subscription(
                Image,
                topic,
                lambda msg, topic_name=topic: self.image_callback(msg, topic_name),
                qos_profile
            )
            self._subscriptions.append(subscription)
            self.message_counts[topic] = 0
            self.images[topic] = None
            
        self.get_logger().info(f'多相机图像查看器已启动，订阅 {len(self.topics)} 个话题')
        for topic in self.topics:
            self.get_logger().info(f'订阅话题: {topic}')

    def image_callback(self, msg, topic_name):
        try:
            # 增加该话题的消息计数
            self.message_counts[topic_name] += 1
            self.get_logger().info(f'{topic_name}: 接收到第 {self.message_counts[topic_name]} 个图像消息')
            
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 存储图像
            self.images[topic_name] = cv_image
            
            # 显示图像
            self.display_images()
                
        except Exception as e:
            self.get_logger().error(f'处理图像 {topic_name} 时出错: {str(e)}')

    def display_images(self):
        """显示所有接收到的图像"""
        for topic_name, image in self.images.items():
            if image is not None:
                # 使用话题名称作为窗口标题
                window_name = f'Camera: {topic_name.split("/")[-2] if "/" in topic_name else topic_name}'
                
                # 调整图像大小以便更好地显示多个窗口
                height, width = image.shape[:2]
                if width > 640:
                    scale = 640.0 / width
                    new_width = int(width * scale)
                    new_height = int(height * scale)
                    image = cv2.resize(image, (new_width, new_height))
                
                cv2.imshow(window_name, image)
        
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    
    viewer = MultiCameraViewer()
    
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        viewer.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()