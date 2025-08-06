#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        
        # 创建CV桥接器
        self.bridge = CvBridge()
        
        # 添加消息计数器
        self.message_count = 0
        
        # 订阅图像话题
        self.subscription = self.create_subscription(
            Image,
            '/mi_desktop_48_b0_2d_7b_03_d0/image',
            self.image_callback,
            10
        )
        
        self.get_logger().info('相机图像查看器已启动')

    def image_callback(self, msg):
        try:
            # 增加消息计数
            self.message_count += 1
            self.get_logger().info(f'接收到第 {self.message_count} 个图像消息')
            
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 显示图像
            cv2.imshow('Camera Image', cv_image)
            cv2.waitKey(1)
                
        except Exception as e:
            self.get_logger().error(f'处理图像时出错: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    viewer = CameraViewer()
    
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