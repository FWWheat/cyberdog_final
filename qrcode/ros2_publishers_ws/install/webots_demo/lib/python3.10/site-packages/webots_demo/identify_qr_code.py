import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image,  PointCloud2
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener #坐标变换
import numpy as np
import cv2
import numpy as np
from PIL import Image
from pyzbar.pyzbar import decode

class QrcodePosePublisher(Node):
    def __init__(self):
        super().__init__('qr_code_pose_publisher')

        # 定义机器人数量
        self.number_robots = 6
        
        # 创建一个字典来存储每个发布器
        self.publishers_dict = {}
        
        # 为每个机器人创建一个发布器
        for i in range(self.number_robots):
            namespace = f'robot_{i}'
            topic_name = f'{namespace}/current_pose'  # 定义每个机器人的话题名
            self.publishers_dict[namespace] = self.create_publisher(PoseStamped, topic_name, 10)


        # 定时器：周期性地发布图像和位置信息
        self.timer = self.create_timer(0.1, self.publish_pose)  # 每 100ms 发布一次

        # 二维码位置
        self.pose=(0,0)
        # 二维码编码内容
        self.decode=None

        self.get_logger().info("Qr code pose publisher node has been started.")


    def publish_pose(self):
        image_path='../img/test6.png'
        try:
            pil_img = Image.open(image_path)  # 获取PIL图像
            cv_img = cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)  # 转换为OpenCV格式

            width, height = pil_img.size  # 获取图像尺寸
            centerpoint = width / 2, height / 2  # 计算中心点
            cp_size = 10  # 中心点大小
            matches = decode(pil_img)  # 扫描传入图像，识别其中的二维码
            print(matches)
            count=0
    # ####
    # matches 变量存储的是 decode 函数返回的一个列表，其中每个元素都是一个 Decoded 对象，代表识别到的 QR 码。每个 Decoded 对象包含以下几个重要属性：
    # 1. data：这是识别到的 QR 码中存储的实际数据，通常是一个字符串。例如，可能是一个 URL 或其他文本信息。
    # 2. polygon：这是一个包含 QR 码四个角点坐标的列表，通常是一个包含四个点的多边形（如果 QR 码是矩形的）。这些点可以用来绘制 QR 码的轮廓。
    # 3. rect：这是一个矩形对象，表示 QR 码在图像中的位置和大小，通常包含 x, y, width, height 属性。
    # 4. type：表示识别到的条形码或 QR 码的类型。
    # ####
            if matches:
                for match in matches:
                    count=count+1
                    tl, bl, br, tr = match.polygon  # 获取QR码的四个角点
                    match_cp = ((tl.x + bl.x + br.x + tr.x) / 4, (tl.y + bl.y + br.y + tr.y) / 4)  # 计算中心点

                    centerpoint_dist = np.linalg.norm(np.array(match_cp) - np.array(centerpoint))  # 计算中心点距离
                    outline_color = (0, 0, 255)  # 默认轮廓颜色
                    if centerpoint_dist < 10:
                        outline_color = (0, 255, 0)  # 如果距离小于10，改变颜色

                    # 绘制轮廓
                    cv2.line(cv_img, (tl.x, tl.y), (bl.x, bl.y), outline_color, 3)
                    cv2.line(cv_img, (bl.x, bl.y), (br.x, br.y), outline_color, 3)
                    cv2.line(cv_img, (br.x, br.y), (tr.x, tr.y), outline_color, 3)
                    cv2.line(cv_img, (tr.x, tr.y), (tl.x, tl.y), outline_color, 3)

                    # 绘制中心点
                    cv2.line(cv_img, (int(match_cp[0] - cp_size), int(match_cp[1] - cp_size)), (int(match_cp[0] + cp_size), int(match_cp[1] + cp_size)), (0, 0, 255), 2)
                    cv2.line(cv_img, (int(match_cp[0] - cp_size), int(match_cp[1] + cp_size)), (int(match_cp[0] + cp_size), int(match_cp[1] - cp_size)), (0, 0, 255), 2)

                    # 绘制图像中心
                    cv2.line(cv_img, (int(centerpoint[0] - cp_size), int(centerpoint[1] - cp_size)), (int(centerpoint[0] + cp_size), int(centerpoint[1] + cp_size)), (255, 0, 0), 2)
                    cv2.line(cv_img, (int(centerpoint[0] - cp_size), int(centerpoint[1] + cp_size)), (int(centerpoint[0] + cp_size), int(centerpoint[1] - cp_size)), (255, 0, 0), 2)

                    # 绘制文本
                    cv2.putText(cv_img, f'{count}code content:{match.data.decode("utf-8")}', (tl.x, tl.y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    
                    # 显示二维码中心点坐标
                    cv2.putText(cv_img, f'{count}{match_cp}', (int(match_cp[0]), int(match_cp[1] - 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    # 在中心点旁边标注坐标（0,0）
                    cv2.putText(cv_img, '(0,0)', (int(centerpoint[0] + 10), int(centerpoint[1] - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                    print(f'{count}二维码位置: {match_cp}, 编码: {match.data.decode("utf-8")}')  # 同时打印出结果

                    
                    # # 创建并发布 PoseStamped 消息
                    # pose_msg = PoseStamped()
                    # pose_msg.header.stamp = self.get_clock().now().to_msg()
                    # pose_msg.header.frame_id = 'odom'
                    # pose_msg.pose.position.x = self.pose[0]
                    # pose_msg.pose.position.y = self.pose[1]
                    # pose_msg.pose.position.z = 0.0
                    # pose_msg.pose.orientation.x = 0.0
                    # pose_msg.pose.orientation.y = 0.0
                    # pose_msg.pose.orientation.z = 0.0
                    # pose_msg.pose.orientation.w = 0.0

                    # self.pose_publisher.publish(pose_msg)

                    # 参数传递
                    self.pose = match_cp
                    self.decode = match.data.decode("utf-8")

                    # 确保二维码内容是 0-self.number_robots-1 的整数
                    try:
                        robot_index = int(self.decode)
                        if 0 <= robot_index < self.number_robots:
                            namespace = f'robot_{robot_index}'  # 使其匹配 robot_0 - robot_5
                            topic_name = f'{namespace}/current_pose'

                            # 创建并发布 PoseStamped 消息
                            pose_msg = PoseStamped()
                            pose_msg.header.stamp = self.get_clock().now().to_msg()
                            pose_msg.header.frame_id = 'odom'
                            pose_msg.pose.position.x = self.pose[0]
                            pose_msg.pose.position.y = self.pose[1]
                            pose_msg.pose.position.z = 0.0
                            pose_msg.pose.orientation.x = 0.0
                            pose_msg.pose.orientation.y = 0.0
                            pose_msg.pose.orientation.z = 0.0
                            pose_msg.pose.orientation.w = 1.0  

                            if namespace in self.publishers_dict:
                                self.publishers_dict[namespace].publish(pose_msg)
                                self.get_logger().info(f'Published pose to {topic_name}: {self.pose}')
                            else:
                                self.get_logger().warn(f"没有找到 {namespace} 的发布者！")
                        else:
                            self.get_logger().warn(f"二维码内容无效: {self.decode}")
                
                    except ValueError:
                        self.get_logger().warn(f"无效的二维码内容: {self.decode}")
                
        except Exception as exc:
            print('Error: %s' % exc)  # 错误日志


def main(args=None):
    rclpy.init(args=args)

    # 创建节点
    node = QrcodePosePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 销毁节点并关闭 rclpy
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


