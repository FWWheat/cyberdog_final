import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from protocol.srv import AudioTextPlay  # 请根据你机器狗的定义修改此路径
import re

class VoiceRelayNode(Node):
    def __init__(self):
        super().__init__('voice')

        # 全局变量，用于存储上一次的QR内容
        self.last_qr_data = None

        # 订阅二维码识别结果
        self.subscription = self.create_subscription(
            String,
            '/qr_detector/qr_info',
            self.qr_callback,
            10
        )

        # 创建 TTS 播报 service 客户端
        self.client = self.create_client(AudioTextPlay, '/mi_desktop_48_b0_2d_7b_03_d0/speech_text_play')  # 替换成实际服务名
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 /mi_desktop_48_b0_2d_7b_03_d0/speech_text_play 服务中...')

        self.get_logger().info('语音播报节点已启动')

    def qr_callback(self, msg):
        self.get_logger().info(f"收到识别信息: {msg.data}")
        
        # 提取 QR 数据
        match = re.search(r"QR:(.*?),Distance:", msg.data)
        if match:
            qr_data = match.group(1)
            self.get_logger().info(f"提取QR内容: {qr_data}")
            
            # 检查是否与上一次的QR内容相同
            if qr_data != self.last_qr_data:
                self.get_logger().info(f"检测到新的QR内容，开始播报")
                self.send_to_tts(qr_data)
                self.last_qr_data = qr_data
            else:
                self.get_logger().info(f"QR内容与上次相同，跳过播报")
        else:
            self.get_logger().warn("无法从信息中提取QR数据")

    def send_to_tts(self, text):
        # 构造 service 请求
        req = AudioTextPlay.Request()
        req.text = text
        req.is_online = True  # 如果你想用在线 TTS，否则用 False

        # 异步发送 service 请求
        future = self.client.call_async(req)
        future.add_done_callback(self.tts_done_callback)

    def tts_done_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info("语音播报完成")
        except Exception as e:
            self.get_logger().error(f"语音播报失败: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = VoiceRelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main() 