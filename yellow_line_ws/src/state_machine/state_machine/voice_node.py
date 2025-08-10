#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from protocol.srv import AudioTextPlay  # 请根据你机器狗的定义修改此路径

class VoiceRelayNode(Node):
    def __init__(self):
        super().__init__('voice')

        # 全局变量，用于存储上一次的QR内容和箭头方向
        self.last_qr_data = None
        self.last_arrow_direction = None  # 记录上一次的箭头方向，防止重复播报
        self.countdown_played = False  # 记录倒计时是否已播报，防止重复播报
        self.countdown_timer = None  # 倒计时定时器

        # 订阅二维码识别结果
        self.qr_subscription = self.create_subscription(
            String,
            '/qr_detector/qr_info',
            self.qr_callback,
            10
        )
        
        # 订阅绿色箭头方向识别结果
        self.arrow_subscription = self.create_subscription(
            String,
            '/green_arrow_detector/direction',
            self.arrow_callback,
            10
        )
        
        # 订阅倒计时命令
        self.countdown_subscription = self.create_subscription(
            String,
            '/voice_node/countdown_command',
            self.countdown_callback,
            10
        )

        # 创建 TTS 播报 service 客户端
        self.client = self.create_client(AudioTextPlay, '/mi_desktop_48_b0_2d_7b_03_d0/speech_text_play')  # 替换成实际服务名
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 /mi_desktop_48_b0_2d_7b_03_d0/speech_text_play 服务中...')

        self.get_logger().info('语音播报节点已启动')
        self.get_logger().info('功能说明:')
        self.get_logger().info('  1. 二维码内容播报（避免重复）')
        self.get_logger().info('  2. 绿色箭头方向播报（一次性）')
        self.get_logger().info('  3. 倒计时播报5,4,3,2,1（一次性）')
        self.get_logger().info('订阅话题:')
        self.get_logger().info('  - /qr_detector/qr_info (二维码)')
        self.get_logger().info('  - /green_arrow_detector/direction (箭头)')
        self.get_logger().info('  - /voice_node/countdown_command (倒计时)')
        self.get_logger().info('发送倒计时命令示例: ros2 topic pub -1 /voice_node/countdown_command std_msgs/msg/String "data: START"')

    def qr_callback(self, msg):
        self.get_logger().info(f"收到二维码识别信息: {msg.data}")
        
        # 新格式：直接是二维码内容，如 "A-1", "A-2", "B-1", "B-2"
        qr_data = msg.data.strip()
        
        # 检查是否与上一次的QR内容相同
        if qr_data != self.last_qr_data:
            self.get_logger().info(f"检测到新的QR内容，开始播报: {qr_data}")
            self.send_to_tts(qr_data)
            self.last_qr_data = qr_data
        else:
            self.get_logger().info(f"QR内容与上次相同，跳过播报")
    
    def arrow_callback(self, msg):
        """绿色箭头方向回调 - 一次性播报"""
        direction = msg.data.strip()
        self.get_logger().info(f"收到绿色箭头方向: {direction}")
        
        # 检查是否与上一次的箭头方向相同，避免重复播报
        if direction != self.last_arrow_direction:
            self.get_logger().info(f"检测到新的箭头方向，开始播报: {direction}")
            
            # 将方向转换为中文播报
            direction_text = ""
            if direction == "left":
                direction_text = "向左"
            elif direction == "right":
                direction_text = "向右"
            else:
                direction_text = direction  # 如果是其他内容，直接播报
            
            self.send_to_tts(f"箭头方向{direction_text}")
            self.last_arrow_direction = direction
        else:
            self.get_logger().info(f"箭头方向与上次相同，跳过播报")
    
    def countdown_callback(self, msg):
        """倒计时命令回调 - 只播报一次5,4,3,2,1"""
        command = msg.data.strip()
        self.get_logger().info(f"收到倒计时命令: {command}")
        
        if command == "START" and not self.countdown_played:
            self.get_logger().info("开始倒计时播报")
            self.countdown_played = True  # 标记为已播报，防止重复
            self.start_countdown()
        elif command == "START" and self.countdown_played:
            self.get_logger().info("倒计时已播报过，跳过播报")
        elif command == "RESET":
            # 重置倒计时播报状态
            self.countdown_played = False
            self.get_logger().info("倒计时状态已重置")
        else:
            self.get_logger().warn(f"未知倒计时命令: {command}")
    
    def start_countdown(self):
        """开始倒计时播报：5,4,3,2,1"""
        self.countdown_step = 5  # 从5开始倒数
        self.countdown_next_step()
    
    def countdown_next_step(self):
        """播报倒计时的下一步"""
        if self.countdown_step > 0:
            self.get_logger().info(f"倒计时播报: {self.countdown_step}")
            self.send_to_tts(str(self.countdown_step))
            
            # 安排下一次播报（1秒后）
            self.countdown_step -= 1
            if self.countdown_step > 0:
                self.countdown_timer = self.create_timer(1.0, self.countdown_next_step)
            else:
                self.get_logger().info("倒计时播报完成")
        
        # 取消定时器（单次使用）
        if hasattr(self, 'countdown_timer') and self.countdown_timer:
            self.countdown_timer.cancel()
            self.countdown_timer = None

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