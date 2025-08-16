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
        self.green_arrow_played = False  # 记录绿色箭头是否已播报，全程只播报一次
        self.yellow_marker_countdown_played = False  # 记录黄色标志物倒计时是否已播报，防止重复播报
        self.continuous_complete_timer = None  # 持续发送完成信号的定时器
        self.is_countdown_in_progress = False  # 标记倒计时是否正在进行

        # 订阅二维码识别结果
        self.qr_subscription = self.create_subscription(
            String,
            '/voice_node/qr_info',
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
        
        # 订阅倒计时触发信号（来自状态机，替代原来的黄色标志物距离检测）
        self.countdown_trigger_subscription = self.create_subscription(
            String,
            '/voice_node/countdown_trigger',
            self.countdown_trigger_callback,
            10
        )

        # 发布语音播报完成信号
        self.complete_publisher = self.create_publisher(
            String,
            '/voice_node/complete_signal',
            10
        )

        # 创建 TTS 播报 service 客户端
        self.client = self.create_client(AudioTextPlay, '/mi_desktop_48_b0_2d_7b_03_d0/speech_text_play')  # 替换成实际服务名
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 /mi_desktop_48_b0_2d_7b_03_d0/speech_text_play 服务中...')

        self.get_logger().info('语音播报节点已启动')
        self.get_logger().info('功能说明:')
        self.get_logger().info('  1. 二维码内容播报（避免重复）')
        self.get_logger().info('  2. 绿色箭头方向播报（全程只播报一次）')
        self.get_logger().info('  3. 接收状态机触发信号进行R1倒计时（一次性，持续发送完成信号）')
        self.get_logger().info('订阅话题:')
        self.get_logger().info('  - /qr_detector/qr_info (二维码)')
        self.get_logger().info('  - /green_arrow_detector/direction (箭头)')
        self.get_logger().info('  - /voice_node/countdown_trigger (倒计时触发信号)')

    def qr_callback(self, msg):
        self.get_logger().info(f"收到二维码识别信息: {msg.data}")
        
        # 新格式：直接是二维码内容，如 "A-1", "A-2", "B-1", "B-2"
        qr_data = msg.data.strip()
        
        # 检查是否与上一次的QR内容相同
        if qr_data != self.last_qr_data:
            self.get_logger().info(f"检测到新的QR内容，开始播报: {qr_data}")
            
            # 转换为播报内容
            broadcast_text = self.convert_qr_to_broadcast(qr_data)
            self.send_to_tts(broadcast_text)
            self.last_qr_data = qr_data
        else:
            self.get_logger().info(f"QR内容与上次相同，跳过播报")
    
    def arrow_callback(self, msg):
        """绿色箭头方向回调 - 全程只播报一次"""
        direction = msg.data.strip()
        self.get_logger().info(f"收到绿色箭头方向: {direction}")
        
        # 检查是否已经播报过绿色箭头，全程只播报一次
        if not self.green_arrow_played:
            self.get_logger().info(f"首次检测到绿色箭头，开始播报: {direction}")
            
            # 将方向转换为播报内容
            broadcast_text = self.convert_arrow_to_broadcast(direction)
            self.send_to_tts(broadcast_text)
            self.green_arrow_played = True  # 标记为已播报
            self.last_arrow_direction = direction
        else:
            self.get_logger().info(f"绿色箭头已播报过，跳过播报")
    
    def countdown_trigger_callback(self, msg):
        """倒计时触发信号回调 - 接收状态机的R1倒计时触发信号"""
        trigger_signal = msg.data.strip()
        self.get_logger().info(f"收到倒计时触发信号: {trigger_signal}")
        
        # 检查触发信号类型
        if trigger_signal == "R1_COUNTDOWN_START" and not self.yellow_marker_countdown_played:
            self.get_logger().info("收到R1倒计时触发信号，开始倒计时播报")
            self.yellow_marker_countdown_played = True  # 标记为已播报，防止重复
            self.start_yellow_marker_countdown()
        elif trigger_signal == "R1_COUNTDOWN_START" and self.yellow_marker_countdown_played:
            self.get_logger().info("R1倒计时已播报过，跳过播报")
        else:
            self.get_logger().warn(f"未知的触发信号: {trigger_signal}")
    
    def start_yellow_marker_countdown(self):
        """开始黄色标志物倒计时播报：5,4,3,2,1"""
        self.yellow_marker_countdown_step = 5  # 从5开始倒数
        self.is_countdown_in_progress = True  # 标记倒计时开始
        self.yellow_marker_countdown_next_step()
    
    def yellow_marker_countdown_next_step(self):
        """播报黄色标志物倒计时的下一步"""
        if self.yellow_marker_countdown_step > 0:
            self.get_logger().info(f"黄色标志物倒计时播报: {self.yellow_marker_countdown_step}")
            self.send_to_tts(str(self.yellow_marker_countdown_step))
            self.yellow_marker_countdown_step -= 1
            
            # 如果还需要继续倒计时，安排下一次播报（1秒后）
            if self.yellow_marker_countdown_step > 0:
                # 先取消之前的定时器（如果存在）
                if hasattr(self, 'yellow_marker_countdown_timer') and self.yellow_marker_countdown_timer:
                    self.yellow_marker_countdown_timer.cancel()
                # 创建新的定时器
                self.yellow_marker_countdown_timer = self.create_timer(1.0, self.yellow_marker_countdown_next_step)
            else:
                self.get_logger().info("黄色标志物倒计时播报完成")
                self.is_countdown_in_progress = False  # 标记倒计时结束
                # 清理定时器
                if hasattr(self, 'yellow_marker_countdown_timer') and self.yellow_marker_countdown_timer:
                    self.yellow_marker_countdown_timer.cancel()
                    self.yellow_marker_countdown_timer = None
                # 倒计时完全结束后，等待最后一次TTS播报完成，然后发送完成信号
                # 这里需要等待一小段时间确保最后的"1"播报完成，然后发送完成信号
                self.create_timer(1.5, self.countdown_final_complete)
    
    def countdown_final_complete(self):
        """倒计时完全结束后发送完成信号"""
        self.get_logger().info("倒计时完全结束，开始发送完成信号")
        self.start_continuous_complete_signal()
    
    def convert_qr_to_broadcast(self, qr_data):
        """将QR码内容转换为播报文本"""
        try:
            # 解析格式如 "A-1", "A-2", "B-1", "B-2"
            if '-' in qr_data and len(qr_data) >= 3:
                area = qr_data.split('-')[0].upper()  # A或B
                position = qr_data.split('-')[1]      # 1或2
                
                return f"{area}区库位{position}"
            else:
                # 如果格式不符合预期，直接播报原内容
                return qr_data
                
        except Exception as e:
            self.get_logger().warn(f"解析QR码内容失败: {qr_data}, 错误: {str(e)}")
            return qr_data
    
    def convert_arrow_to_broadcast(self, direction):
        """将箭头方向转换为播报文本"""
        if direction == "left":
            return "左侧路线"
        elif direction == "right":
            return "右侧路线"
        else:
            # 如果是其他内容，直接播报
            return direction

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
            
            # 只有在不是倒计时播报或倒计时已完全结束时才发送完成信号
            if not self.is_countdown_in_progress:
                self.start_continuous_complete_signal()
            
        except Exception as e:
            self.get_logger().error(f"语音播报失败: {str(e)}")
            
            # 即使播报失败，如果不是倒计时播报或倒计时已完全结束，也发送完成信号
            if not self.is_countdown_in_progress:
                self.start_continuous_complete_signal()
    
    def start_continuous_complete_signal(self):
        """开始持续发送完成信号（每0.5秒一次）"""
        # 先发送一次完成信号
        self.send_complete_signal()
        
        # 如果已经有定时器在运行，先取消
        if self.continuous_complete_timer:
            self.continuous_complete_timer.cancel()
        
        # 创建新的持续发送定时器
        self.continuous_complete_timer = self.create_timer(0.5, self.send_complete_signal)
        self.get_logger().info("开始持续发送完成信号（每0.5秒一次）")
    
    def send_complete_signal(self):
        """发送完成信号"""
        complete_msg = String()
        complete_msg.data = "COMPLETE"
        self.complete_publisher.publish(complete_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VoiceRelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main() 