# 预设
1. 状态枚举
    - start
    - start_to_qra
    - in_qr_a
    - qra_to_a1
    - qra_to_a2
    - in_a1
    - in_a2
    - a1_to_s1
    - a2_to_s1
    - end

2. 基础运动类型
    1. 向前，+x 
    2. 向后，-x 
    3. 向右，-y 
    4. 向左，+y 
    5. 右转90度 
    6. 左转90度
    7. 站立，
    8. 趴下，
    9. 停止，

3. ros2话题通信
    - 订阅
        1. 二维码信息，/qr_detector/qr_info
        2. 任务控制命令，/state_machine/start_command（任务行输入：START，STOP，RESET）
        3. rgb相机，/image_rgb
        4. 右鱼眼相机，/image_right

    - 发布
        1. 当前状态信息，/state_machine/state_info
        2. 运动控制指令，/mi_desktop_48_b0_2d_7b_03_d0/motion_servo_cmd

4. 每个状态做的事情，（完成后，即可进入下一个状态）
    - start ：收到任务控制指令为START （下一状态为start_to_qra）
    - start_to_qra ：执行运动类型7,执行运行类型1,然后执行运动类型5（下一状态为in_qr_a）
    - in_qr_a ：等待收到二维码信息，保存信息，**语音播放**（根据二维码信息，为A-1时，下一状态为qra_to_a1；为A-2时，下一状态为qra_to_a2）
    - qra_to_a1 ：执行运动类型1，执行运动类型3,然后执行运动类型2（下一状态为in_a1）
    - qra_to_a2 ：执行运动类型1，执行运动类型4,然后执行运动类型2（下一状态为in_a2）
    - in_a1 ：执行运动类型8，**语音交互**（下一状态为a1_to_s1）
    - in_a2 ：执行运动类型8，**语音交互**（下一状态为a2_to_s1）
    - a1_to_s1 ：执行运动类型7,然后执行任务1,然后执行任务4,然后执行任务2,然后执行任务6（下一状态为end）
    - a2_to_s1：执行运动类型7,然后执行任务1,然后执行任务3,然后执行任务2,然后执行任务6（下一状态为end）
    - end ：清理，关闭


5. 黄线识别运动控制模块

    - start_to_qra：
        - 站起
        - 前进：接收/image_right消息，该话题发布一个鱼眼相机拍摄的图像,
            希望在完成向前行走的任务后，查看当前鱼眼图片中，是否位于两条黄色纵向平行线中间，如果靠近左边的线，则向后走一步；如果靠近右边的线，则向前走一步，如果差不多在中间，则进入下一阶段
        - 右转：接收/image_rgb消息，该话题发布一个普通相机拍摄的图像,
            希望在完成向右转的任务后，查看当前图片中，是否位于两条黄色纵向平行线中间，如果靠近左边的线，则向右转1步；如果靠近右边的线，则向左转一步，如果差不多在中间，则进入下一阶段
        

    - qra_to_a1：
        - 前进：
        
        - 右移
        - 后退

    - qra_to_a2
 
    - a1_to_s1
    - a2_to_s1


鱼眼相机相关参数:
mi@mi-desktop:~$ v4l2-ctl -d /dev/video2 --all | head -27
Driver Info (not using libv4l2):
	Driver name   : tegra-video
	Card type     : vi-output, ov9782 2-0060
	Bus info      : platform:15c10000.vi:1
	Driver version: 4.9.253
	Capabilities  : 0x84200001
		Video Capture
		Streaming
		Extended Pix Format
		Device Capabilities
	Device Caps   : 0x04200001
		Video Capture
		Streaming
		Extended Pix Format
Priority: 2
Video input : 0 (Camera 1: ok)
Format Video Capture:
	Width/Height      : 1000/800
	Pixel Format      : 'RG10'
	Field             : None
	Bytes per Line    : 2000
	Size Image        : 1600000
	Colorspace        : sRGB
	Transfer Function : Default (maps to sRGB)
	YCbCr/HSV Encoding: Default (maps to ITU-R 601)
	Quantization      : Default (maps to Full Range)
	Flags             : 
mi@mi-desktop:~$ 


rgb相机参数：
mi@mi-desktop:~$ v4l2-ctl -d /dev/video1 --all
Driver Info (not using libv4l2):
	Driver name   : tegra-video
	Card type     : vi-output, gc02m1 2-0037
	Bus info      : platform:15c10000.vi:0
	Driver version: 4.9.253
	Capabilities  : 0x84200001
		Video Capture
		Streaming
		Extended Pix Format
		Device Capabilities
	Device Caps   : 0x04200001
		Video Capture
		Streaming
		Extended Pix Format
Priority: 2
Video input : 0 (Camera 0: ok)
Format Video Capture:
	Width/Height      : 1600/1200
	Pixel Format      : 'RG10'
	Field             : None
	Bytes per Line    : 3200
	Size Image        : 3840000
	Colorspace        : sRGB
	Transfer Function : Default (maps to sRGB)
	YCbCr/HSV Encoding: Default (maps to ITU-R 601)
	Quantization      : Default (maps to Full Range)
	Flags             : 

