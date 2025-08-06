1. 进入：ssh  mi@192.168.217.61
2. 加载ros2环境：source /opt/ros2/galactic/setup.bash
3.  
``` 
# /camera/camera节点启动指令

  ros2 launch realsense2_camera on_dog.py

  

  #初始化

  ros2 lifecycle set /camera/camera configure 

  

  #开启数据

  ros2 lifecycle set /camera/camera activate
```

3. ros2 launch camera_test stereo_camera.py
```
mi@mi-desktop:~$ ros2 launch camera_test stereo_camera.py
[INFO] [launch]: All log files can be found below /home/mi/.ros/log/2025-08-05-00-13-33-401263-mi-desktop-8856
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [stereo_camera-1]: process started with pid [8885]

```
4. ros2 lifecycle set /stereo_camera configure
```
mi@mi-desktop:~$ ros2 launch camera_test stereo_camera.py
[INFO] [launch]: All log files can be found below /home/mi/.ros/log/2025-08-05-00-13-33-401263-mi-desktop-8856
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [stereo_camera-1]: process started with pid [8885]
[stereo_camera-1] [INFO] [1754324148.929051411] [STEREO_CAMERA]: on_configure: configure


mi@mi-desktop:~$ ros2 lifecycle set /stereo_camera configure
Transitioning successful
mi@mi-desktop:~$ 
```

5. ros2 lifecycle set /stereo_camera activate
```
mi@mi-desktop:~$ ros2 lifecycle set /stereo_camera activate
Transitioning successful
mi@mi-desktop:~$ 


```
 

6. 查看设备情况:
- sudo apt update && sudo apt install v4l-utils

```
mi@mi-desktop:~$ ls -la /dev/video*
crw-rw----+ 1 root video   81,  0 Aug  4 23:25 /dev/video0
crw-rw----+ 1 root video   81,  3 Aug  4 23:25 /dev/video1
crw-rw----+ 1 root video   81,  6 Aug  4 23:25 /dev/video2
crw-rw----+ 1 root video   81,  9 Aug  4 23:25 /dev/video3
crw-rw-rw-+ 1 root plugdev 81, 12 Aug  4 23:25 /dev/video4
crw-rw-rw-+ 1 root plugdev 81, 13 Aug  4 23:25 /dev/video5


mi@mi-desktop:~$ v4l2-ctl --list-devices
vi-output, gc02m1 2-0037 (platform:15c10000.vi:0):
	/dev/video1

vi-output, ov9782 2-0060 (platform:15c10000.vi:1):
	/dev/video2

vi-output, ov9782 2-0010 (platform:15c10000.vi:2):
	/dev/video3

vi-output, ov13b10 2-0036 (platform:15c10000.vi:4):
	/dev/video0

Intel(R) RealSense(TM) Depth Ca (usb-3610000.xhci-3.3):
	/dev/video4
	/dev/video5

mi@mi-desktop:~$ 


mi@mi-desktop:~$ v4l2-ctl -d /dev/video0 --all
Driver Info (not using libv4l2):
	Driver name   : tegra-video
	Card type     : vi-output, ov13b10 2-0036
	Bus info      : platform:15c10000.vi:4
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
Video input : 0 (Camera 4: ok)
Format Video Capture:
	Width/Height      : 4208/3120
	Pixel Format      : 'RG10'
	Field             : None
	Bytes per Line    : 8416
	Size Image        : 26257920
	Colorspace        : sRGB
	Transfer Function : Default (maps to sRGB)
	YCbCr/HSV Encoding: Default (maps to ITU-R 601)
	Quantization      : Default (maps to Full Range)
	Flags             : 



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



mi@mi-desktop:~$ v4l2-ctl -d /dev/video3 --all | head -27
Driver Info (not using libv4l2):
	Driver name   : tegra-video
	Card type     : vi-output, ov9782 2-0010
	Bus info      : platform:15c10000.vi:2
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
Video input : 0 (Camera 2: ok)
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



mi@mi-desktop:~$ v4l2-ctl -d /dev/video4 --all | head -27
Driver Info (not using libv4l2):
	Driver name   : uvcvideo
	Card type     : Intel(R) RealSense(TM) Depth Ca
	Bus info      : usb-3610000.xhci-3.3
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
	Width/Height      : 256/144
	Pixel Format      : 'Z16 '
	Field             : None
	Bytes per Line    : 512
	Size Image        : 73728
	Colorspace        : Default
	Transfer Function : Default (maps to Rec. 709)
	YCbCr/HSV Encoding: Default (maps to ITU-R 601)
	Quantization      : Default (maps to Full Range)
	Flags             : 
mi@mi-desktop:~$ 



mi@mi-desktop:~$ v4l2-ctl -d /dev/video5 --all | head -27
Driver Info (not using libv4l2):
	Driver name   : uvcvideo
	Card type     : Intel(R) RealSense(TM) Depth Ca
	Bus info      : usb-3610000.xhci-3.3
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
	Width/Height      : 424/240
	Pixel Format      : 'GREY'
	Field             : None
	Bytes per Line    : 424
	Size Image        : 101760
	Colorspace        : Default
	Transfer Function : Default (maps to Rec. 709)
	YCbCr/HSV Encoding: Default (maps to ITU-R 601)
	Quantization      : Default (maps to Full Range)
	Flags             : 
mi@mi-desktop:~$ 


mi@mi-desktop:~$ lspci | grep -i nvidia
0004:00:00.0 PCI bridge: NVIDIA Corporation Device 1ad1 (rev a1)
mi@mi-desktop:~$ 

```



# 断开与APP的连接
- ros2 topic pub -1 /disconnect_app std_msgs/msg/Bool "{data: true}"
