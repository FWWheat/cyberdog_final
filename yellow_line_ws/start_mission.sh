#!/bin/bash

# 黄色线任务启动脚本
# 作者: 根据启动节点名单.md创建
# 使用方法: ./start_mission.sh

echo "=========================================="
echo "黄色线任务启动脚本"
echo "=========================================="

# 设置ROS2环境
echo "[1/6] 设置ROS2环境..."
source /opt/ros2/galactic/setup.bash
echo "✓ ROS2环境已设置"

# 设置工作空间环境
echo "[2/6] 设置工作空间环境..."
cd yellow_line_ws
source install/setup.bash
echo "✓ 工作空间环境已设置"

# 启动AI相机
echo "[3/6] 启动AI相机..."
echo "启动camera_server节点..."
ros2 run camera_test camera_server &
CAMERA_SERVER_PID=$!
echo "✓ camera_server已启动 (PID: $CAMERA_SERVER_PID)"

# 等待一下让camera_server完全启动
sleep 3

echo "配置AI相机服务..."
ros2 service call /mi_desktop_48_b0_2d_7b_03_d0/camera_service protocol/srv/CameraService "{command: 9, width: 640, height: 480, fps: 0}"
echo "✓ AI相机配置完成"

# 启动RGB相机和鱼眼相机
echo "[4/6] 启动RGB相机和鱼眼相机..."
echo "启动RealSense相机..."
ros2 launch realsense2_camera on_dog.py &
REALSENSE_PID=$!
echo "✓ RealSense相机已启动 (PID: $REALSENSE_PID)"

# 等待RealSense相机启动
sleep 5

echo "配置RealSense相机..."
ros2 lifecycle set /camera/camera configure
ros2 lifecycle set /camera/camera activate
echo "✓ RealSense相机配置完成"

echo "启动立体相机..."
ros2 launch camera_test stereo_camera.py &
STEREO_CAMERA_PID=$!
echo "✓ 立体相机已启动 (PID: $STEREO_CAMERA_PID)"

# 等待立体相机启动
sleep 5

echo "配置立体相机..."
ros2 lifecycle set /stereo_camera configure
ros2 lifecycle set /stereo_camera activate
echo "✓ 立体相机配置完成"

# 启动任务相关节点
echo "[5/6] 启动任务相关节点..."
echo "启动二维码检测节点..."
ros2 run state_machine qr_detector_node &
QR_DETECTOR_PID=$!
echo "✓ 二维码检测节点已启动 (PID: $QR_DETECTOR_PID)"

echo "启动状态机节点..."
ros2 run state_machine state_machine_node &
STATE_MACHINE_PID=$!
echo "✓ 状态机节点已启动 (PID: $STATE_MACHINE_PID)"

# 等待节点启动
sleep 3

# 启动任务
echo "[6/6] 启动任务..."
echo "发送启动命令..."
ros2 topic pub -1 /state_machine/start_command std_msgs/msg/String "data: 'START'"
echo "✓ 任务启动命令已发送"

echo "=========================================="
echo "所有节点已启动完成！"
echo "=========================================="
echo ""
echo "运行中的进程PID:"
echo "- camera_server: $CAMERA_SERVER_PID"
echo "- RealSense相机: $REALSENSE_PID"
echo "- 立体相机: $STEREO_CAMERA_PID"
echo "- 二维码检测: $QR_DETECTOR_PID"
echo "- 状态机: $STATE_MACHINE_PID"
echo ""
echo "监控命令:"
echo "- 查看状态机状态: ros2 topic echo /state_machine/state_info"
echo "- 查看二维码信息: ros2 topic echo /qr_detector/qr_info"
echo "- 查看调试图像: ros2 topic echo /qr_detector/debug_image"
echo ""
echo "停止所有节点: Ctrl+C"
echo "=========================================="

# 等待用户中断
wait 