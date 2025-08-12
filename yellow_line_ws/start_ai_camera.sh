#!/bin/bash

# AI相机启动脚本
# 使用方法: ./start_ai_camera.sh

# 记录后台进程PID
CAMERA_SERVER_PID=""

# 定义清理函数
cleanup() {
    echo "正在关闭AI相机节点..."
    if [ -n "$CAMERA_SERVER_PID" ]; then
        kill $CAMERA_SERVER_PID 2>/dev/null
        wait $CAMERA_SERVER_PID 2>/dev/null
    fi
    echo "AI相机节点已关闭。"
}

# 捕获Ctrl+C
trap cleanup SIGINT

echo "=========================================="
echo "AI相机启动脚本"
echo "=========================================="

# 设置ROS2环境
echo "[1/2] 设置ROS2环境..."
source /opt/ros2/galactic/setup.bash
echo "✓ ROS2环境已设置"

# 启动AI相机
echo "[2/2] 启动AI相机..."
echo "启动camera_server节点..."
ros2 run camera_test camera_server &
CAMERA_SERVER_PID=$!
echo "✓ camera_server已启动 (PID: $CAMERA_SERVER_PID)"

# 等待一下让camera_server完全启动
sleep 3

echo "配置AI相机服务..."
ros2 service call /mi_desktop_48_b0_2d_7b_03_d0/camera_service protocol/srv/CameraService "{command: 9, width: 640, height: 480, fps: 0}"
echo "✓ AI相机配置完成"


# 等待用户中断
wait 