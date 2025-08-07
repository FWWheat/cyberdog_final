#!/bin/bash

# 黄色线任务启动脚本
# 使用方法: ./start_mission.sh

# 记录所有后台进程PID
CAMERA_SERVER_PID=""
REALSENSE_PID=""
STEREO_CAMERA_PID=""
QR_DETECTOR_PID=""
VOICE_NODE_PID=""
STATE_MACHINE_PID=""

# 定义清理函数
cleanup() {
    echo "正在关闭所有节点..."
    for pid in $STATE_MACHINE_PID $VOICE_NODE_PID $QR_DETECTOR_PID $STEREO_CAMERA_PID $REALSENSE_PID $CAMERA_SERVER_PID; do
        if [ -n "$pid" ]; then
            kill $pid 2>/dev/null
            wait $pid 2>/dev/null
        fi
    done
    echo "所有节点已关闭。"
}

# 捕获Ctrl+C
trap cleanup SIGINT

echo "=========================================="
echo "黄色线任务启动脚本"
echo "=========================================="

# 设置ROS2环境
source /opt/ros2/galactic/setup.bash
echo "✓ ROS2环境已设置"

# 设置工作空间环境
source install/setup.bash
echo "✓ 工作空间环境已设置"

# 启动任务相关节点
echo "启动二维码检测节点..."
ros2 run state_machine qr_detector_node &
QR_DETECTOR_PID=$!
echo "✓ 二维码检测节点已启动 (PID: $QR_DETECTOR_PID)"

echo "启动语音节点..."
ros2 run state_machine voice_node &
VOICE_NODE_PID=$!
echo "✓ 语音节点已启动 (PID: $VOICE_NODE_PID)"

echo "启动状态机节点..."
ros2 run state_machine state_machine_node &
STATE_MACHINE_PID=$!
echo "✓ 状态机节点已启动 (PID: $STATE_MACHINE_PID)"

# 等待节点启动
sleep 3

# 等待用户中断
wait 