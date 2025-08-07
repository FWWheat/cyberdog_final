#!/bin/bash

# 记录后台进程PID
REALSENSE_PID=""
STEREO_CAMERA_PID=""

# 定义清理函数
cleanup() {
    echo "正在关闭相机节点..."
    if [ -n "$REALSENSE_PID" ]; then
        kill $REALSENSE_PID
        wait $REALSENSE_PID 2>/dev/null
    fi
    if [ -n "$STEREO_CAMERA_PID" ]; then
        kill $STEREO_CAMERA_PID
        wait $STEREO_CAMERA_PID 2>/dev/null
    fi
    echo "所有相机节点已关闭。"
}

# 捕获Ctrl+C
trap cleanup SIGINT

# 设置ROS2环境
source /opt/ros2/galactic/setup.bash


# 启动RGB相机和鱼眼相机
echo "ros2 launch realsense2_camera on_dog.py &"
ros2 launch realsense2_camera on_dog.py &
REALSENSE_PID=$!

sleep 5

ros2 lifecycle set /camera/camera configure
ros2 lifecycle set /camera/camera activate


ros2 launch camera_test stereo_camera.py &
STEREO_CAMERA_PID=$!


# 等待立体相机启动
sleep 5


ros2 lifecycle set /stereo_camera configure
ros2 lifecycle set /stereo_camera activate

# 等待所有后台进程
wait