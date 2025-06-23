### 工作空间功能
- 该工作空间主要用于话题的发布与分装

### 功能包：webots_demo
#### 节点：
- identify_qr_code.py
    - 识别图像中的二维码，并发出位置信息
    - 位置话题：`/qr_code_pose`
### 使用方式
- ros2 run webots_demo identify_qr_code