```
mi@mi-desktop:~$ source /opt/ros2/galactic/setup.bash
not found: "/opt/ros2/galactic/share/libg2o/local_setup.bash"
mi@mi-desktop:~$ ros2 action list
/mi_desktop_48_b0_2d_7b_03_d0/automatic_recharge
/mi_desktop_48_b0_2d_7b_03_d0/cyberdog_ota_action
/mi_desktop_48_b0_2d_7b_03_d0/navigate_to_pose
/mi_desktop_48_b0_2d_7b_03_d0/reconstruct_map
/mi_desktop_48_b0_2d_7b_03_d0/seatadjust
/mi_desktop_48_b0_2d_7b_03_d0/speech_text_action
/mi_desktop_48_b0_2d_7b_03_d0/start_algo_task
/mi_desktop_48_b0_2d_7b_03_d0/tracking_target
/mi_desktop_48_b0_2d_7b_03_d0/update_map
mi@mi-desktop:~$ 


mi@mi-desktop:~$ ros2 action info /mi_desktop_48_b0_2d_7b_03_d0/start_algo_task
Action: /mi_desktop_48_b0_2d_7b_03_d0/start_algo_task
Action clients: 3
    //m/i/_/d/e/s/k/t/o/p/_/4/8/_/b/0/_/2/d/_/7/b/_/0/3/_/d/0uwb_tracking_action_client
    //m/i/_/d/e/s/k/t/o/p/_/4/8/_/b/0/_/2/d/_/7/b/_/0/3/_/d/0app_server
    //m/i/_/d/e/s/k/t/o/p/_/4/8/_/b/0/_/2/d/_/7/b/_/0/3/_/d/0cyberdog_bluetooth
Action servers: 1
    //m/i/_/d/e/s/k/t/o/p/_/4/8/_/b/0/_/2/d/_/7/b/_/0/3/_/d/0algorithm_manager
mi@mi-desktop:~$ 

mi@mi-desktop:~$ ros2 action list -t
/mi_desktop_48_b0_2d_7b_03_d0/automatic_recharge [mcr_msgs/action/AutomaticRecharge]
/mi_desktop_48_b0_2d_7b_03_d0/cyberdog_ota_action [protocol/action/OverTheAir]
/mi_desktop_48_b0_2d_7b_03_d0/navigate_to_pose [nav2_msgs/action/NavigateToPose]
/mi_desktop_48_b0_2d_7b_03_d0/reconstruct_map [cyberdog_visions_interfaces/action/MapParam]
/mi_desktop_48_b0_2d_7b_03_d0/seatadjust [protocol/action/SeatAdjust]
/mi_desktop_48_b0_2d_7b_03_d0/speech_text_action [protocol/action/Speech]
/mi_desktop_48_b0_2d_7b_03_d0/start_algo_task [protocol/action/Navigation]
/mi_desktop_48_b0_2d_7b_03_d0/tracking_target [mcr_msgs/action/TargetTracking]
/mi_desktop_48_b0_2d_7b_03_d0/update_map [cyberdog_visions_interfaces/action/UpdateParam]
mi@mi-desktop:~$ 


mi@mi-desktop:~$ ros2 action send_goal /mi_desktop_48_b0_2d_7b_03_d0/start_algtask protocol/action/Navigation "{nav_type: 5, map_name: 'vision_map', outdoor: true, object_tracking: false}"
Waiting for an action server to become available...
Sending goal:
     nav_type: 5
poses: []
label_id: ''
map_name: vision_map
tracking_roi:
  x_offset: 0
  y_offset: 0
  height: 0
  width: 0
  do_rectify: false
relative_pos: 0
keep_distance: 0.0
outdoor: true
object_tracking: false

Goal was rejected.



```
# 教程

## 通用接口
- 启动任务
```
接口形式：ros action
接口名字："start_algo_task"
接口文件：protocol/ros/srv/Navigation.action
接口内容：
uint8                         nav_type         # 任务类型

geometry_msgs/PoseStamped[]   poses            # AB点导航时设定的目标点

string                        map_name  

bool                          outdoor          # 室内室外建图标志位                           

sensor_msgs/RegionOfInterest  tracking_roi     # 跟随时的roi信息

bool                          object_tracking  # 万物跟随标志位

---

uint8                         result           # 结果

---

int32                         feedback_code    # 状态反馈

string                        feedback_msg     # 状态信息  
```

- 关闭任务
```
接口形式： ros service
接口名字： "stop_algo_task"
接口文件：protocol/ros/srv/StopAlgoTask.srv
接口内容：
uint8 task_id    # 任务的ID

---

bool  result     # true: 任务正常结束, false: 任务结束异常
```

- 任务状态查询
```
接口形式： ros topic
接口名字： "algo_task_status"
接口文件：protocol/ros/msg/AlgoTaskStatus.msg
接口内容：
uint8 task_status 

int32 task_sub_status
```

## 视觉建图
- 视觉建图应用在室外环境中，使用相机数据，完成同时定位和建图功能，生成的二维平面地图供导航使用。
- 初始位置处的重定位功能，即狗在开机后，可根据平面地图定位自身的位置。
- 与激光地图相同，可以在绘制的视觉地图上标注语义信息。
- 启动方式见启动任务action接口，其中所需的关键goal字段包含：
`"nav_type: 5"         # 5表示启动建图`

`"out_door: true"      # true表示室外建图，即视觉建图`

- 关闭方式见关闭任务service接口，其中所需的关键request字段包含：
`"task_id: 5"          # 5表示关闭建图`



# 关机指令
- 上电,站起来：`ros2 service call /mi_desktop_48_b0_2d_7b_03_d0/dog_leg_calibration std_srvs/srv/SetBool "{data: true}"`
- 断电，趴下：` ros2 service call /mi_desktop_48_b0_2d_7b_03_d0/dog_leg_calibration std_srvs/srv/SetBool "{data: false}"`

# 具体过程
## 1. 开启视觉建图：
- `ros2 action send_goal /mi_desktop_48_b0_2d_7b_03_d0/start_algo_task protocol/action/Navigation "{nav_type: 5, map_name: 'vision_map', outdoor: true, object_tracking: false}"`
## 2. 保存建图结果：
- `ros2 run nav2_map_server map_saver_cli -f ~/vision_map --ros-args -r map:=/mi_desktop_48_b0_2d_7b_03_d0/map`
## 3. 关闭视觉建图任务：
- `ros2 service call /mi_desktop_48_b0_2d_7b_03_d0/stop_algo_task protocol/srv/StopAlgoTask "{task_id: 5}"`