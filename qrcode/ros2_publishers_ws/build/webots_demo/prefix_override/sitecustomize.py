import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/dsh/Documents/work_logs/1_3二维码定位/ros2_publishers_ws/install/webots_demo'
