#!/bin/bash


tar -xvf yellow_line_ws.tar.gz
cd yellow_line_ws
source /opt/ros2/galactic/setup.bash
source install/setup.bash
ros2 run state_machine state_machine_node
