cmake_minimum_required(VERSION 3.5)
project(state_machine)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python module
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  state_machine/state_machine_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME state_machine_node
)
install(PROGRAMS
  state_machine/qr_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME qr_detector_node
)
install(PROGRAMS
  state_machine/voice_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME voice_node
)
install(PROGRAMS
  state_machine/green_arrow_detector.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME green_arrow_detector
)
install(PROGRAMS
  state_machine/red_barrier_detector.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME red_barrier_detector
)
install(PROGRAMS
  state_machine/yellow_marker_detector.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME yellow_marker_detector
)
install(PROGRAMS
  state_machine/yellow_line_walker.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME yellow_line_walker
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package() 