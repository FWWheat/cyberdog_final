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

ament_package() 