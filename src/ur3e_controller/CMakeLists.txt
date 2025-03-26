cmake_minimum_required(VERSION 3.5)
project(ur3e_controller)

# Default to C++14
set(CMAKE_CXX_STANDARD 14)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(moveit_commander REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)


# Install Python modules
install(PROGRAMS
  ur3e_control.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install package.xml
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
