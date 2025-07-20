cmake_minimum_required(VERSION 3.8)
project(foundation_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(foundation_interfaces REQUIRED)
find_package(foundation_common REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include directories
include_directories(include)

# C++ Example - Mock Sensor Component
add_executable(mock_sensor_component 
  src/mock_sensor_component.cpp
)

ament_target_dependencies(mock_sensor_component
  rclcpp
  foundation_interfaces
  foundation_common
  sensor_msgs
)

# Install C++ executables
install(TARGETS
  mock_sensor_component
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/mock_actuator_component.py
  scripts/simple_robot_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
