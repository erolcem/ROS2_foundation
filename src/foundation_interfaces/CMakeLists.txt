cmake_minimum_required(VERSION 3.8)
project(foundation_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Message files
set(msg_files
  "msg/RobotStatus.msg"
  "msg/ComponentStatus.msg"
  "msg/SystemHealth.msg"
)

# Service files
set(srv_files
  "srv/SetMode.srv"
  "srv/GetSystemInfo.srv"
  "srv/ConfigureComponent.srv"
)

# Action files
set(action_files
  "action/MoveToPosition.action"
  "action/ExecuteTask.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs action_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
