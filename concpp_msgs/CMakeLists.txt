cmake_minimum_required(VERSION 3.8)
project(concpp_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CellState.msg"
  "msg/LocalView.msg"
  "msg/RobotState.msg"
  "msg/ExecutePath.msg"
  "msg/NetworkNode.msg"
  "action/PlanPath.action"
  "srv/Sense.srv"
  "srv/GetInitialState.srv"
  "srv/PlaceRelay.srv"
  DEPENDENCIES nav_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()