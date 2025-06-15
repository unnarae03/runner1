cmake_minimum_required(VERSION 3.8)
project(runner1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)

# ▶ 실행용 노드
add_executable(bt_main
  src/main.cpp
  src/MoveToGoalNode.cpp
  src/SetAngleNode.cpp
  src/WaitForResultNode.cpp
  src/CaptureImageNode.cpp
  src/SaveResultNode.cpp
  src/ReturnToBaseNode.cpp
  src/SendMetaDataNode.cpp
  src/WaitForStartNode.cpp
)

ament_target_dependencies(bt_main
  rclcpp
  behaviortree_cpp_v3
  nav2_msgs
)

target_include_directories(bt_main PRIVATE
  ${behaviortree_cpp_v3_INCLUDE_DIRS}
)

install(TARGETS bt_main
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

# ▶ Groot plugin용 노드
add_library(runner_bt_nodes SHARED
  src/RepeatUntilFailure.cpp
  src/MoveToGoalNode.cpp
  src/SetAngleNode.cpp
  src/WaitForResultNode.cpp
  src/CaptureImageNode.cpp
  src/SaveResultNode.cpp
  src/ReturnToBaseNode.cpp
  src/SendMetaDataNode.cpp
  src/WaitForStartNode.cpp
)

target_include_directories(runner_bt_nodes PRIVATE
  include
  ${behaviortree_cpp_v3_INCLUDE_DIRS}
)

ament_target_dependencies(runner_bt_nodes
  rclcpp
  behaviortree_cpp_v3
  nav2_msgs
  pluginlib
)

pluginlib_export_plugin_description_file(behaviortree_cpp_v3 plugin.xml)

install(TARGETS runner_bt_nodes
  DESTINATION lib
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CaptureImage.srv"
  "srv/CheckImageMatch.srv"
  "srv/SaveResult.srv"
)

ament_package()
