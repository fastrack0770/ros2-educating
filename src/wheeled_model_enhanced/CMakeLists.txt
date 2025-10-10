cmake_minimum_required(VERSION 3.8)
project(wheeled_model_enhanced)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ros_ign_gazebo REQUIRED)

# Generate custom interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ReachGoal.action"
  DEPENDENCIES builtin_interfaces std_msgs action_msgs
  ADD_LINTER_TESTS
)

# Get support for using custom interfaces
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# Actions
add_executable(reach_goal_action_server src/reach_goal_action_server.cpp)
ament_target_dependencies(reach_goal_action_server rclcpp action_msgs rclcpp_action)
target_link_libraries(reach_goal_action_server "${cpp_typesupport_target}")

# Install
install(TARGETS
  reach_goal_action_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY models DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
