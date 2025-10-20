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
find_package(sensor_msgs REQUIRED)

# Generate custom interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ReachGoal.action"
  DEPENDENCIES builtin_interfaces std_msgs action_msgs
  ADD_LINTER_TESTS
)

# Get support for using custom interfaces
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# Actions
add_executable(reach_goal_action_server
  src/reach_goal_action_server.cpp
  include/wheeled_model_enhanced/types.hpp
  include/wheeled_model_enhanced/utils.hpp)
ament_target_dependencies(reach_goal_action_server rclcpp action_msgs rclcpp_action sensor_msgs)
target_include_directories(reach_goal_action_server PUBLIC include)
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
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test test/main.cpp test/tutorial_test.cpp test/utils_test.cpp)
  ament_target_dependencies(${PROJECT_NAME}_test rclcpp action_msgs rclcpp_action sensor_msgs)
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    ../include
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME}_test "${cpp_typesupport_target}")
endif()

ament_package()
