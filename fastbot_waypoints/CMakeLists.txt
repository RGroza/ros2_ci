cmake_minimum_required(VERSION 3.8)
project(fastbot_waypoints)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(fastbot_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(fastbot_action_server src/fastbot_action_server.cpp)
ament_target_dependencies(fastbot_action_server
  rclcpp rclcpp_action geometry_msgs nav_msgs fastbot_msgs tf2 tf2_geometry_msgs)

install(TARGETS fastbot_action_server
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_action REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(fastbot_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)

  ament_add_gtest(test_end_position test/test_end_position.cpp)
  ament_target_dependencies(test_end_position
    rclcpp rclcpp_action geometry_msgs nav_msgs fastbot_msgs tf2 tf2_geometry_msgs)

  ament_add_gtest(test_end_rotation test/test_end_rotation.cpp)
  ament_target_dependencies(test_end_rotation
    rclcpp rclcpp_action geometry_msgs nav_msgs fastbot_msgs tf2 tf2_geometry_msgs)
endif()


ament_package()
