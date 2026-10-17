cmake_minimum_required(VERSION 3.8)
project(wall_pose_estimation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Core algorithm: plain C++, no ROS dependency, so it can be unit tested
add_library(wall_estimator src/wall_estimator.cpp)
target_include_directories(wall_estimator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# ROS 2 node
add_executable(wall_pose_estimator_node src/wall_pose_estimator_node.cpp)
target_link_libraries(wall_pose_estimator_node wall_estimator)
ament_target_dependencies(wall_pose_estimator_node rclcpp sensor_msgs geometry_msgs std_msgs)

install(TARGETS wall_estimator wall_pose_estimator_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_wall_estimator test/test_wall_estimator.cpp)
  target_link_libraries(test_wall_estimator wall_estimator)
endif()

ament_package()
