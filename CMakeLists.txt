cmake_minimum_required(VERSION 3.8)
project(hands_on_kitti)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_transport REQUIRED)

add_executable(listener src/subscriber.cpp)
# target_link_libraries(listener PUBLIC rclcpp::rclcpp ${std_msgs_TARGETS})

ament_target_dependencies(listener
  rclcpp
  std_msgs
  sensor_msgs
  rosbag2_cpp
  rosbag2_storage
  rosbag2_transport
)

install(TARGETS
  listener
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
