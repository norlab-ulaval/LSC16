cmake_minimum_required(VERSION 3.8)
project(lslidar_c16_decoder)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(lslidar_c16_msgs REQUIRED)

include_directories(
  include
)

add_library(lslidar_c16_decoder src/lslidar_c16_decoder.cpp)
ament_target_dependencies(lslidar_c16_decoder
  rclcpp
  sensor_msgs
  std_msgs
  pcl_conversions
  pcl_ros
  lslidar_c16_msgs
)

add_executable(lslidar_c16_decoder_node src/lslidar_c16_decoder_node.cpp)
target_link_libraries(lslidar_c16_decoder_node
  lslidar_c16_decoder
)

install(TARGETS lslidar_c16_decoder_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

ament_package()
