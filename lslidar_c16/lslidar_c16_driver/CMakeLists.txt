cmake_minimum_required(VERSION 3.8)
project(lslidar_c16_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(lslidar_c16_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)

include_directories(
  include
)

add_library(lslidar_c16_driver src/lslidar_c16_driver.cc)
ament_target_dependencies(lslidar_c16_driver
  rclcpp
  tf2_ros
  lslidar_c16_msgs
  diagnostic_updater
)

add_executable(lslidar_c16_driver_node src/lslidar_c16_driver_node.cc)
target_link_libraries(lslidar_c16_driver_node
  lslidar_c16_driver
)

install(TARGETS lslidar_c16_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

ament_package()
