cmake_minimum_required(VERSION 3.8)
project(lslidar_c16_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs)
find_package(sensor_msgs)
find_package(builtin_interfaces)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LslidarC16Layer.msg"
  "msg/LslidarC16Packet.msg"
  "msg/LslidarC16Point.msg"
  "msg/LslidarC16Scan.msg"
  "msg/LslidarC16Sweep.msg"
  "msg/LslidarC16ScanUnified.msg"
  DEPENDENCIES std_msgs sensor_msgs builtin_interfaces
)

ament_package()
