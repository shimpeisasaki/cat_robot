cmake_minimum_required(VERSION 3.8)
project(cat_panorama_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CapturePanorama.action"
  "srv/StartCapture.srv"
  "msg/CapturedFrame.msg"
  DEPENDENCIES sensor_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
