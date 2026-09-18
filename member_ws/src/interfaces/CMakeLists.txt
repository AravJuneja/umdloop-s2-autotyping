cmake_minimum_required(VERSION 3.8)
project(interfaces)
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObservationStatus.msg"
  "msg/JointObservation.msg"
  "msg/ImageObservation.msg"
  "msg/CalibrationObservation.msg"
  "msg/LaunchKeyObservation.msg"
  "msg/Transform.msg"
  "msg/TransformObservation.msg"
  "msg/RobotState.msg"
  "msg/Pixel.msg"
  "msg/MarkerDetection.msg"
  "msg/MarkerDetections.msg"
  "srv/GetRobotState.srv"
  DEPENDENCIES builtin_interfaces
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
