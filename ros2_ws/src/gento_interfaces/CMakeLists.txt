cmake_minimum_required(VERSION 3.8)
project(gento_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DriverQuery.srv" "action/ExecuteTrajectory.action"
  DEPENDENCIES sensor_msgs trajectory_msgs geometry_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
