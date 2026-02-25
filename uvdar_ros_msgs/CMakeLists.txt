cmake_minimum_required(VERSION 3.5)
project(uvdar_ros_msgs)

option(ENABLE_TESTS "Enable tests" OFF)


set(DEPENDENCIES
  ament_cmake
  geometry_msgs
  mrs_modules_msgs
  mrs_msgs
  sensor_msgs
  std_msgs
  rosidl_default_generators
  builtin_interfaces
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

set(MSG_FILES
  "msg/ImagePoint2dArrayStamped.msg"
  "msg/Point2d.msg"
  "msg/IdentifiedPoint3d.msg"
  "msg/IdentifiedPoint3dArrayStamped.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES 
    builtin_interfaces 
    geometry_msgs
    mrs_modules_msgs
    mrs_msgs
    sensor_msgs
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()