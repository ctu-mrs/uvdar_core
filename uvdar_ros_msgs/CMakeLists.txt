cmake_minimum_required(VERSION 3.5)
project(uvdar_ros_msgs)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fno-omit-frame-pointer)
endif()

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