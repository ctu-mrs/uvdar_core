cmake_minimum_required(VERSION 3.5)
project(uvdar_ros_msgs)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fno-omit-frame-pointer)
endif()

if(NOT MRS_ENABLE_TESTING)
  message(WARNING "relwithdebinfo profile detected, building with -O3")
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -O3")
endif()

if(COVERAGE)
  message(WARNING "building with --coverage, the performance might be limited")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
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
  "msg/DefaultMsg.msg"
  "msg/FrequencySet.msg"
  "msg/ImagePointsWithFloatStamped.msg"
  "msg/Int32MultiArrayStamped.msg"
  "msg/Point2DWithFloat.msg"
  "msg/RecMsg.msg"
  "msg/USM.msg"
)

set(AMI_MSG_FILES
  "msg/ami/AMIAllSequences.msg"
  "msg/ami/AMIDataForLogging.msg"
  "msg/ami/AMISeqPoint.msg"
  "msg/ami/AMISeqVariables.msg"
)

set(SRV_FILES
  "srv/SetInts.srv"
  "srv/SetLedMessage.srv"
)


rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${AMI_MSG_FILES}
  ${SRV_FILES}
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