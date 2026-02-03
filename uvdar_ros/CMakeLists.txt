cmake_minimum_required(VERSION 3.5)
project(uvdar_ros)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fno-omit-frame-pointer)
endif()

# if(NOT MRS_ENABLE_TESTING)
#   message(WARNING "relwithdebinfo profile detected, building with -O3")
#   set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -O3")
# endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG" CACHE STRING "" FORCE)


if(COVERAGE)
  message(WARNING "building with --coverage, the performance might be limited")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
endif()

set(DEPENDENCIES
  ament_cmake
  rclcpp
  rclcpp_components
  cv_bridge
  image_transport
  mrs_lib
  mrs_modules_msgs
  mrs_msgs
  sensor_msgs
  std_msgs
  Eigen3
  tf2_eigen
  uvdar
  uvdar_ros_msgs
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

## | --------------------- UvdarRosWrapper_UvLedDetectorRos --------------------- |

add_library(UvdarRosWrapper_UvLedDetectorRos SHARED
  src/utils/ros_logger.cpp
  src/uv_led_detector_ros.cpp
)

ament_target_dependencies(UvdarRosWrapper_UvLedDetectorRos
  ${DEPENDENCIES}
)

target_compile_definitions(UvdarRosWrapper_UvLedDetectorRos
  PRIVATE
    $<$<CONFIG:Debug>:DEBUG>
)

 target_include_directories(UvdarRosWrapper_UvLedDetectorRos
    PUBLIC
      $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )

rclcpp_components_register_nodes(UvdarRosWrapper_UvLedDetectorRos "uvdar::UvLedDetectorComponent")

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(
  TARGETS
    UvdarRosWrapper_UvLedDetectorRos
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()