cmake_minimum_required(VERSION 2.8.3)
project(uvdar)

FILE(GLOB_RECURSE HEADER_FILES include *.h)
FILE(GLOB_RECURSE LAUNCHER_FILES include *.launch)
FILE(GLOB_RECURSE YAML_FILES include *.yaml)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-diagnostics-color -fmessage-length=0 -std=c++17 -Wno-return-type -Wno-unused-result -Wno-unused-variable -Wall -Wno-maybe-uninitialized -DEIGEN_INITIALIZE_MATRICES_BY_NAN")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  cv_bridge
  image_transport
  sensor_msgs
  std_msgs
  mrs_msgs
  mrs_lib
  tf2
  tf2_ros
  tf2_msgs
  tf2_eigen
  tf2_sensor_msgs
  tf_conversions
  tf2_eigen
  tf2_geometry_msgs
  )

# find_package(Eigen3)
# if(NOT EIGEN3_FOUND)
#   # Fallback to cmake_modules
#   find_package(cmake_modules REQUIRED)
#   find_package(Eigen REQUIRED)
#   set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
#   set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
# endif()

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})
message(${EIGEN3_INCLUDE_DIRS})

find_package(OpenCV REQUIRED HINTS /usr/local/lib)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

add_message_files(DIRECTORY msg FILES
  ROIVector.msg
  Int32MultiArrayStamped.msg
  DistRange.msg
  DistRangeVector.msg
  FrequencySet.msg
  )

generate_messages(DEPENDENCIES
  std_msgs
  sensor_msgs
  )

catkin_package(
 INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs message_runtime mrs_msgs
  DEPENDS Eigen
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  #${OpenCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  )

add_library(OCamCalib include/OCamCalib/ocam_functions.cpp )

add_library(ht3dbt include/ht3dbt/ht3d.cpp)
add_library(ht4dbt include/ht3dbt/ht4d.cpp)
add_library(uv_led_detect_fast include/detect/uv_led_detect_fast.cpp)

target_link_libraries(uv_led_detect_fast 
  debug ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_library(uvdar_detector src/detector.cpp)
add_dependencies(uvdar_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(unscented include/unscented/unscented.cpp)
add_dependencies(unscented ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(p3p include/p3p/P3p.cpp)
add_dependencies(p3p ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(uvdar_blink_processor src/blink_processor.cpp)
add_dependencies(uvdar_blink_processor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(uv_follower_node src/follower.cpp include/slider/slider.cpp)
# add_dependencies(uv_follower_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(uvdar_pose_reporter_node src/uav_pose_calculator.cpp)
add_dependencies(uvdar_pose_reporter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(uvdar_kalman_node src/filter.cpp)
add_dependencies(uvdar_kalman_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(uvdar_kalman_anonymous_node src/filter_anonymous.cpp)
add_dependencies(uvdar_kalman_anonymous_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_executable(uvdar_orientation_calibrator src/orientationCalibrator.cpp)
# add_dependencies(uvdar_orientation_calibrator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(uvdar_reprojector_node src/reprojector.cpp)
add_dependencies(uvdar_reprojector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(uvdar_bluefox_emulator src/bluefox_emulator.cpp)
add_dependencies(uvdar_bluefox_emulator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(uvdar_mask_generator_node src/mask_generator.cpp)
add_dependencies(uvdar_mask_generator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(uvdar_frequency_setter_node src/frequency_setter.cpp)
add_dependencies(uvdar_frequency_setter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_custom_target(headers SOURCES ${HEADER_FILES} ${LAUNCHER_FILES} ${YAML_FILES})

target_link_libraries(uvdar_detector
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  uv_led_detect_fast
  )

target_link_libraries(uvdar_blink_processor
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ht4dbt
  )

# target_link_libraries(uv_follower_node
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   ${EIGEN3_LIBRARIES}
#   OCamCalib
#   unscented
#   )

target_link_libraries(uvdar_pose_reporter_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  OCamCalib
  unscented
  p3p
  stdc++fs
  )

target_link_libraries(uvdar_kalman_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  )

target_link_libraries(uvdar_kalman_anonymous_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  )

# target_link_libraries(uvdar_orientation_calibrator
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   ${EIGEN3_LIBRARIES}
#   )

target_link_libraries(uvdar_bluefox_emulator
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  OCamCalib
  )


target_link_libraries(uvdar_reprojector_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  OCamCalib
  unscented
  )

target_link_libraries(uvdar_mask_generator_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  )

target_link_libraries(uvdar_frequency_setter_node
  ${catkin_LIBRARIES}
  )

