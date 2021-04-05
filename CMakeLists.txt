cmake_minimum_required(VERSION 3.1.2)
project(uvdar_core)

FILE(GLOB_RECURSE HEADER_FILES include *.h)
FILE(GLOB_RECURSE LAUNCHER_FILES include *.launch)
FILE(GLOB_RECURSE YAML_FILES include *.yaml)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-diagnostics-color -fmessage-length=0 -std=c++17 -Wno-return-type -Wno-unused-result -Wno-unused-variable -Wall -Wno-maybe-uninitialized -DEIGEN_INITIALIZE_MATRICES_BY_NAN -g -Ofast")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -fno-diagnostics-color -fmessage-length=0 -std=c++17 -Wno-return-type -Wno-unused-result -Wno-unused-variable -Wall -Wno-maybe-uninitialized -DEIGEN_INITIALIZE_MATRICES_BY_NAN -g -Ofast")
# set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -O1")

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
  # uvdar_gazebo_plugin
  )

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

find_package(OpenCV REQUIRED HINTS /usr/local/lib)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

add_service_files(
  FILES
  SetLedState.srv
)

add_message_files(DIRECTORY msg FILES
  FrequencySet.msg
   USM.msg
   RecMsg.msg
   DefaultMsg.msg
  )

generate_messages(DEPENDENCIES
  mrs_msgs
  # uvdar_gazebo_plugin
  std_msgs
  sensor_msgs
  )

catkin_package(
 INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs message_runtime mrs_msgs
  DEPENDS Eigen
  LIBRARIES OCamCalib unscented
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  )


add_executable(uvdar_rx_node src/uvdar_rx.cpp)
add_executable(uvdar_tx_node src/uvdar_tx.cpp)
add_executable(fr_setter_node src/fr_setter.cpp)
add_executable(new_uvdar_rx_node src/new_uvdar_rx.cpp)

add_library(OCamCalib include/OCamCalib/ocam_functions.cpp )
add_library(ht4dbt include/ht4dbt/ht4d.cpp)
add_library(uv_led_detect_fast include/detect/uv_led_detect_fast.cpp)
add_library(frequency_classifier include/frequency_classifier/frequency_classifier.cpp)
add_library(color_selector include/color_selector/color_selector.cpp)

target_link_libraries(uvdar_rx_node ${catkin_LIBRARIES})
target_link_libraries(uvdar_tx_node ${catkin_LIBRARIES})
target_link_libraries(fr_setter_node ${catkin_LIBRARIES})
target_link_libraries(new_uvdar_rx_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
 ht4dbt)
add_dependencies(new_uvdar_rx_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(uv_led_detect_fast
  debug ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

target_link_libraries(frequency_classifier
  debug ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

target_link_libraries(color_selector
  debug ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_library(UVDARDetector src/detector.cpp)
add_dependencies(UVDARDetector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(unscented include/unscented/unscented.cpp)
add_dependencies(unscented ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(p3p include/p3p/P3p.cpp)
add_dependencies(p3p ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(UVDARBlinkProcessor src/blink_processor.cpp)
add_dependencies(UVDARBlinkProcessor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(uvdar_uav_pose_calculator_node src/uav_pose_calculator.cpp)
add_dependencies(uvdar_uav_pose_calculator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(uvdar_pose_calculator_node src/pose_calculator.cpp)
add_dependencies(uvdar_pose_calculator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(uvdar_filter_node src/filter.cpp)
add_dependencies(uvdar_filter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(UVDARBluefoxEmulator src/bluefox_emulator.cpp)
add_dependencies(UVDARBluefoxEmulator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(uvdar_mask_generator_node src/mask_generator.cpp)
add_dependencies(uvdar_mask_generator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(uvdar_frequency_setter_node src/frequency_setter.cpp)
add_dependencies(uvdar_frequency_setter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_custom_target(headers SOURCES ${HEADER_FILES} ${LAUNCHER_FILES} ${YAML_FILES})

target_link_libraries(UVDARDetector
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  uv_led_detect_fast
  )

target_link_libraries(UVDARBlinkProcessor
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ht4dbt
  frequency_classifier
  color_selector
  )

target_link_libraries(uvdar_uav_pose_calculator_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  OCamCalib
  unscented
  p3p
  color_selector
  frequency_classifier
  )

target_link_libraries(uvdar_pose_calculator_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  OCamCalib
  unscented
  p3p
  color_selector
  frequency_classifier
  )

target_link_libraries(uvdar_filter_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  )

target_link_libraries(UVDARBluefoxEmulator
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  OCamCalib
  )

target_link_libraries(uvdar_mask_generator_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  )

target_link_libraries(uvdar_frequency_setter_node
  ${catkin_LIBRARIES}
  )

