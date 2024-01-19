cmake_minimum_required(VERSION 3.5)
project(uvdar_core)

# FILE(GLOB_RECURSE HEADER_FILES include *.h)
# FILE(GLOB_RECURSE LAUNCHER_FILES include *.launch)
# FILE(GLOB_RECURSE YAML_FILES include *.yaml)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall)
add_definitions(-Wextra)

if(NOT TESTING)
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -O3")
endif()

set(CATKIN_DEPENDENCIES
  cv_bridge
  image_transport
  message_generation
  message_runtime
  mrs_lib
  mrs_modules_msgs
  mrs_msgs
  nodelet
  roscpp
  sensor_msgs
  std_msgs
  )

set(LIBRARIES
  OCamCalib
  ht4dbt
  omta
  extendedSearch
  uv_led_detect_fast
  frequency_classifier
  color_selector
  UVDARDetector
  unscented
  p3p
  UVDARBlinkProcessor
  UVDARBluefoxEmulator
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

find_package(OpenCV REQUIRED HINTS /usr/local/lib)

add_service_files(
  FILES
  SetLedMessage.srv
  SetInts.srv
  # SetIntIndex.srv
  # SetLedState.srv
  )

add_message_files(DIRECTORY msg FILES
  FrequencySet.msg
  USM.msg
  RecMsg.msg
  DefaultMsg.msg
  omtaDataForLogging.msg
  omtaSeqVariables.msg
  omtaAllSequences.msg
  omtaSeqPoint.msg
  ImagePointsWithFloatStamped.msg
  Point2DWithFloat.msg
  Int32MultiArrayStamped.msg
  )

generate_messages(DEPENDENCIES
  mrs_msgs
  std_msgs
  sensor_msgs
  # uvdar_gazebo_plugin
  )

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  LIBRARIES ${LIBRARIES}
  DEPENDS Eigen
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  )

## | ---------------------- uvdar rx node --------------------- |

add_executable(uvdar_rx_node
  src/uvdar_rx.cpp
  )

target_link_libraries(uvdar_rx_node
  ${catkin_LIBRARIES}
  )

add_dependencies(uvdar_rx_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

## | ---------------------- uvdar tx node --------------------- |

add_executable(uvdar_tx_node
  src/uvdar_tx.cpp
  )

target_link_libraries(uvdar_tx_node
  ${catkin_LIBRARIES}
  )

add_dependencies(uvdar_tx_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

## | -------------------- new uvdar rx node ------------------- |

add_executable(new_uvdar_rx_node
  src/new_uvdar_rx.cpp
  )

target_link_libraries(new_uvdar_rx_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ht4dbt
  )

add_dependencies(new_uvdar_rx_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

## | ------------------------ OCamCalub ----------------------- |

add_library(OCamCalib
  include/OCamCalib/ocam_functions.cpp
  )

target_link_libraries(OCamCalib
  ${catkin_LIBRARIES}
  )

add_dependencies(OCamCalib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_compile_options(OCamCalib PRIVATE
  -Wno-unused-variable
  -Wno-unused-result
  )

## | ------------------------- ht4dbt ------------------------- |

add_library(ht4dbt
  include/ht4dbt/ht4d.cpp
  )

add_dependencies(ht4dbt
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(ht4dbt
  ${catkin_LIBRARIES}
  )

## | -------------------------- omta -------------------------- |

add_library(omta
  include/omta/omta.cpp
  )

add_dependencies(omta
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(omta
  ${catkin_LIBRARIES}
  )

## | --------------------- extended Search -------------------- |

add_library(extendedSearch
  include/omta/extended_search.cpp
  )

add_dependencies(extendedSearch
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(extendedSearch
  ${catkin_LIBRARIES}
  )

## | ------------------- uv led detect fast ------------------- |

add_library(uv_led_detect_fast
  include/detect/uv_led_detect_fast.cpp
  )

add_dependencies(uv_led_detect_fast
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(uv_led_detect_fast
  debug
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  )

## | ------------------ frequency classifier ------------------ |

add_library(frequency_classifier
  include/frequency_classifier/frequency_classifier.cpp
  )

add_dependencies(frequency_classifier
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(frequency_classifier
  debug
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  )

## | --------------------- color selector --------------------- |

add_library(color_selector
  include/color_selector/color_selector.cpp
  )

add_dependencies(color_selector
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(color_selector
  debug
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  )

## | --------------------- uvdar detector --------------------- |

add_library(UVDARDetector
  src/detector.cpp
  )

add_dependencies(UVDARDetector
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(UVDARDetector
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  uv_led_detect_fast
  )

## | ------------------------ unscented ----------------------- |

add_library(unscented
  include/unscented/unscented.cpp
  )

add_dependencies(unscented
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(unscented
  ${catkin_LIBRARIES}
  )

## | --------------------------- p3p -------------------------- |

add_library(p3p
  include/p3p/P3p.cpp
  )

add_dependencies(p3p
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(p3p
  ${catkin_LIBRARIES}
  )

## | ------------------ uvdar blnk processor ------------------ |

add_library(UVDARBlinkProcessor
  src/blink_processor.cpp
  )

add_dependencies(UVDARBlinkProcessor
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(UVDARBlinkProcessor
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ht4dbt
  frequency_classifier
  omta
  extendedSearch
  color_selector
  )

## | --------------- uvdar pose calculator node --------------- |

add_executable(uvdar_pose_calculator_node
  src/pose_calculator.cpp
  )

add_dependencies(uvdar_pose_calculator_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
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

## | -------------------- uvdar filter node ------------------- |

add_executable(uvdar_filter_node
  src/filter.cpp
  )

add_dependencies(uvdar_filter_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(uvdar_filter_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  )

## | ----------------- uvdar bluefox emulator ----------------- |

add_library(UVDARBluefoxEmulator
  src/bluefox_emulator.cpp
  )

add_dependencies(UVDARBluefoxEmulator
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(UVDARBluefoxEmulator
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  OCamCalib
  )

## | ---------------- uvdar mask generator node --------------- |

add_executable(uvdar_mask_generator_node
  src/mask_generator.cpp
  )

add_dependencies(uvdar_mask_generator_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(uvdar_mask_generator_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  )

## | ----------------- uvdar led manager node ----------------- |

add_executable(uvdar_led_manager_node
  src/led_manager.cpp
  )

add_dependencies(uvdar_led_manager_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(uvdar_led_manager_node
  ${catkin_LIBRARIES}
  )

## | ---------------------- what is this? --------------------- |

add_custom_target(headers
  SOURCES ${HEADER_FILES} ${LAUNCHER_FILES} ${YAML_FILES}
  )
