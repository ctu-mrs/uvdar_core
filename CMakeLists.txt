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
  UvdarCore_OCamCalib
  UvdarCore_ht4dbt
  UvdarCore_omta
  UvdarCore_extendedSearch
  UvdarCore_uv_led_detect_fast
  UvdarCore_frequency_classifier
  UvdarCore_color_selector
  UvdarCore_UVDARDetector
  UvdarCore_unscented
  UvdarCore_p3p
  UvdarCore_UVDARBlinkProcessor
  UvdarCore_UVDARBluefoxEmulator
  )

set(EXECUTABLES
  uvdar_rx_node
  uvdar_tx_node
  new_uvdar_rx_node
  uvdar_led_manager_node
  uvdar_filter_node
  uvdar_pose_calculator_node
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
  UvdarCore_ht4dbt
  )

add_dependencies(new_uvdar_rx_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

## | ------------------------ OCamCalib ----------------------- |

add_library(UvdarCore_OCamCalib
  include/OCamCalib/ocam_functions.cpp
  )

target_link_libraries(UvdarCore_OCamCalib
  ${catkin_LIBRARIES}
  )

add_dependencies(UvdarCore_OCamCalib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_compile_options(UvdarCore_OCamCalib PRIVATE
  -Wno-unused-variable
  -Wno-unused-result
  )

## | ------------------------- UvdarCore_ht4dbt ------------------------- |

add_library(UvdarCore_ht4dbt
  include/ht4dbt/ht4d.cpp
  )

add_dependencies(UvdarCore_ht4dbt
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(UvdarCore_ht4dbt
  ${catkin_LIBRARIES}
  )

## | -------------------------- UvdarCore_omta -------------------------- |

add_library(UvdarCore_omta
  include/omta/omta.cpp
  )

add_dependencies(UvdarCore_omta
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(UvdarCore_omta
  ${catkin_LIBRARIES}
  )

## | --------------------- extended Search -------------------- |

add_library(UvdarCore_extendedSearch
  include/omta/extended_search.cpp
  )

add_dependencies(UvdarCore_extendedSearch
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(UvdarCore_extendedSearch
  ${catkin_LIBRARIES}
  )

## | ------------------- uv led detect fast ------------------- |

add_library(UvdarCore_uv_led_detect_fast
  include/detect/uv_led_detect_fast.cpp
  )

add_dependencies(UvdarCore_uv_led_detect_fast
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(UvdarCore_uv_led_detect_fast
  debug
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  )

## | ------------------ frequency classifier ------------------ |

add_library(UvdarCore_frequency_classifier
  include/frequency_classifier/frequency_classifier.cpp
  )

add_dependencies(UvdarCore_frequency_classifier
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(UvdarCore_frequency_classifier
  debug
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  )

## | --------------------- color selector --------------------- |

add_library(UvdarCore_color_selector
  include/color_selector/color_selector.cpp
  )

add_dependencies(UvdarCore_color_selector
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(UvdarCore_color_selector
  debug
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  )

## | --------------------- uvdar detector --------------------- |

add_library(UvdarCore_UVDARDetector
  src/detector.cpp
  )

add_dependencies(UvdarCore_UVDARDetector
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(UvdarCore_UVDARDetector
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  UvdarCore_uv_led_detect_fast
  )

## | ------------------------ UvdarCore_unscented ----------------------- |

add_library(UvdarCore_unscented
  include/unscented/unscented.cpp
  )

add_dependencies(UvdarCore_unscented
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(UvdarCore_unscented
  ${catkin_LIBRARIES}
  )

## | --------------------------- UvdarCore_p3p -------------------------- |

add_library(UvdarCore_p3p
  include/p3p/P3p.cpp
  )

add_dependencies(UvdarCore_p3p
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(UvdarCore_p3p
  ${catkin_LIBRARIES}
  )

## | ------------------ uvdar blnk processor ------------------ |

add_library(UvdarCore_UVDARBlinkProcessor
  src/blink_processor.cpp
  )

add_dependencies(UvdarCore_UVDARBlinkProcessor
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(UvdarCore_UVDARBlinkProcessor
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  UvdarCore_ht4dbt
  UvdarCore_frequency_classifier
  UvdarCore_omta
  UvdarCore_extendedSearch
  UvdarCore_color_selector
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
  UvdarCore_OCamCalib
  UvdarCore_unscented
  UvdarCore_p3p
  UvdarCore_color_selector
  UvdarCore_frequency_classifier
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

add_library(UvdarCore_UVDARBluefoxEmulator
  src/bluefox_emulator.cpp
  )

add_dependencies(UvdarCore_UVDARBluefoxEmulator
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(UvdarCore_UVDARBluefoxEmulator
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  UvdarCore_OCamCalib
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

# add_custom_target(headers
#   SOURCES ${HEADER_FILES} ${LAUNCHER_FILES} ${YAML_FILES}
#   )

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(TARGETS ${EXECUTABLES}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY msg srv
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY scripts/
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY scripts
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
