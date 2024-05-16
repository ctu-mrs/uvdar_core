cmake_minimum_required(VERSION 3.5)
project(uvdar_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall)
add_definitions(-Wextra)
add_definitions(-fno-omit-frame-pointer)
# add_definitions(-O0)

if(NOT MRS_ENABLE_TESTING)
  message(WARNING "relwithdebinfo profile detected, building with -O3")
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -O3")
endif()

if(COVERAGE)
  message(WARNING "building with --coverage, the performance might be limited")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
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
  UvdarCore_ami
  UvdarCore_extendedSearch
  UvdarCore_uv_led_detect_fast
  UvdarCore_frequency_classifier
  UvdarCore_color_selector
  UvdarCore_UVDARDetector
  UvdarCore_unscented
  UvdarCore_UVDARBlinkProcessor
  UvdarCore_UVDARBluefoxEmulator
  UvdarCore_compute_lib
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

set(OpenGL_GL_PREFERENCE "LEGACY")
find_package(OpenGL REQUIRED)

# find_package(LIBGBM REQUIRED)
# # pkg_check_modules(PC_LIBGBM gbm)

# find_path(LIBGBM_INCLUDE_DIRS
#     NAMES gbm.h
#     HINTS ${PC_LIBGBM_INCLUDE_DIRS} ${PC_LIBGBM_INCUDEDIR}
# )

find_library(LGBM libgbm.so REQUIRED)
find_library(LEGL libEGL.so REQUIRED)
find_library(LGL libGL.so REQUIRED)

include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(LIBGBM DEFAULT_MSG LIBGBM_LIBRARIES)

mark_as_advanced(LIBGBM_INCLUDE_DIRS LIBGBM_LIBRARIES)

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
  AMIDataForLogging.msg
  AMISeqVariables.msg
  AMIAllSequences.msg
  AMISeqPoint.msg
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
  ${OpenGL_INCLUDE_DIRS}
  ${LIBGBM_INCLUDE_DIRS}
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
  include/ht4dbt/ht4d_cpu.cpp
  include/ht4dbt/ht4d_cpu.cpp
  )

add_dependencies(UvdarCore_ht4dbt
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(UvdarCore_ht4dbt
  ${catkin_LIBRARIES}
  UvdarCore_compute_lib
  )

## | -------------------------- UvdarCore_ami -------------------------- |

add_library(UvdarCore_ami
  include/ami/ami.cpp
  )

add_dependencies(UvdarCore_ami
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(UvdarCore_ami
  ${catkin_LIBRARIES}
  )

## | --------------------- extended Search -------------------- |

add_library(UvdarCore_extendedSearch
  include/ami/ami_extended_search.cpp
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
  include/detect/uv_led_detect_fast_cpu.cpp
  include/detect/uv_led_detect_fast_gpu.cpp
  )

add_dependencies(UvdarCore_uv_led_detect_fast
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(UvdarCore_uv_led_detect_fast
  debug
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  UvdarCore_compute_lib
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

## | --------------------- compute lib --------------------- |

add_library(UvdarCore_compute_lib
  include/compute_lib/compute_lib.c
  include/compute_lib/queue.c
  include/compute_lib/gl32_utils.c
  )

add_dependencies(UvdarCore_compute_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(UvdarCore_compute_lib
  debug
  ${OpenGL_LIBRARIES}
  ${LIBGBM_LIBRARIES}
  ${catkin_LIBRARIES}
  ${LGBM}
  ${LEGL}
  ${LGL}
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
  UvdarCore_ami
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
