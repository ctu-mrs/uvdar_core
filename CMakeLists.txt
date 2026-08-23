cmake_minimum_required(VERSION 3.14)
project(uvdar_core LANGUAGES C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(GNUInstallDirs)
include(FetchContent)

FetchContent_Declare(
  cvplot
  GIT_REPOSITORY https://github.com/Profactor/cv-plot.git
  GIT_TAG v1.2.2
)
FetchContent_GetProperties(cvplot)
if(NOT cvplot_POPULATED)
  FetchContent_Populate(cvplot)
endif()
add_library(cvplot INTERFACE)
target_include_directories(cvplot INTERFACE "${cvplot_SOURCE_DIR}/CvPlot/inc")
target_compile_definitions(cvplot INTERFACE CVPLOT_HEADER_ONLY)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mrs_msgs REQUIRED)
find_package(mrs_modules_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

find_library(EGL_LIBRARY EGL REQUIRED)
find_library(GBM_LIBRARY gbm REQUIRED)
find_library(GLESV2_LIBRARY GLESv2 REQUIRED)

set(msg_files
    "msg/Point2DWithCovariance.msg"
    "msg/ImagePointsWithCovariancesStamped.msg"
    "msg/TrackedBlinker.msg"
    "msg/TrackerOutput.msg"
    "msg/PoseWithCovarianceIdentified.msg"
    "msg/PoseWithCovarianceArrayStamped.msg"
)

set(srv_files
    "srv/SetInts.srv"
    "srv/SetLedMessage.srv"
)

file(GLOB_RECURSE SHADER_FILES_PATHS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/shaders/*.comp")
set(SHADER_OBJECTS)
foreach(SHADER_FILE_PATH ${SHADER_FILES_PATHS})
    file(RELATIVE_PATH SHADER_FILE ${CMAKE_CURRENT_SOURCE_DIR} ${SHADER_FILE_PATH})
    get_filename_component(SHADER_NAME_WE ${SHADER_FILE} NAME_WE)
    set(SHADER_OBJ "${CMAKE_CURRENT_BINARY_DIR}/${SHADER_NAME_WE}.o")
    add_custom_command(
        OUTPUT ${SHADER_OBJ}
        COMMAND ld -r -z noexecstack -b binary -o ${SHADER_OBJ} ${SHADER_FILE}
        DEPENDS ${SHADER_FILE_PATH}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        VERBATIM
    )
    list(APPEND SHADER_OBJECTS ${SHADER_OBJ})
endforeach()

# Message and service interfaces for points, tracker outputs and LED manager control.
rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    ${srv_files}
    DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

# Shared helpers used by both detector and tracker nodes.
set(SHARED_SOURCES
    src/app/package_config.cpp
    src/helpers/thread_pool.cpp
    src/app/visualization.cpp
)

# Detector sources and GPU shader objects.
set(DETECTOR_SOURCES
    src/detection/fimd/generated_module.cpp
    src/detection/fimd/cpu_detector.cpp
    src/detection/fimd/gpu_detector.cpp
    src/helpers/compute_shader.cpp
    src/app/detector_node.cpp
    ${SHADER_OBJECTS}
)

# Tracker backend sources.
set(TRACKER_SOURCES
    src/tracking/ami/ami.cpp
    src/tracking/generalized/generalized_tracker.cpp
    src/app/tracker_node.cpp
)

set(CALIBRATION_SOURCES
    src/calibration/lens_model_loader.cpp
    src/calibration/fisheye/ocam_model.cpp
    src/calibration/pinhole/pinhole_model.cpp
    src/calibration/fisheye/equidistant_model.cpp
    src/calibration/fisheye/radial_model.cpp
)

set(CALIBRATOR_SOURCES
    src/calibration/calibrator.cpp
    src/calibration/grid_extractor.cpp
    src/calibration/pattern_detector.cpp
    src/calibration/intermediate_dataset.cpp
    src/calibration/calibration_visualization.cpp
    src/app/calibrator_node.cpp
)

set(POSE_ESTIMATION_SOURCES
    src/pose_estimation/body_model.cpp
    src/pose_estimation/uncertainty.cpp
    src/pose_estimation/geometric_solver/geometric_solver.cpp
    src/pose_estimation/particle_filter/reprojection_model.cpp
    src/pose_estimation/particle_filter/particle_filter.cpp
    src/pose_estimation/particle_filter/sampling.cpp
    src/pose_estimation/kf_pose.cpp
)

add_library(${PROJECT_NAME}_common STATIC ${SHARED_SOURCES})
add_library(${PROJECT_NAME}_calibration STATIC ${CALIBRATION_SOURCES})
add_library(${PROJECT_NAME}_calibrator STATIC ${CALIBRATOR_SOURCES})
add_library(${PROJECT_NAME}_detector STATIC ${DETECTOR_SOURCES})
add_library(${PROJECT_NAME}_tracker STATIC ${TRACKER_SOURCES})
add_library(${PROJECT_NAME}_pose_estimation STATIC ${POSE_ESTIMATION_SOURCES})

set(UVDAR_FIMD_GENERATOR_CFLAGS
    "-shared -fPIC -O3 -DNDEBUG -march=native -mtune=native -fomit-frame-pointer -fno-semantic-interposition"
)

target_compile_definitions(${PROJECT_NAME}_detector PRIVATE
    "UVDAR_FIMD_GENERATOR_COMPILER=\"$<SHELL_PATH:${CMAKE_C_COMPILER}>\""
    "UVDAR_FIMD_GENERATOR_CFLAGS=\"${UVDAR_FIMD_GENERATOR_CFLAGS}\""
)

# Shared include directories.
target_include_directories(${PROJECT_NAME}_common PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME}_calibration PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME}_calibrator PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME}_detector PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME}_tracker PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME}_pose_estimation PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# Common support.
target_link_libraries(${PROJECT_NAME}_common
    cvplot
    yaml-cpp::yaml-cpp
    ${OpenCV_LIBRARIES}
    Eigen3::Eigen
)

target_link_libraries(${PROJECT_NAME}_calibration
    Eigen3::Eigen
    yaml-cpp::yaml-cpp
)

target_link_libraries(${PROJECT_NAME}_calibrator
    ${PROJECT_NAME}_common
    ${PROJECT_NAME}_calibration
    ${PROJECT_NAME}_detector
    ${OpenCV_LIBRARIES}
    Eigen3::Eigen
    yaml-cpp::yaml-cpp
)

target_link_libraries(${PROJECT_NAME}_detector
    ${PROJECT_NAME}_common
    ${EGL_LIBRARY}
    ${GBM_LIBRARY}
    ${GLESV2_LIBRARY}
    ${OpenCV_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_tracker
    ${PROJECT_NAME}_common
    ${OpenCV_LIBRARIES}
    Eigen3::Eigen
)
target_link_libraries(${PROJECT_NAME}_pose_estimation
    Eigen3::Eigen
)
ament_target_dependencies(${PROJECT_NAME}_detector
    rclcpp
    sensor_msgs
    cv_bridge
    builtin_interfaces
)

ament_target_dependencies(${PROJECT_NAME}_calibrator
    rclcpp
    sensor_msgs
    std_msgs
    cv_bridge
)

ament_target_dependencies(${PROJECT_NAME}_tracker
    rclcpp
    sensor_msgs
    cv_bridge
    std_msgs
    builtin_interfaces
)
# ROS 2 message type-support is required by executables using interface types.
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(${PROJECT_NAME}_detector ${cpp_typesupport_target})
target_link_libraries(${PROJECT_NAME}_tracker ${cpp_typesupport_target})

ament_export_dependencies(rosidl_default_runtime)

# Executables.
add_executable(detector_node
    src/app/main_detector_node.cpp
)
add_executable(calibrator_node
    src/app/main_calibrator_node.cpp
)
add_executable(tracker_node
    src/app/main_tracker_node.cpp
)
add_executable(pose_estimator_node
    src/app/main_pose_estimator_node.cpp
    src/app/pose_estimator_node.cpp
)
add_executable(filter_node
    src/app/main_filter_node.cpp
    src/app/filter_node.cpp
)
add_executable(led_manager_node
    src/app/main_led_manager_node.cpp
    src/app/led_manager_node.cpp
)

target_link_libraries(detector_node
    ${PROJECT_NAME}_detector
)
target_link_libraries(calibrator_node
    ${PROJECT_NAME}_calibrator
)
target_link_libraries(tracker_node
    ${PROJECT_NAME}_tracker
)
target_link_libraries(pose_estimator_node
    ${PROJECT_NAME}_common
    ${PROJECT_NAME}_pose_estimation
    ${PROJECT_NAME}_calibration
    yaml-cpp::yaml-cpp
    ${OpenCV_LIBRARIES}
)
target_link_libraries(filter_node
    ${PROJECT_NAME}_pose_estimation
    yaml-cpp::yaml-cpp
)
target_link_libraries(pose_estimator_node ${cpp_typesupport_target})
target_link_libraries(filter_node ${cpp_typesupport_target})
target_link_libraries(led_manager_node ${cpp_typesupport_target})

target_include_directories(led_manager_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(detector_node
    rclcpp
)

ament_target_dependencies(calibrator_node
    rclcpp
)

ament_target_dependencies(tracker_node
    rclcpp
)
ament_target_dependencies(pose_estimator_node
    rclcpp
    builtin_interfaces
    geometry_msgs
    nav_msgs
    std_msgs
    sensor_msgs
    cv_bridge
    tf2
    tf2_ros
)
ament_target_dependencies(filter_node
    rclcpp
    builtin_interfaces
    geometry_msgs
    tf2
    tf2_ros
)
ament_target_dependencies(led_manager_node
    rclcpp
    std_srvs
    mrs_msgs
    mrs_modules_msgs
)

install(TARGETS
    ${PROJECT_NAME}_common
    ${PROJECT_NAME}_calibration
    ${PROJECT_NAME}_calibrator
    ${PROJECT_NAME}_detector
    ${PROJECT_NAME}_tracker
    ${PROJECT_NAME}_pose_estimation
    detector_node
    calibrator_node
    tracker_node
    pose_estimator_node
    filter_node
    led_manager_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include
)
install(DIRECTORY launch config shaders
    DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
