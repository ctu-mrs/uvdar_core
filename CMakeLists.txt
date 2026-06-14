cmake_minimum_required(VERSION 3.14)
project(uvdar_core LANGUAGES C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(GNUInstallDirs)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

find_library(EGL_LIBRARY EGL REQUIRED)
find_library(GBM_LIBRARY gbm REQUIRED)
find_library(GLESV2_LIBRARY GLESv2 REQUIRED)

set(msg_files
    "msg/Point2DWithCovariance.msg"
    "msg/ImagePointsWithCovariancesStamped.msg"
    "msg/TrackedBlinker.msg"
    "msg/TrackerOutput.msg"
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

# Message interfaces for points and tracker outputs.
rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    DEPENDENCIES builtin_interfaces
)

# Shared helpers used by both detector and tracker nodes.
set(SHARED_SOURCES
    src/app/package_config.cpp
)

set(UTIL_SOURCES
    src/utils/thread_pool.cpp
)

# Detector sources and GPU shader objects.
set(DETECTOR_SOURCES
    src/detection/fimd/generated_module.cpp
    src/detection/fimd/cpu_detector.cpp
    src/detection/fimd/gpu_detector.cpp
    src/utils/compute_shader.cpp
    src/app/detector_node.cpp
    ${SHADER_OBJECTS}
)

# Tracker backend sources.
set(TRACKER_SOURCES
    src/tracking/ami/ami.cpp
    src/tracking/ami/blink_processor.cpp
    src/tracking/generalized/generalized_tracker.cpp
    src/tracking/generalized/blink_processor.cpp
    src/app/tracker_node.cpp
)

add_library(${PROJECT_NAME}_common STATIC ${SHARED_SOURCES})
add_library(${PROJECT_NAME}_utils STATIC ${UTIL_SOURCES})
add_library(${PROJECT_NAME}_detector STATIC ${DETECTOR_SOURCES})
add_library(${PROJECT_NAME}_tracker STATIC ${TRACKER_SOURCES})

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
target_include_directories(${PROJECT_NAME}_utils PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_include_directories(${PROJECT_NAME}_detector PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME}_tracker PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Common support.
target_link_libraries(${PROJECT_NAME}_common
    yaml-cpp::yaml-cpp
)

target_link_libraries(${PROJECT_NAME}_utils
    ${PROJECT_NAME}_common
)

target_link_libraries(${PROJECT_NAME}_detector
    ${PROJECT_NAME}_common
    ${PROJECT_NAME}_utils
    ${EGL_LIBRARY}
    ${GBM_LIBRARY}
    ${GLESV2_LIBRARY}
    ${OpenCV_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_tracker
    ${PROJECT_NAME}_common
    ${PROJECT_NAME}_utils
    ${OpenCV_LIBRARIES}
    Eigen3::Eigen
)

ament_target_dependencies(${PROJECT_NAME}_detector
    rclcpp
    sensor_msgs
    cv_bridge
    builtin_interfaces
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
add_executable(tracker_node
    src/app/main_tracker_node.cpp
)

target_link_libraries(detector_node
    ${PROJECT_NAME}_detector
)
target_link_libraries(tracker_node
    ${PROJECT_NAME}_tracker
)

ament_target_dependencies(detector_node
    rclcpp
)

ament_target_dependencies(tracker_node
    rclcpp
)

install(TARGETS
    ${PROJECT_NAME}_common
    ${PROJECT_NAME}_utils
    ${PROJECT_NAME}_detector
    ${PROJECT_NAME}_tracker
    detector_node
    tracker_node
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
