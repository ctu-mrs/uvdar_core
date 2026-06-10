cmake_minimum_required(VERSION 3.14)
project(uvdar_core LANGUAGES C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(GNUInstallDirs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

find_library(EGL_LIBRARY EGL REQUIRED)
find_library(GBM_LIBRARY gbm REQUIRED)
find_library(GLESV2_LIBRARY GLESv2 REQUIRED)

set(msg_files
	"msg/Point2DWithFloat.msg"
	"msg/ImagePointsWithFloatStamped.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
	${msg_files}
	DEPENDENCIES builtin_interfaces
)

add_library(${PROJECT_NAME}_detector STATIC
	src/detection/fimd/generated_module.cpp
	src/detection/fimd/cpu_detector.cpp
	src/detection/fimd/gpu_detector.cpp
	src/utils/thread_pool.cpp
	src/utils/compute_shader.cpp
	src/app/package_config.cpp
	src/app/detector_node.cpp
)
target_include_directories(${PROJECT_NAME}_detector PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME}_detector PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_compile_definitions(${PROJECT_NAME}_detector PRIVATE
	UVDAR_CORE_SHADER_DIR="${CMAKE_CURRENT_SOURCE_DIR}/shaders"
)
target_link_libraries(${PROJECT_NAME}_detector
	yaml-cpp::yaml-cpp
	dl
	${EGL_LIBRARY}
	${GBM_LIBRARY}
	${GLESV2_LIBRARY}
	${OpenCV_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}_detector
	rclcpp
	sensor_msgs
	cv_bridge
	builtin_interfaces
)

add_executable(detector_node
	src/app/main_detector_node.cpp
)
target_link_libraries(detector_node
	${PROJECT_NAME}_detector
)
ament_target_dependencies(detector_node
	rclcpp
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(${PROJECT_NAME}_detector ${cpp_typesupport_target})

install(TARGETS
	${PROJECT_NAME}_detector
	detector_node
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

ament_export_dependencies(rosidl_default_runtime)
ament_package()