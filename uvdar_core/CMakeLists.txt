cmake_minimum_required(VERSION 3.5)
project(uvdar_core)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fno-omit-frame-pointer)
endif()


option(ENABLE_TESTS "Enable building tests" ON)

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
  ament_index_cpp
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
  kompute
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()



# find_package(kompute CONFIG REQUIRED)


# find_package(OpenCV REQUIRED HINTS /usr/local/lib)

# set(OpenGL_GL_PREFERENCE "LEGACY")
# find_package(OpenGL REQUIRED)
# find_library(LGBM libgbm.so REQUIRED)
# find_library(LEGL libEGL.so REQUIRED)
# find_library(LGL libGL.so REQUIRED)
# include(FindPackageHandleStandardArgs)
# FIND_PACKAGE_HANDLE_STANDARD_ARGS(LIBGBM DEFAULT_MSG LIBGBM_LIBRARIES)
# mark_as_advanced(LIBGBM_INCLUDE_DIRS LIBGBM_LIBRARIES)


include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${mrs_lib_INCLUDE_DIRS}
  # ${OpenGL_INCLUDE_DIRS}
  # ${LIBGBM_INCLUDE_DIRS}
  )

set(LIBRARIES
  UvdarCore_UvLedDetectorLib
)

## | --------------------- UvdarCore_UvLedDetectorLib --------------------- |
add_library(UvdarCore_UvLedDetectorLib SHARED
  src/uv_led_detector/uv_led_detector_fast_gpu.cpp
  src/uv_led_detector/uv_led_detector_fast_cpu.cpp
  src/uv_led_detector/uv_led_detector.cpp
  src/uv_led_detector/gpu_context.cpp
)

ament_target_dependencies(UvdarCore_UvLedDetectorLib
  PUBLIC
    ${DEPENDENCIES}
)

target_link_libraries(UvdarCore_UvLedDetectorLib
  PUBLIC
    kompute
)

## | -------------------- export interfaces ------------------- |
foreach(library IN LISTS LIBRARIES)
  target_include_directories(${library}
    PUBLIC
      $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )
endforeach()


# Create an umbrella
add_library(uvdar_core_uvdar_core INTERFACE)

add_library(uvdar_core::uvdar_core ALIAS uvdar_core_uvdar_core)

set_target_properties(uvdar_core_uvdar_core
  PROPERTIES
    EXPORT_NAME uvdar_core
    OUTPUT_NAME uvdar_core
)

target_link_libraries(uvdar_core_uvdar_core
  INTERFACE
    ${LIBRARIES}
)

target_include_directories(uvdar_core_uvdar_core
  INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)


## --------------------------------------------------------------
## |                           Testing                          |
## --------------------------------------------------------------

if(ENABLE_TESTS)
  add_subdirectory(test)
endif()

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

ament_export_libraries(
  ${LIBRARIES}
)

install(
  TARGETS
    ${LIBRARIES}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY "include/"
  DESTINATION "include/${PROJECT_NAME}"
)

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)

install(DIRECTORY include/uvdar_core/uv_led_detector/gpu_shaders
  DESTINATION share/${PROJECT_NAME}/uv_led_detector/
)

# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

# install(DIRECTORY config
#   DESTINATION share/${PROJECT_NAME}
# )

# ament_export_targets(
#   export_${PROJECT_NAME} HAS_LIBRARY_TARGET
# )

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()