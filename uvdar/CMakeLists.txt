cmake_minimum_required(VERSION 3.5)
project(uvdar)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fno-omit-frame-pointer)
endif()

option(ENABLE_TESTS "Enable building tests" ON)
option(USE_TRACY "Enable Tracy profiling" OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG" CACHE STRING "" FORCE)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(COVERAGE)
  message(WARNING "building with --coverage, the performance might be limited")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
endif()

include(cmake/UpdateSubmodules.cmake)

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
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

find_package(Threads REQUIRED)
find_package(Vulkan)
if(NOT Vulkan_FOUND)
    message(FATAL_ERROR 
        "Vulkan SDK not found! \n"
        "Please install it using: sudo apt install libvulkan-dev vulkan-tools vulkan-headers")
endif()

add_subdirectory(third_party)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${mrs_lib_INCLUDE_DIRS}
)


set(LIBRARIES
  UvdarCore_UvLedDetectorLib
)

## | --------------------- Shaders --------------------- |
find_program(GLSLANG_VALIDATOR glslangValidator)
if(NOT GLSLANG_VALIDATOR)
  message(FATAL_ERROR "glslangValidator not found. Install glslang-tools.")
endif()

set(SHADER_SRC 
  ${CMAKE_CURRENT_SOURCE_DIR}/gpu_shaders/eval_fast_ring.comp
)

set(SHADER_SPV 
  ${CMAKE_CURRENT_BINARY_DIR}/eval_fast_ring.spv
)

add_custom_command(
  OUTPUT ${SHADER_SPV}
  COMMAND ${GLSLANG_VALIDATOR} -V ${SHADER_SRC} -o ${SHADER_SPV}
  DEPENDS ${SHADER_SRC}
  VERBATIM
)
add_custom_target(uvdar_shaders ALL DEPENDS ${SHADER_SPV})
## --------------------------------------------------------


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
  PRIVATE
    kompute
    Vulkan::Vulkan
)

# Disable spdlog macros (if anything uses them)
target_compile_definitions(UvdarCore_UvLedDetectorLib PRIVATE SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_OFF)

target_compile_definitions(UvdarCore_UvLedDetectorLib PRIVATE KOMPUTE_OPT_ACTIVE_LOG_LEVEL=KOMPUTE_LOG_LEVEL_OFF)

add_dependencies(UvdarCore_UvLedDetectorLib uvdar_shaders)


## | -------------------- export interfaces ------------------- |
foreach(library IN LISTS LIBRARIES)
  target_include_directories(${library}
    PUBLIC
      $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )
endforeach()


# Create an umbrella
add_library(uvdar_uvdar INTERFACE) # just a container for usage requirements
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS uvdar_uvdar) # namespaced alias so I can use target_link_libraries(target PRIVATE uvdar::uvdar)

set_target_properties(uvdar_uvdar
  PROPERTIES
    EXPORT_NAME uvdar
    OUTPUT_NAME uvdar
)

target_link_libraries(uvdar_uvdar
  INTERFACE
    ${LIBRARIES}
)

target_include_directories(uvdar_uvdar
  INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

## ---------------------------------------------------------------



## --------------------------------------------------------------
## |                           Testing                          |
## --------------------------------------------------------------

if(ENABLE_TESTS)
  add_subdirectory(test)
endif()

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS UvdarCore_UvLedDetectorLib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(USE_TRACY)
  install(TARGETS tracy_shared
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endif()


install(
  DIRECTORY "include/"
  DESTINATION "include/${PROJECT_NAME}"
)


install(FILES ${SHADER_SPV}
  DESTINATION share/${PROJECT_NAME}/gpu_shaders
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/gpu_shaders/
  DESTINATION share/${PROJECT_NAME}/gpu_shaders
)



  ament_export_libraries(UvdarCore_UvLedDetectorLib)
  ament_export_dependencies(${DEPENDENCIES})

ament_export_include_directories("include/${PROJECT_NAME}")



# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

# install(DIRECTORY config
#   DESTINATION share/${PROJECT_NAME}
# )

# ament_export_targets(
#   export_${PROJECT_NAME} HAS_LIBRARY_TARGET
# )


ament_package()