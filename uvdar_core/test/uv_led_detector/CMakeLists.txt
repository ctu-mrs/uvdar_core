get_filename_component(TEST_NAME "${CMAKE_CURRENT_SOURCE_DIR}" NAME)

## --------------------------------------------------------------
## |                   CPU single point                          |
## --------------------------------------------------------------
ament_add_gtest(test_${TEST_NAME}_single_point_cpu
  test_single_point_cpu.cpp
  TIMEOUT 60
)

target_link_libraries(test_${TEST_NAME}_single_point_cpu
  uvdar_core::uvdar_core
)

## --------------------------------------------------------------
## |                   GPU single point                          |
## --------------------------------------------------------------
ament_add_gtest(test_${TEST_NAME}_single_point_gpu
  test_single_point_gpu.cpp
  TIMEOUT 60
)

target_link_libraries(test_${TEST_NAME}_single_point_gpu
  uvdar_core::uvdar_core
)

## --------------------------------------------------------------
## |                    CPU multi points                        |
## --------------------------------------------------------------
# CPU computational time
ament_add_gtest(test_${TEST_NAME}_multi_points_cpu
  test_multi_points_cpu.cpp
  TIMEOUT 60
)

find_package(Threads REQUIRED)
find_package(Vulkan REQUIRED)


target_link_libraries(test_${TEST_NAME}_multi_points_cpu
  uvdar_core::uvdar_core
  Threads::Threads
  Vulkan::Vulkan
)

if(USE_TRACY)
  target_compile_definitions(test_${TEST_NAME}_multi_points_cpu 
    PUBLIC TRACY_ENABLE
    Threads::Threads
  )
endif()

## --------------------------------------------------------------
## |                   GPU multi points                         |
## --------------------------------------------------------------
# GPU computational time
ament_add_gtest(test_${TEST_NAME}_multi_points_gpu
  test_multi_points_gpu.cpp
  TIMEOUT 60
)

find_package(Threads REQUIRED)
find_package(Vulkan REQUIRED)


target_link_libraries(test_${TEST_NAME}_multi_points_gpu
  uvdar_core::uvdar_core
  Threads::Threads
  Vulkan::Vulkan
)

if(USE_TRACY)
  target_compile_definitions(test_${TEST_NAME}_multi_points_gpu 
    PUBLIC TRACY_ENABLE
    Threads::Threads
  )
endif()

