get_filename_component(TEST_NAME "${CMAKE_CURRENT_SOURCE_DIR}" NAME)

# CPU
ament_add_gtest(test_${TEST_NAME}_single_point_cpu
  test_single_point_cpu.cpp
  TIMEOUT 60
)

target_link_libraries(test_${TEST_NAME}_single_point_cpu
  uvdar_core::uvdar_core
)

# GPU
ament_add_gtest(test_${TEST_NAME}_single_point_gpu
  test_single_point_gpu.cpp
  TIMEOUT 60
)

target_link_libraries(test_${TEST_NAME}_single_point_gpu
  uvdar_core::uvdar_core
)

# GPU computational time
ament_add_gtest(test_${TEST_NAME}_comp_time
  test_comp_time_gpu.cpp
  TIMEOUT 60
)

target_link_libraries(test_${TEST_NAME}_comp_time
  uvdar_core::uvdar_core
)