get_filename_component(TEST_NAME "${CMAKE_CURRENT_SOURCE_DIR}" NAME)

# CPU
ament_add_gtest(test_${TEST_NAME}_single_point
  test_single_point.cpp
  TIMEOUT 60
)

target_link_libraries(test_${TEST_NAME}_single_point
  uvdar_core::uvdar_core
)