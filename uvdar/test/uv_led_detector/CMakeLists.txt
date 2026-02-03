get_filename_component(TEST_NAME "${CMAKE_CURRENT_SOURCE_DIR}" NAME)

## --------------------------------------------------------------
## |                   CPU single point                          |
## --------------------------------------------------------------
ament_add_gtest(test_${TEST_NAME}_single_point_cpu
  test_single_point_cpu.cpp
  TIMEOUT 60
)

target_link_libraries(test_${TEST_NAME}_single_point_cpu
  ${PROJECT_NAME}::${PROJECT_NAME}
)

## --------------------------------------------------------------
## |                   GPU single point                          |
## --------------------------------------------------------------
ament_add_gtest(test_${TEST_NAME}_single_point_gpu
  test_single_point_gpu.cpp
  TIMEOUT 60
)

target_link_libraries(test_${TEST_NAME}_single_point_gpu
  ${PROJECT_NAME}::${PROJECT_NAME}
)

## --------------------------------------------------------------
## |                    CPU multi points                        |
## --------------------------------------------------------------
# CPU computational time
ament_add_gtest(test_${TEST_NAME}_multi_points_cpu
  test_multi_points_cpu.cpp
  TIMEOUT 60
)

target_link_libraries(test_${TEST_NAME}_multi_points_cpu
  ${PROJECT_NAME}::${PROJECT_NAME}
)

## --------------------------------------------------------------
## |                   GPU multi points                         |
## --------------------------------------------------------------
# GPU computational time
ament_add_gtest(test_${TEST_NAME}_multi_points_gpu
  test_multi_points_gpu.cpp
  TIMEOUT 60
)

target_link_libraries(test_${TEST_NAME}_multi_points_gpu
  ${PROJECT_NAME}::${PROJECT_NAME}
)


