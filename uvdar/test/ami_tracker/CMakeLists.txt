get_filename_component(TEST_NAME "${CMAKE_CURRENT_SOURCE_DIR}" NAME)

## --------------------------------------------------------------
## |                   Signal Matcher                           |
## --------------------------------------------------------------
ament_add_gtest(test_${TEST_NAME}_signal_matcher
  test_signal_matcher.cpp
  TIMEOUT 60
)

target_link_libraries(test_${TEST_NAME}_signal_matcher
  ${PROJECT_NAME}::${PROJECT_NAME}
)

## --------------------------------------------------------------
## |                   Local Search                             |
## --------------------------------------------------------------
ament_add_gtest(test_${TEST_NAME}_local_search
  test_local_search.cpp
  TIMEOUT 60
)

target_link_libraries(test_${TEST_NAME}_local_search
  ${PROJECT_NAME}::${PROJECT_NAME}
)




