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

target_compile_options(test_${TEST_NAME}_signal_matcher PRIVATE
  -Wno-unused-parameter
  -Wno-missing-field-initializers
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

target_compile_options(test_${TEST_NAME}_local_search PRIVATE
  -Wno-unused-parameter
  -Wno-missing-field-initializers
)

## --------------------------------------------------------------
## |                   Poly Regression Predictor                |
## --------------------------------------------------------------
ament_add_gtest(test_${TEST_NAME}_poly_regression_predictor
  test_poly_regression_predictor.cpp
  TIMEOUT 60
)

target_link_libraries(test_${TEST_NAME}_poly_regression_predictor
  ${PROJECT_NAME}::${PROJECT_NAME}
)

target_compile_options(test_${TEST_NAME}_poly_regression_predictor PRIVATE
  -Wno-unused-parameter
  -Wno-missing-field-initializers
)

## --------------------------------------------------------------
## |                   Extended Search                          |
## --------------------------------------------------------------
ament_add_gtest(test_${TEST_NAME}_extended_search
  test_extended_search.cpp
  TIMEOUT 60
)

target_link_libraries(test_${TEST_NAME}_extended_search
  ${PROJECT_NAME}::${PROJECT_NAME}
)


target_compile_options(test_${TEST_NAME}_extended_search PRIVATE
  -Wno-unused-parameter
  -Wno-missing-field-initializers
)

## --------------------------------------------------------------
## |                   Blink processor                          |
## --------------------------------------------------------------
ament_add_gtest(test_${TEST_NAME}_blink_processor
  test_blink_processor.cpp
  TIMEOUT 60
)

target_link_libraries(test_${TEST_NAME}_blink_processor
  ${PROJECT_NAME}::${PROJECT_NAME}
)

target_compile_options(test_${TEST_NAME}_blink_processor PRIVATE
  -Wno-unused-parameter
  -Wno-missing-field-initializers
)

## --------------------------------------------------------------
## |                   Camera Calibration                       |
## --------------------------------------------------------------
ament_add_gtest(test_${TEST_NAME}_camera_calibration
  test_camera_calibration.cpp
  TIMEOUT 60
)

target_link_libraries(test_${TEST_NAME}_camera_calibration
  ${PROJECT_NAME}::${PROJECT_NAME}
)

target_compile_options(test_${TEST_NAME}_camera_calibration PRIVATE
  -Wno-unused-parameter
  -Wno-missing-field-initializers
)

target_compile_definitions(test_${TEST_NAME}_camera_calibration PRIVATE
  TEST_ASSETS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/assets"
)
