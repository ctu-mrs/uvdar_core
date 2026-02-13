find_package(ament_cmake_gtest REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

add_subdirectory(./uv_led_detector)
add_subdirectory(./blink_processor)
