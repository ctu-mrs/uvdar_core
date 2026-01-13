find_package(ament_cmake_gtest REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

link_libraries(uvdar_core::uvdar_core)

add_subdirectory(./uv_led_detector)

