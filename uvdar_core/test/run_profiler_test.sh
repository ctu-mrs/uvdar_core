#!/bin/bash

while [ ! -e "build/COLCON_IGNORE" ]; do
  cd ..
  if [[ `pwd` == "/" ]]; then
    # we reached the root and didn't find the build/COLCON_IGNORE file - that's a fail!
    echo "Cannot compile, probably not in a workspace (if you want to create a new workspace, call \"colcon init\" in its root first)".
    exit 1
  fi
done


colcon test-result --delete-yes

colcon test --packages-select uvdar_core \
  --ctest-args -R "test_uv_led_detector_multi_points_gpu" \
  --event-handlers console_direct+

colcon test-result --all --verbose