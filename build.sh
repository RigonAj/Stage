#!/bin/bash

alias build="colcon build \
  --symlink-install \
  --packages-select ball_tracking_cpp \
  --cmake-args \
  -DCMAKE_C_COMPILER=gcc-13 \
  -DCMAKE_CXX_COMPILER=g++-13 \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
alias run="source install/setup.bash && ros2 run ball_tracking_cpp talker"
alias clear="rm -rf build install log && ccache -c"

