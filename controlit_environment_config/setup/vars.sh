#!/bin/bash

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:`rospack find controlit_logging`/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:`rospack find controlit_dependency_addons`/lib

export PYTHONPATH=$PYTHONPATH:`rospack find controlit_robot_interface_library`/src
export PYTHONPATH=$PYTHONPATH:`rospack find controlit_core`/src

# Build variables.
export ROS_BUILD_TYPE=RelWithDebInfo
# export ROS_BUILD_TYPE=Release

#export CAPTURE_CODE_COVERAGE=On
# unset CAPTURE_CODE_COVERAGE

export CONTROLIT_COMPILE_FLAGS="-W -Wall -Wno-unused-parameter -Werror -fno-strict-aliasing -pthread -fopenmp -march=native -mtune=native"
# export CONTROLIT_COMPILE_FLAGS="-W -Wall -Wno-unused-parameter -Werror -fno-strict-aliasing -pthread -fopenmp -march=core-avx-i -mtune=native"
# export CONTROLIT_COMPILE_FLAGS="-fno-strict-aliasing -pthread -fopenmp -march=native -mtune=native"