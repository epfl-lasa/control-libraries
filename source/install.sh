#!/bin/bash
SCRIPT=$(readlink -f "${BASH_SOURCE[0]}")
SOURCE_PATH=$(dirname "$SCRIPT")

# options
# TODO: parse arguments and provide --help to set these options
BUILD_TESTING="ON"
BUILD_CONTROLLERS="ON"
BUILD_DYNAMICAL_SYSTEMS="ON"
BUILD_ROBOT_MODEL="ON"
BUILD_TESTING="OFF"

FAIL_MESSAGE="The provided input arguments are not valid.
Run the script with the '--help' argument."

HELP_MESSAGE="Usage: ./install.sh [OPTIONS]

An install script for the control libraries.

Options:
  --no-controllers         Exclude the controllers library
  --no-dynamical-systems   Exclude the dynamical systems library
  --no-robot-model         Exclude the robot model library
  --build-tests            Build the unittest targets
  --help                   Show this help message"

while [ "$#" -gt 0 ]; do
  case "$1" in
    --no-controllers) BUILD_CONTROLLERS="OFF"; shift 1;;
    --no-dynamical-systems) BUILD_DYNAMICAL_SYSTEMS="OFF"; shift 1;;
    --no-robot-model) BUILD_ROBOT_MODEL="OFF"; shift 1;;
    --build-tests) BUILD_TESTING="ON"; shift 1;;
    --help) echo "$HELP_MESSAGE"; exit 1;;

    -*) echo "Unknown option: $1" >&2; echo "$FAIL_MESSAGE"; exit 1;;
  esac
done

# install base dependencies
apt-get update && apt-get install -y libeigen3-dev

# install module-specific dependencies
if [ "${BUILD_ROBOT_MODEL}" == "ON" ]; then
  apt-get install -y lsb-release gnupg2 curl

  # install pinocchio
  echo "deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | tee /etc/apt/sources.list.d/robotpkg.list
  curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | apt-key add -

  apt-get update && apt-get install -y robotpkg-pinocchio

  export PATH=/opt/openrobots/bin:$PATH
  export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
  export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
  export PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:$PYTHONPATH # Adapt your desired python version here
  export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH

  # install osqp
  mkdir -p "${SOURCE_PATH}"/robot_model/lib
  cd "${SOURCE_PATH}"/robot_model/lib && git clone --recursive https://github.com/oxfordcontrol/osqp
  cd "${SOURCE_PATH}"/robot_model/lib/osqp/ && mkdir build && cd build && cmake -G "Unix Makefiles" .. && cmake --build . --target install
  # install osqp eigen wrapper
  cd "${SOURCE_PATH}"/robot_model/lib && git clone https://github.com/robotology/osqp-eigen.git
  cd "${SOURCE_PATH}"/robot_model/lib/osqp-eigen && mkdir build && cd build && cmake .. && make -j && make install
fi

# install testing dependencies
if [ "${BUILD_TESTING}" == "ON" ]; then
  mkdir "${SOURCE_PATH}"/lib
  cd "${SOURCE_PATH}"/lib && git clone --depth 1 --branch v1.10.x https://github.com/google/googletest.git
fi

# build and install the specified modules
cd "${SOURCE_PATH}" && mkdir -p build && cd build \
  && cmake -DBUILD_CONTROLLERS="${BUILD_CONTROLLERS}" \
           -DBUILD_DYNAMICAL_SYSTEMS="${BUILD_DYNAMICAL_SYSTEMS}" \
           -DBUILD_ROBOT_MODEL="${BUILD_ROBOT_MODEL}" \
           -DBUILD_TESTING="${BUILD_TESTING}" .. \
  && make -j && make install

# reset location
cd "${SOURCE_PATH}" || return
