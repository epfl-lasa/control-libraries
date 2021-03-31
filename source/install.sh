#!/bin/bash
SCRIPT=$(readlink -f "${BASH_SOURCE[0]}")
SOURCE_PATH=$(dirname "$SCRIPT")

BUILD_TESTING="OFF"
BUILD_CONTROLLERS="ON"
BUILD_DYNAMICAL_SYSTEMS="ON"
BUILD_ROBOT_MODEL="ON"

FAIL_MESSAGE="The provided input arguments are not valid.
Run the script with the '-h' or '--help' argument."

HELP_MESSAGE="Usage: ./install.sh [OPTIONS]

An install script for the control libraries.

Options:
  -c, --controllers        Build the controllers library ('ON'|'OFF')
  -d, --dynamical_systems  Build the dynamical systems library ('ON'|'OFF')
  -r, --robot_model        Build the robot model library ('ON'|'OFF')
  -t, --testing            Build and run the unit tests ('ON'|'OFF')
  -h, --help               Show this help message"

while [ "$#" -gt 0 ]; do
  if [[ "$1" != "-h" && "$1" != "--help" ]]; then
    if [[ "$1" =~ "=" ]]; then # if '=' in argument
      if [[ "${1#*=}" != "ON" && "${1#*=}" != "OFF" ]]; then
        echo "$FAIL_MESSAGE"; exit 1
      fi
    else
      if [[ "$2" != "ON" ]] && [[ "$2" != "OFF" ]]; then # if no '=' in argument
        echo "$FAIL_MESSAGE"; exit 1
      fi
    fi
  fi
  case "$1" in
    -t) BUILD_TESTING="$2"; shift 2;;
    -c) BUILD_CONTROLLERS="$2"; shift 2;;
    -d) BUILD_DYNAMICAL_SYSTEMS="$2"; shift 2;;
    -r) BUILD_ROBOT_MODEL="$2"; shift 2;;
    -h) echo "$HELP_MESSAGE"; exit 1;;

    --testing=*) BUILD_TESTING="${1#*=}"; shift 1;;
    --controllers=*) BUILD_CONTROLLERS="${1#*=}"; shift 1;;
    --dynamical_systems=*) BUILD_DYNAMICAL_SYSTEMS="${1#*=}"; shift 1;;
    --robot_model=*) BUILD_ROBOT_MODEL="${1#*=}"; shift 1;;
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

  apt-get update && apt-get install -y robotpkg-py38-pinocchio

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
