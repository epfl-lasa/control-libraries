#!/bin/bash
SCRIPT=$(readlink -f "${BASH_SOURCE[0]}")
SOURCE_PATH=$(dirname "$SCRIPT")

BUILD_CONTROLLERS="ON"
BUILD_DYNAMICAL_SYSTEMS="ON"
BUILD_ROBOT_MODEL="ON"
BUILD_TESTING="OFF"
INSTALL_DESTINATION="/usr/local"

FAIL_MESSAGE="The provided input arguments are not valid.
Run the script with the '--help' argument."

HELP_MESSAGE="Usage: [sudo] ./install.sh [OPTIONS]

An install script for the control libraries.

Options:
  --dir [path]             Configure the installation directory
                           (default: /usr/local)
  --no-controllers         Exclude the controllers library
  --no-dynamical-systems   Exclude the dynamical systems library
  --no-robot-model         Exclude the robot model library
  --build-tests            Build the unittest targets
  --help                   Show this help message
  --clean                  Delete any previously installed
                           header files from /usr/local/include and any
                           shared library files from /usr/local/lib
  --cleandir [path]        Delete any previously installed
                           header and library files from the specified path"

function uninstall {
  function delete_components {
    rm -r "${INSTALL_DESTINATION}"/include/controllers
    rm -r "${INSTALL_DESTINATION}"/include/dynamical_systems
    rm -r "${INSTALL_DESTINATION}"/include/robot_model
    rm -r "${INSTALL_DESTINATION}"/include/state_representation
    rm -r "${INSTALL_DESTINATION}"/lib/libcontrollers*.so
    rm -r "${INSTALL_DESTINATION}"/lib/libdynamical_systems*.so
    rm -r "${INSTALL_DESTINATION}"/lib/librobot_model*.so
    rm -r "${INSTALL_DESTINATION}"/lib/libstate_representation*.so
  }

  delete_components >/dev/null 2>&1

  echo "Deleted any control library artefacts from ${INSTALL_DESTINATION}."
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --dir) INSTALL_DESTINATION=$2; shift 2;;
    --clean) uninstall; exit 1;;
    --cleandir) INSTALL_DESTINATION=$2; uninstall; exit 1;;
    --no-controllers) BUILD_CONTROLLERS="OFF"; shift 1;;
    --no-dynamical-systems) BUILD_DYNAMICAL_SYSTEMS="OFF"; shift 1;;
    --no-robot-model) BUILD_ROBOT_MODEL="OFF"; shift 1;;
    --build-tests) BUILD_TESTING="ON"; shift 1;;
    --help) echo "$HELP_MESSAGE"; exit 1;;

    -*) echo "Unknown option: $1" >&2; echo "$FAIL_MESSAGE"; exit 1;;
  esac
done

# install base dependencies
apt-get update && apt-get install libeigen3-dev

# install module-specific dependencies
if [ "${BUILD_ROBOT_MODEL}" == "ON" ]; then
  apt-get install lsb-release gnupg2 curl

  # install pinocchio
  echo "deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
    | tee /etc/apt/sources.list.d/robotpkg.list
  curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | apt-key add -

  apt-get update && apt-get install robotpkg-pinocchio

  export PATH=/opt/openrobots/bin:$PATH
  export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
  export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
  export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH

  # install osqp
  mkdir -p "${SOURCE_PATH}"/tmp/lib
  cd "${SOURCE_PATH}"/tmp/lib || exit 1
  git clone --recursive https://github.com/oxfordcontrol/osqp
  cd "${SOURCE_PATH}"/tmp/lib/osqp/ && mkdir -p build && cd build || exit 1
  cmake -G "Unix Makefiles" .. && cmake --build . --target install
  # install osqp eigen wrapper
  cd "${SOURCE_PATH}"/tmp/lib || exit 1
  git clone https://github.com/robotology/osqp-eigen.git
  cd "${SOURCE_PATH}"/tmp/lib/osqp-eigen && mkdir -p build && cd build || exit 1
  cmake .. && make -j && make install
fi

# install testing dependencies
if [ "${BUILD_TESTING}" == "ON" ]; then
  apt-get update && apt-get install libgtest-dev

  mkdir -p "${SOURCE_PATH}"/tmp/lib && cd "${SOURCE_PATH}"/tmp/lib || exit 1
  cmake /usr/src/gtest && make
  cp lib/* /usr/local/lib || cp ./*.a /usr/local/lib
fi

# build and install the specified modules
cd "${SOURCE_PATH}" && mkdir -p build && cd build || exit 1

cmake -DCMAKE_INSTALL_PREFIX="${INSTALL_DESTINATION}" \
  -DBUILD_CONTROLLERS="${BUILD_CONTROLLERS}" \
  -DBUILD_DYNAMICAL_SYSTEMS="${BUILD_DYNAMICAL_SYSTEMS}" \
  -DBUILD_ROBOT_MODEL="${BUILD_ROBOT_MODEL}" \
  -DBUILD_TESTING="${BUILD_TESTING}" ..

make -j && make install

# cleanup any temporary folders
rm -rf "${SOURCE_PATH}"/tmp

# reset location
cd "${SOURCE_PATH}" || return
