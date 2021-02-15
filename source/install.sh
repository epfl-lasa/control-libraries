#!/bin/bash
SCRIPT=$(readlink -f "${BASH_SOURCE[0]}")
SOURCE_PATH=$(dirname "$SCRIPT")

# options
# TODO: parse arguments and provide --help to set these options
BUILD_TESTS="ON"
BUILD_DYNAMICAL_SYSTEMS="ON"
BUILD_ROBOT_MODEL="ON"

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
if [ "${BUILD_TESTS}" == "ON" ]; then
  mkdir "${SOURCE_PATH}"/lib
  cd "${SOURCE_PATH}"/lib && git clone --depth 1 --branch v1.10.x https://github.com/google/googletest.git
fi

# build and install the specified modules
cd "${SOURCE_PATH}" && mkdir -p build && cd build \
  && cmake -DBUILD_DYNAMICAL_SYSTEMS="${BUILD_DYNAMICAL_SYSTEMS}" \
           -DBUILD_ROBOT_MODEL="${BUILD_ROBOT_MODEL}" \
           -Druntests="${BUILD_TESTS}" .. \
  && make -j && make install

# reset location
cd "${SOURCE_PATH}" || return
