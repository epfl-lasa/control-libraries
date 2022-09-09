#!/bin/bash
SCRIPT=$(readlink -f "${BASH_SOURCE[0]}")
SOURCE_PATH=$(dirname "$SCRIPT")

BUILD_CONTROLLERS="ON"
BUILD_DYNAMICAL_SYSTEMS="ON"
BUILD_ROBOT_MODEL="ON"
BUILD_TESTING="OFF"
INSTALL_DESTINATION="/usr/local"
AUTO_INSTALL=""

EIGEN_VERSION=3.4.0
OSQP_TAG=0.6.2
OSQP_EIGEN_TAG=0.6.4
PINOCCHIO_TAG=2.6.9

FAIL_MESSAGE="The provided input arguments are not valid.
Run the script with the '--help' argument."

HELP_MESSAGE="Usage: [sudo] ./install.sh [OPTIONS]

An install script for the control libraries.

Options:
  -y, --auto               Suppress any input prompts and
                           automatically approve install steps.
  -d, --dir [path]         Configure the installation directory
                           (default: /usr/local).

  --no-controllers         Exclude the controllers library.
  --no-dynamical-systems   Exclude the dynamical systems library.
  --no-robot-model         Exclude the robot model library.
  --build-tests            Build the unittest targets.

  --clean                  Delete any previously installed header
                           files from /usr/local/include and any
                           shared library files from /usr/local/lib.
  --cleandir [path]        Delete any previously installed header
                           and library files from the specified path.

  -h, --help               Show this help message."

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
    -y|--auto) AUTO_INSTALL="-y"; shift 1;;
    --build-tests) BUILD_TESTING="ON"; shift 1;;
    --clean) uninstall; exit 0;;
    --cleandir) INSTALL_DESTINATION=$2; uninstall; exit 0;;
    -d|--dir) INSTALL_DESTINATION=$2; shift 2;;
    -h|--help) echo "$HELP_MESSAGE"; exit 0;;
    --no-controllers) BUILD_CONTROLLERS="OFF"; shift 1;;
    --no-dynamical-systems) BUILD_DYNAMICAL_SYSTEMS="OFF"; shift 1;;
    --no-robot-model) BUILD_ROBOT_MODEL="OFF"; shift 1;;

    -*) echo "Unknown option: $1" >&2; echo "$FAIL_MESSAGE"; exit 1;;
  esac
done

if [ "${BUILD_CONTROLLERS}" == "ON" ] && [ "${BUILD_ROBOT_MODEL}" == "OFF" ]; then
  echo "The robot model library is required to build the controllers library!"
  echo "Either disable controller installation with '--no-controllers' or enable"
  echo "the robot model installation by removing the '--no-robot-model' flag."
  exit 1
fi

# cleanup any previous build folders
rm -rf "${SOURCE_PATH}"/tmp

# install base dependencies
echo ">>> INSTALLING BASE DEPENDENCIES"
INSTALLED_EIGEN=$(pkg-config --modversion eigen3)
if [ "${INSTALLED_EIGEN::4}" != "${EIGEN_VERSION::4}" ]; then
  echo ">>> INSTALLING EIGEN"
  mkdir -p "${SOURCE_PATH}"/tmp/lib && cd "${SOURCE_PATH}"/tmp/lib || exit 1
  wget -c "https://gitlab.com/libeigen/eigen/-/archive/${EIGEN_VERSION}/eigen-${EIGEN_VERSION}.tar.gz" -O - | tar -xz || exit 1
  cd "eigen-${EIGEN_VERSION}" && mkdir -p build && cd build && cmake .. && make install || exit 1
fi
EIGEN_PATH=$(cmake --find-package -DNAME=Eigen3 -DCOMPILER_ID=GNU -DLANGUAGE=C -DMODE=COMPILE)
if [ "${EIGEN_PATH::14}" != "-I/usr/include" ]; then
  rm -rf /usr/include/eigen3 && ln -s ${EIGEN_PATH:2} /usr/include/eigen3 || exit 1
fi

# install module-specific dependencies
if [ "${BUILD_ROBOT_MODEL}" == "ON" ]; then
  echo ">>> INSTALLING ROBOT MODEL DEPENDENCIES"
  apt-get update && apt-get install "${AUTO_INSTALL}" libboost-all-dev liburdfdom-dev || exit 1

  INSTALLED_PINOCCHIO=$(pkg-config --modversion pinocchio)
  if [ "${INSTALLED_PINOCCHIO}" != "${PINOCCHIO_TAG}" ]; then
    mkdir -p "${SOURCE_PATH}"/tmp/lib && cd "${SOURCE_PATH}"/tmp/lib || exit 1

    echo ">>> INSTALLING OSQP [1/3]"
    git clone --depth 1 -b v${OSQP_TAG} --recursive https://github.com/oxfordcontrol/osqp \
        && cd osqp && mkdir build && cd build && cmake -G "Unix Makefiles" .. && cmake --build . --target install \
        && cd ../.. && rm -r osqp || exit 1

    echo ">>> INSTALLING OSQP_EIGEN [2/3]"
    git clone --depth 1 -b v${OSQP_EIGEN_TAG} https://github.com/robotology/osqp-eigen.git \
        && cd osqp-eigen && mkdir build && cd build && cmake .. && make -j && make install \
        && cd ../.. && rm -r osqp-eigen || exit 1

    echo ">>> INSTALLING PINOCCHIO [3/3]"
    git clone --depth 1 -b v${PINOCCHIO_TAG} --recursive https://github.com/stack-of-tasks/pinocchio \
        && cd pinocchio && mkdir build && cd build \
        && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_PYTHON_INTERFACE=OFF \
        && make -j1 && make install && cd ../.. && rm -r pinocchio || exit 1
  fi
  ldconfig
fi

# install testing dependencies
if [ "${BUILD_TESTING}" == "ON" ]; then
  echo ">>> INSTALLING TEST DEPENDENCIES"
  apt-get update && apt-get install "${AUTO_INSTALL}" libgtest-dev || exit 1

  mkdir -p "${SOURCE_PATH}"/tmp/lib/gtest && cd "${SOURCE_PATH}"/tmp/lib/gtest || exit 1
  cmake /usr/src/gtest && make || exit 1
  cp lib/* /usr/local/lib || cp ./*.a /usr/local/lib
fi

# build and install the specified modules
echo ">>> BUILDING CONTROL LIBRARIES"
cd "${SOURCE_PATH}" && mkdir -p build && cd build || exit 1

cmake -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_DESTINATION}" \
  -DBUILD_CONTROLLERS="${BUILD_CONTROLLERS}" \
  -DBUILD_DYNAMICAL_SYSTEMS="${BUILD_DYNAMICAL_SYSTEMS}" \
  -DBUILD_ROBOT_MODEL="${BUILD_ROBOT_MODEL}" \
  -DBUILD_TESTING="${BUILD_TESTING}" .. || exit 1

make -j && make install || exit 1

ldconfig

# cleanup any temporary folders
rm -rf "${SOURCE_PATH}"/tmp

# reset location
cd "${SOURCE_PATH}" || return
