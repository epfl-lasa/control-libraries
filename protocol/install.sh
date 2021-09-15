#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
PROTOBUF_DIR="${SCRIPT_DIR}"/protobuf
CLPROTO_DIR="${SCRIPT_DIR}"/clproto_cpp

INSTALL_DESTINATION="/usr/local"
AUTO_INSTALL=""
BINDINGS_ONLY=false
PROTOBUF_VERSION="3.17.0"

HELP_MESSAGE="Usage: [sudo] ./install.sh [OPTIONS]

An install script for the clproto library.

Options:
  -y, --auto               Suppress any input prompts and
                           automatically approve install steps.

  --bindings-only          Only generate the protobuf bindings
                           without building or installing clproto.

  -d, --dir [path]         Configure the installation directory
                           (default: ${INSTALL_DESTINATION}).

  --clean-bindings         Clean any previously generated protobuf
                           bindings.

  --clean                  Delete any previously installed header
                           files from ${INSTALL_DESTINATION}/include and any
                           shared library files from ${INSTALL_DESTINATION}/lib.

  --cleandir [path]        Delete any previously installed header
                           and library files from the specified path.

  -h, --help               Show this help message."

function make_bindings() {
  cd "${PROTOBUF_DIR}" && make all || exit 1
}

function clean_bindings() {
  cd "${PROTOBUF_DIR}" && make clean
}

function install_protobuf() {
  echo ">>> INSTALLING PROTOBUF DEPENDENCIES"
  apt-get update && apt-get install "${AUTO_INSTALL}" autoconf automake libtool curl make g++ unzip || exit 1

  mkdir -p "${SCRIPT_DIR}"/install
  cd "${SCRIPT_DIR}"/install || exit 1
  wget -O protobuf-cpp-"${PROTOBUF_VERSION}".tar.gz \
    https://github.com/protocolbuffers/protobuf/releases/download/v"${PROTOBUF_VERSION}"/protobuf-cpp-"${PROTOBUF_VERSION}".tar.gz &&
    tar -xzf protobuf-cpp-"${PROTOBUF_VERSION}".tar.gz &&
    rm protobuf-cpp-"${PROTOBUF_VERSION}".tar.gz

  cd "${SCRIPT_DIR}"/install/protobuf-"${PROTOBUF_VERSION}" || exit 1
  ./autogen.sh && ./configure && make && make install || exit 1
  ldconfig
}

function install_state_representation() {
  echo ">> INSTALLING CONTROL LIBRARY DEPENDENCIES"
  mkdir -p "${SCRIPT_DIR}"/install
  cd "${SCRIPT_DIR}"/install || exit 1
  CL_INSTALL_SCRIPT="$(dirname "${SCRIPT_DIR}")"/source/install.sh
  if [ -f "$CL_INSTALL_SCRIPT" ]; then
    bash "${CL_INSTALL_SCRIPT}" --no-controllers --no-dynamical-systems --no-robot-model "${AUTO_INSTALL}"
  else
    echo ">>> INSTALL SCRIPT NOT FOUND: ${CL_INSTALL_SCRIPT}!"
    exit 1
  fi
  ldconfig
}

function install_clproto() {
  cd "${CLPROTO_DIR}" && mkdir -p build && cd build || exit 1

  cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=On \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_DESTINATION}" .. || exit 1

  make -j && make install || exit 1
  ldconfig
}

function uninstall() {
  function delete_components() {
    rm -r "${INSTALL_DESTINATION}"/include/clproto
    rm -r "${INSTALL_DESTINATION}"/lib/libclproto*.so
    clean_bindings
  }

  delete_components >/dev/null 2>&1

  echo "Deleted any clproto artefacts from ${INSTALL_DESTINATION} and ${PROTOBUF_DIR}."
}

while [ "$#" -gt 0 ]; do
  case "$1" in
  -y | --auto)
    AUTO_INSTALL="-y"
    shift 1
    ;;
  -d | --dir)
    INSTALL_DESTINATION=$2
    shift 2
    ;;
  --bindings-only)
    BINDINGS_ONLY=true
    shift 1
    ;;
  --clean-bindings)
    clean_bindings
    exit 0
    ;;
  --clean)
    uninstall
    exit 0
    ;;
  --cleandir)
    INSTALL_DESTINATION=$2
    uninstall
    exit 0
    ;;
  -h | --help)
    echo "$HELP_MESSAGE"
    exit 0
    ;;

  -*)
    echo "Unknown option: $1" >&2
    echo "$FAIL_MESSAGE"
    exit 1
    ;;
  esac
done

if ! [ -x "$(command -v protoc)" ]; then
  echo ">>> PROTOC NOT FOUND"
  install_protobuf || exit 1
fi

echo ">>> GENERATING PROTOBUF BINDINGS"
make_bindings || exit 1

if [ $BINDINGS_ONLY == true ]; then
  echo ">>> DONE!"
  exit 0
fi

PROTOBUF_INSTALL=$(ldconfig -p | grep libprotobuf)
if [ -z "$PROTOBUF_INSTALL" ] ]; then
  echo ">>> LIBPROTOBUF NOT FOUND"
  install_protobuf || exit 1
fi

STATE_REPRESENTATION_INSTALL=$(ldconfig -p | grep libstate_representation)
if [ -z "$STATE_REPRESENTATION_INSTALL" ]; then
  echo ">>> STATE REPRESENTATION LIBRARY NOT FOUND!"
  install_state_representation
fi

echo ">>> INSTALLING CLPROTO"
install_clproto || exit 1

echo ">>> DONE"
