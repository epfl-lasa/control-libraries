#!/bin/bash
SCRIPT=$(readlink -f "${BASH_SOURCE[0]}")
PROTOBUF_DIR=$(dirname "${SCRIPT}")/protobuf
CLPROTO_DIR=$(dirname "${SCRIPT}")/clproto_cpp

INSTALL_DESTINATION="/usr/local"

echo ">>> GENERATING PROTOBUF BINDINGS"
cd ${PROTOBUF_DIR} && make -j all

echo ">>> INSTALLING CLPROTO"
cd ${CLPROTO_DIR} && mkdir -p build && cd build || exit 1

cmake -DCMAKE_BUILD_TYPE=Release \
	-DCMAKE_INSTALL_PREFIX="${INSTALL_DESTINATION}" .. 

make -j && sudo make install