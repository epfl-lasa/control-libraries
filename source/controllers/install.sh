#!/bin/sh
SCRIPT=$(readlink -f "$BASH_SOURCE")
SOURCE_PATH=$(dirname "$SCRIPT")

apt-get update && apt-get install -y \
    libeigen3-dev

# install googletest
mkdir ${SOURCE_PATH}/lib
cd ${SOURCE_PATH}/lib && git clone https://github.com/google/googletest.git

# install dynamical_systems
cd ${SOURCE_PATH} && mkdir -p build && cd build && cmake -Druntests="OFF" .. && make -j && make install

# reset location
cd ${SOURCE_PATH}