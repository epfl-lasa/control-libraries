#!/bin/sh
SOURCE_PATH=${PWD}

apt-get update && apt-get install -y \
    libeigen3-dev

# install googletest
mkdir ${SOURCE_PATH}/lib
cd ${SOURCE_PATH}/lib && git clone https://github.com/google/googletest.git

# install state_representation
cd ${SOURCE_PATH} && mkdir -p build && cd build && cmake -Druntests="OFF" .. && make -j && make install