#!/bin/sh
SOURCE_PATH=${PWD}

apt-get update && apt-get install -y \
    libeigen3-dev

# install robot_model
cd ${SOURCE_PATH} && mkdir -p build && cd build && cmake -Druntests="OFF" .. && make -j && make install