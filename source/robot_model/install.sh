#!/bin/sh
SCRIPT=$(readlink -f "$BASH_SOURCE")
SOURCE_PATH=$(dirname "$SCRIPT")

apt-get update && apt-get install -y \
    libeigen3-dev \
    lsb-release \
    gnupg2 \
    curl

# install googletest
mkdir ${SOURCE_PATH}/lib
cd ${SOURCE_PATH}/lib && git clone https://github.com/google/googletest.git

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
cd ${SOURCE_PATH}/lib && git clone --recursive https://github.com/oxfordcontrol/osqp
cd ${SOURCE_PATH}/lib/osqp/ && mkdir build && cd build && cmake -G "Unix Makefiles" .. && cmake --build . --target install
# install osqp eigen wrapper
cd ${SOURCE_PATH}/lib && git clone https://github.com/robotology/osqp-eigen.git
cd ${SOURCE_PATH}/lib/osqp-eigen && mkdir build && cd build && cmake .. && make -j && make install

# install robot_model
cd ${SOURCE_PATH} && mkdir -p build && cd build && cmake -Druntests="OFF" .. && make -j && make install

# reset location
cd ${SOURCE_PATH}