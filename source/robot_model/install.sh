#!/bin/sh
SOURCE_PATH=${PWD}

apt-get update && apt-get install -y \
    libeigen3-dev \
    zlib1g-dev \
    libssl-dev \
    libboost-all-dev \
    libboost-dev \
    libboost-filesystem-dev \
    libboost-system-dev \
    libboost-chrono-dev \
    libboost-date-time-dev \
    libboost-thread-dev \
    libboost-test-dev \
    libbz2-dev \
    libncurses-dev \
    pax \
    tar \
    liburdfdom-dev \
    libassimp-dev \
    assimp-utils \
    doxygen \
    texlive-latex-extra

# install googletest
mkdir ${SOURCE_PATH}/lib
cd ${SOURCE_PATH}/lib && git clone https://github.com/google/googletest.git

# install pinocchio
cd ${SOURCE_PATH}/lib && git clone git://git.openrobots.org/robots/robotpkg
cd ${SOURCE_PATH}/lib/robotpkg/bootstrap && ./bootstrap --prefix=${SOURCE_PATH}/lib/openrobots
export ROBOTPKG_BASE="${SOURCE_PATH}/lib/openrobots"
cd ${SOURCE_PATH}/lib/robotpkg/math/pinocchio && make update

export PATH="${SOURCE_PATH}/lib/openrobots/bin:${PATH}"
export PKG_CONFIG_PATH="${SOURCE_PATH}/lib/openrobots/lib/pkgconfig:${PKG_CONFIG_PATH}"
export LD_LIBRARY_PATH="${SOURCE_PATH}/lib/openrobots/lib:${LD_LIBRARY_PATH}"
export PYTHONPATH="${SOURCE_PATH}/lib/openrobots/lib/python3.6/site-packages:${PYTHONPATH}"

# install osqp
cd ${SOURCE_PATH}/lib && git clone --recursive https://github.com/oxfordcontrol/osqp
cd ${SOURCE_PATH}/lib/osqp/ && mkdir build && cd build && cmake -G "Unix Makefiles" .. && cmake --build . --target install
# install osqp eigen wrapper
cd ${SOURCE_PATH}/lib && git clone https://github.com/robotology/osqp-eigen.git
cd ${SOURCE_PATH}/lib/osqp-eigen && mkdir build && cd build && cmake .. && make -j && make install

# install robot_model
cd ${SOURCE_PATH} && mkdir -p build && cd build && cmake -Druntests="OFF" .. && make -j && make install