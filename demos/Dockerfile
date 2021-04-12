FROM ubuntu:20.04
ENV DEBIAN_FRONTEND=noninteractive
ENV QT_X11_NO_MITSHM 1

# install core compilation and access dependencies for building the libraries
RUN apt-get update && apt-get install -y \
    gcc g++ make cmake \
    python python3-pip \
    libgl1-mesa-glx \
    sudo git curl wget lsb-release gnupg2 \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /tmp
RUN git clone -b v1.0.0 --depth 1 https://github.com/epfl-lasa/control_libraries.git
WORKDIR /tmp/control_libraries/source
RUN /bin/bash -c "source install.sh"

# get cppzmq bindings
RUN apt-get update && apt-get install -y \
    libsodium-dev \
    libzmq3-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /tmp
RUN wget https://github.com/zeromq/cppzmq/archive/v4.7.1.tar.gz -O cppzmq-4.7.1.tar.gz \
    && tar -xzf cppzmq-4.7.1.tar.gz
WORKDIR cppzmq-4.7.1/build
RUN cmake .. -DCPPZMQ_BUILD_TESTS=OFF && make -j4 install

# # install pybullet
# WORKDIR /tmp
# RUN git clone https://github.com/bulletphysics/bullet3
# WORKDIR /tmp/bullet3
# RUN ./build_cmake_pybullet_double.sh
# ENV PYTHONPATH="/bullet3/build_cmake/examples/pybullet":"${PYTHONPATH}"
#
# WORKDIR /tmp
# RUN git clone -b development --depth 1 https://github.com/epfl-lasa/pybullet_robot.git
# WORKDIR /tmp/pybullet_robot
# RUN ./install.sh

RUN rm -rf /tmp/*

COPY ./ /tmp/demos
WORKDIR /tmp/demos/build
RUN cmake .. && make -j all && make install
ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib

WORKDIR /usr/local/bin
RUN rm -rf /tmp/demos/build
