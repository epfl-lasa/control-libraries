FROM ubuntu:20.04 AS core-build-dependencies
ENV DEBIAN_FRONTEND=noninteractive

# install core compilation and access dependencies for building the libraries
RUN apt-get update && apt-get install -y \
    gcc \
    g++ \
    make \
    cmake \
    git \
    curl \
    wget \
    lsb-release \
    gnupg2 \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*


FROM core-build-dependencies as project-dependencies

# add pinocchio to package list
RUN echo "deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
    | tee /etc/apt/sources.list.d/robotpkg.list \
    && curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key \
    | apt-key add -

# install dependencies for building the libraries
RUN apt-get update && apt-get install -y \
    libeigen3-dev \
    robotpkg-pinocchio \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# install osqp and eigen wrapper
WORKDIR /tmp/osqp_build
RUN git clone --recursive https://github.com/oxfordcontrol/osqp \
    && cd osqp && mkdir build && cd build && cmake -G "Unix Makefiles" .. && cmake --build . --target install

RUN git clone https://github.com/robotology/osqp-eigen.git \
    && cd osqp-eigen && mkdir build && cd build && cmake .. && make -j && make install

RUN rm -rf /tmp/*


FROM project-dependencies as development-dependencies

RUN apt-get update && apt-get install -y \
    libgtest-dev \
    build-essential \
    ssh \
    libssl-dev \
    gdb \
    clang \
    rsync \
    tar \
    python \
    sudo \
    iputils-ping \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# install gtest
WORKDIR /tmp/gtest_build
RUN cmake /usr/src/gtest \
  && make \
  && cp lib/* /usr/local/lib || cp *.a /usr/local/lib

RUN rm -rf /tmp/*


FROM development-dependencies as remote-development

RUN ( \
    echo 'LogLevel DEBUG2'; \
    echo 'PermitRootLogin yes'; \
    echo 'PasswordAuthentication yes'; \
    echo 'Subsystem sftp /usr/lib/openssh/sftp-server'; \
  ) > /etc/ssh/sshd_config_development \
  && mkdir /run/sshd

ENV DEBIAN_FRONTEND=keyboard-interactive
RUN useradd -m remote && yes password | passwd remote

CMD ["/usr/sbin/sshd", "-D", "-e", "-f", "/etc/ssh/sshd_config_development"]


FROM development-dependencies as build-testing
ARG BUILD_TESTING=ON
ARG BUILD_CONTROLLERS=ON
ARG BUILD_DYNAMICAL_SYSTEMS=ON
ARG BUILD_ROBOT_MODEL=ON

WORKDIR /tmp/control_lib
COPY ./source ./

WORKDIR /tmp/control_lib/build
RUN cmake -DBUILD_CONTROLLERS="${BUILD_CONTROLLERS}" \
    -DBUILD_DYNAMICAL_SYSTEMS="${BUILD_DYNAMICAL_SYSTEMS}" \
    -DBUILD_ROBOT_MODEL="${BUILD_ROBOT_MODEL}" \
    -DBUILD_TESTING="${BUILD_TESTING}" .. \
  && make -j all

RUN CTEST_OUTPUT_ON_FAILURE=1 make test


FROM development-dependencies as runtime-demonstrations

WORKDIR /tmp/control_lib
COPY ./source ./

WORKDIR /tmp/control_lib/build
RUN cmake -DBUILD_CONTROLLERS="ON" \
    -DBUILD_DYNAMICAL_SYSTEMS="ON" \
    -DBUILD_ROBOT_MODEL="ON" \
    -DBUILD_TESTING="OFF" .. \
  && make -j all \
  && make install

RUN rm -rf /tmp/control_lib/
ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib:/opt/openrobots/lib/

WORKDIR /tmp/
COPY ./demos ./demos

WORKDIR /tmp/demos/build
RUN cmake .. && make -j all && make install

WORKDIR /usr/local/bin
RUN rm -rf /tmp/demos/