FROM ubuntu:20.04
ENV DEBIAN_FRONTEND=noninteractive

# install dependencies for building the libraries
RUN apt-get update && apt-get install -y \
    gcc \
    g++ \
    make \
    git \
    wget \
    cmake \
    && rm -rf /var/lib/apt/lists/*

# import previously downloaded packages
WORKDIR /root/control_lib
# install libraries and dependencies
COPY ./source/state_representation ./state_representation
RUN /bin/bash -c "source ./state_representation/install.sh"

COPY ./source/dynamical_systems ./dynamical_systems
RUN /bin/bash -c "source ./dynamical_systems/install.sh"

COPY ./source/robot_model ./robot_model
RUN /bin/bash -c "source ./robot_model/install.sh"

# change directory
WORKDIR /root

# Clean image
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

CMD ["bash"]