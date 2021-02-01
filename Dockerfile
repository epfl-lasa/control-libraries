FROM ubuntu:20.04
ENV DEBIAN_FRONTEND=noninteractive

# install dependencies for building the robot_model library
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
COPY ./source/ .
# install libraries and dependencies
RUN . ./install.sh

# change directory
WORKDIR /root

# Clean image
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

CMD ["bash"]