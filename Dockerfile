FROM ubuntu:20.04 AS builder
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
COPY ./source ./
RUN /bin/bash -c "source ./install.sh"

COPY ./source/controllers ./controllers
RUN /bin/bash -c "source ./controllers/install.sh"

# change directory
WORKDIR /root

# clean image
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

# secondary target for development purposes
FROM builder AS development

# install development tools
RUN apt-get update && apt-get install -y \
    sudo \
    gdb \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

ENV USER udev

# create the same user as the host itself
ARG UID=1000
ARG GID=1000
RUN addgroup --gid ${GID} ${USER}
RUN adduser --gecos "udev User" --disabled-password --uid ${UID} --gid ${GID} ${USER}
RUN usermod -a -G dialout ${USER}
RUN echo "${USER} ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/99_aptget
RUN chmod 0440 /etc/sudoers.d/99_aptget && chown root:root /etc/sudoers.d/99_aptget

# choose to run as user
USER ${USER}

# change HOME environment variable
ENV HOME /home/${USER}
WORKDIR ${HOME}

# copy lib from root folder
COPY --from=builder --chown=${USER} /root/control_lib ./control_lib

# change entrypoint to source ~/.bashrc and start in ~
COPY config/entrypoint.sh /entrypoint.sh 
RUN sudo chmod +x /entrypoint.sh ; sudo chown ${USER} /entrypoint.sh ; 

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]