ARG ROS_VERSION
FROM ghcr.io/aica-technology/ros-ws:${ROS_VERSION}

ARG BRANCH=develop
WORKDIR /tmp
RUN git clone --single-branch --branch ${BRANCH} https://github.com/epfl-lasa/control_libraries
RUN bash control_libraries/source/install.sh --auto
RUN sudo ldconfig
RUN rm -rf /tmp/control_libraries/*

WORKDIR /home/${USER}/ros_ws
COPY --chown=${USER} ./ ./src/ros_examples
RUN su ${USER} -c /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; catkin_make install"
