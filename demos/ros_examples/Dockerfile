FROM control-libraries/source-dependencies as ros-dependencies

# install ROS
ENV ROS_DISTRO="noetic"
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt update && apt install -y ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-tf \
    ros-${ROS_DISTRO}-rviz \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    sudo \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN echo "Set disable_coredump false" >> /etc/sudo.conf

# create a user with same permissions as the host itself
ENV USER ros
ARG UID=1000
ARG GID=1000
RUN addgroup --gid ${GID} ${USER}
RUN adduser --gecos "ROS User" --disabled-password --uid ${UID} --gid ${GID} ${USER}
RUN usermod -a -G dialout ${USER}
RUN echo "${USER} ALL=(ALL) NOPASSWD: ALL" | sudo tee -a /etc/sudoers.d/99_aptget > /dev/null \
    && chmod 0440 /etc/sudoers.d/99_aptget \
    && chown root:root /etc/sudoers.d/99_aptget

# change user to the newly created one
USER ${USER}
ENV HOME /home/${USER}

# add and build a ros workspace
ENV ROS_WORKSPACE ${HOME}/ros_ws
RUN mkdir -p ${ROS_WORKSPACE}/src
WORKDIR ${ROS_WORKSPACE}/src
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; catkin_init_workspace"
WORKDIR ${ROS_WORKSPACE}
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; catkin_make"

# change .bashrc
COPY config/update_bashrc /sbin/update_bashrc
RUN sudo chmod +x /sbin/update_bashrc ; \
    sudo chown ${USER} /sbin/update_bashrc sync ; \
    /bin/bash -c /sbin/update_bashrc ; \
    sudo rm /sbin/update_bashrc

# change entrypoint to source ~/.bashrc and start in ~
COPY config/entrypoint.sh /ros_entrypoint.sh
RUN sudo chmod +x /ros_entrypoint.sh ; sudo chown ${USER} /ros_entrypoint.sh ;

# env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute


FROM ros-dependencies as runtime-demonstrations

# copy the whole folder in the ros_ws
COPY . ${ROS_WORKSPACE}/src/ros_examples

# build and install the package
WORKDIR ${ROS_WORKSPACE}
RUN /bin/bash -c "source ${ROS_WORKSPACE}/devel/setup.bash; catkin_make install"

# Clean image
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
