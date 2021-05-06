FROM control-libraries/source-dependencies as ros-dependencies

# install ROS
ENV ROS_DISTRO="foxy"
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
    tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update && apt install -y ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-rviz2 \
    python3-colcon-common-extensions \
    python3-argcomplete \
    build-essential \
    sudo \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# create a user with same permissions as the host itself
ENV USER ros2
ARG UID=1000
ARG GID=1000
RUN addgroup --gid ${GID} ${USER}
RUN adduser --gecos "ROS2 User" --disabled-password --uid ${UID} --gid ${GID} ${USER}
RUN usermod -a -G dialout ${USER}
RUN echo "${USER} ALL=(ALL) NOPASSWD: ALL" | sudo tee -a /etc/sudoers.d/99_aptget > /dev/null \
    && chmod 0440 /etc/sudoers.d/99_aptget \
    && chown root:root /etc/sudoers.d/99_aptget

# change user to the newly created one
USER ${USER}
ENV HOME /home/${USER}

# add and build a ros workspace
ENV ROS_WORKSPACE ${HOME}/ros2_ws
RUN mkdir -p ${ROS_WORKSPACE}/src
WORKDIR ${ROS_WORKSPACE}
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; colcon build"

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
COPY . ${ROS_WORKSPACE}/src/ros2_examples

# build and install the package
WORKDIR ${ROS_WORKSPACE}
RUN /bin/bash -c "source ${ROS_WORKSPACE}/install/setup.bash; colcon build --symlink-install"

# Clean image
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
