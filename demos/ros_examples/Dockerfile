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
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# add and build a ros workspace
WORKDIR /root/ros_ws/src
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; catkin_init_workspace"
WORKDIR /root/ros_ws/
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; catkin_make"

# Change .bashrc
COPY config/update_bashrc /sbin/update_bashrc
RUN chmod +x /sbin/update_bashrc ; sync ; /bin/bash -c /sbin/update_bashrc ; rm /sbin/update_bashrc

# Change entrypoint to source ~/.bashrc and start in ~
COPY config/entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh


FROM ros-dependencies as runtime-demonstrations

# copy the whole folder in the ros_ws
COPY . /root/ros_ws/src/ros_examples

# build and install the package
WORKDIR /root/ros_ws/
RUN /bin/bash -c "source /root/ros_ws/devel/setup.bash; catkin_make install"

# Clean image
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
