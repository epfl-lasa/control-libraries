# `ros_examples` demonstration scripts

## Table of contents:

* [Prerequisites](#prerequisites)
* [Running a demo script](#running-demonstration-scripts)
* [Task Space control loop example](#task_space_control_loop)
* [Joint Space velocity control loop example](#joint_space_velocity_control)

## Prerequisites

This package contains the same control loop examples as in the [`control_loop_examples`](../control_loop_examples) demos
folder, but adapted for usage in a ROS environment. The whole folder is a fully functional ROS package that can be
directly copied in a ROS workspace.

There is also a `Dockerfile` provided that encapsulates the whole package in a containerized ROS environment. If you
want to run demo with the dockerized environment, you need to do the following first:

```console
cd path/to/desired/location
git clone https://github.com/aica-technology/docker-images.git
cd docker-images/scripts
sudo ./install-aica-docker.sh
```

Visit the [docker-images](https://github.com/aica-technology/docker-images) repository for more information on this.

## Running demonstration scripts

If working with Docker, build and spin up a first terminal (T1) with

```console
./build.sh
aica-docker interactive control-libraries/ros-demos:latest -u ros
```

Any additional `docker run` arguments (gpu, network, volumes, etc.) can be specified in the `aica-docker interactive`
command (run `aica-docker interactive -h` for more information).

In order to run the demonstrations, a `roscore` needs to be running in T1. Then, open a second terminal
with `aica-docker connect control-libraries-ros-demos-latest-runtime -u ros`. Running the scripts uses ROS
commands, e.g. to run the `task_space_control_loop` script:

```console
rosrun ros_examples task_space_control_loop
```

You can also directly use the launch file that starts any requirement for the demo to run correctly:

```console
rolaunch ros_examples task_space_control_loop.launch
```

## `task_space_control_loop`

* **Showcased libraries:** `state_representation`, `dynamical_systems`

This simple demonstration shows how to create a control loop with a `Linear` dynamical system in task
space (`CartesianState`). It moves a pose towards a random attractor in a `100Hz` control loop. At each control step,
the scripts publishes the current and attractor poses using ROS `tf` for a visualization in `rviz`.

## `joint_space_velocity_control`

* **Showcased libraries:** `state_representation`, `dynamical_systems`, `robot_model`

This demonstration shows how to create a control loop with a `Linear` dynamical system in task space (`CartesianState`) 
and compute forward kinematics and inverse velocity with a robot `Model`. A ROS subscriber is set up to receive the joint 
state from the `joint_states` topic.
The robot moves a pose towards a random attractor in a `500Hz` control loop.
At each control step, the scripts publishes the desired joint velocity to the `velocity_controller/command` topic.

To run this script and visualize the robot, follow the steps below to launch a *PyBullet* simulation with a ROS
interface:

- in a terminal (T2), run:
    ```console
    # in a directory of your choice (preferably not in the control_libraries directory)
    git clone https://github.com/domire8/pybullet_ros.git --branch control-libraries-demo --single-branch
    cd pybullet_ros/docker
    ./build-run.sh # this will build a docker image and spin up a container
    # inside the container
    roslaunch pybullet_ros franka.launch
    ```
  
- back in T1, run:
    ```console
    aica-docker interactive control-libraries/ros-demos:latest -u ros --net host --no-hostname
    # inside the container
    roslaunch ros_examples joint_space_velocity_control.launch robot_name:=franka
    ```
