# `ros2_examples` demonstration scripts

## Table of contents:

* [Prerequisites](#prerequisites)
* [Running a demo script](#running-demonstration-scripts)
* [Task Space control loop example](#task_space_control_loop)

## Prerequisites

This package contains the same control loop examples as in the [`control_loop_examples`](../control_loop_examples) demos
folder, but adapted for usage in a ROS2 environment. The whole folder is a fully functional ROS2 package that can be
directly copied in a ROS2 workspace.

There is also a `Dockerfile` provided that encapsulates the whole package in a containerized ROS2 environment. If you
want to run demo with the dockerized environment, you need to do the following first:

```console
cd path/to/desired/location
git clone https://github.com/aica-technology/docker-images.git
cd docker-images/scripts
sudo ./install-aica-docker.sh
```

Visit the [docker-images](https://github.com/aica-technology/docker-images) repository for more information on this.

## Running demonstration scripts

If working with Docker, build and run the image with

```console
./build.sh
aica-docker interactive control-libraries/ros2-demos:latest -u ros2
```

Any additional `docker run` arguments (gpu, network, volumes, etc.) can be specified in the `aica-docker interactive`
command (run `aica-docker interactive -h` for more information).

Running the scripts uses ROS2 commands, e.g. to run the `task_space_control_loop` script:

```console
ros2 run ros2_examples task_space_control_loop
```

Note that there is no `roscore` in ROS2, as opposed to ROS. You can also directly use the launch file that starts any
requirement for the demo to run correctly:

```console
ros2 launch ros2_examples task_space_control_loop.py
```

## `task_space_control_loop`

* **Showcased libraries:** `state_representation`, `dynamical_systems`

This simple demonstration shows how to create a control loop with a `Linear` dynamical system in task space (`CartesianState`).
It moves a pose towards a random attractor in a `100Hz` control loop.
At each control step, the scripts publishes the current and attractor poses using ROS2 `tf2` for a visualization in `rviz2`.
