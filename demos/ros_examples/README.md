# `ros_examples` demonstration scripts

## Table of contents:
* [Running a demo script](#running-demonstration-scripts)
* [Task Space control loop example](#task_space_control_loop)

## Running demonstration scripts
This package contains the same control loop examples as in [`control_loop_examples`](../control_loop_examples) demos
folder, but adapted for usage in a ROS environment.
The whole folder is a fully functional ROS package that can be directly copied in a ROS workspace.
There is also a provided `Dockerfile` that encapsulates the whole in a containerized ROS environment.
Running the `run-demo.sh` script opens an interactive container with the ROS environment already setup for running
the demonstration scripts.

In order to run the demonstrations, a `roscore` needs to be running.
One can be started from the interactive container.
Running the scripts uses ROS commands, e.g. to run the `task_space_control_loop` script:

```console
rosrun ros_examples task_space_control_loop
```

You can also directly use the launch file that starts any requirement for the demo to run correctly:

```console
roslaunch ros_examples task_space_control_loop.launch
```

## `task_space_control_loop`
* **Showcased libraries:** `state_representation`, `dynamical_systems`

This simple demonstration shows how to create a control loop with a `Linear` dynamical system in task space (`CartesianState`).
It moves a pose towards a random attractor in a `100Hz` control loop.
At each control step, the scripts publishes the current and attractor poses using ROS `tf` for a visualization in `rviz`.

## `joint_space_velocity_control`
* **Showcased libraries:** `state_representation`, `dynamical_systems`, `robot_model`

This demonstration shows how to create a control loop with a `Linear` dynamical system in task space (`CartesianState`) 
and compute forward kinematics and inverse velocity with a robot `Model`. A ROS subscriber is set up to receive the joint 
state from the `joint_states` topic.
The robot moves a pose towards a random attractor in a `500Hz` control loop.
At each control step, the scripts publishes the desired joint velocity to the `velocity_controller/command` topic.

To run this script and visualize the robot, follow the steps below to launch a *PyBullet* simulation with a ROS interface:

- in a first terminal, run:
    ```bash
    # in a directory of your choice (preferably not in the control-libraries directory)
    git clone https://github.com/domire8/pybullet_ros.git  --depth 1 --branch control-libraries-demo
    cd pybullet_ros/docker
    bash build-run.sh # this will build a docker image and spin up a container
    # inside the container
    roslaunch pybullet_ros franka.launch
    ```

- in a second terminal, run:
    ```bash
    cd control-libraries/demos/ros_examples
    bash run-demo.sh # this will build a docker image and spin up a container
    # inside the container
    roslaunch ros_examples joint_space_velocity_control.launch robot_name:=franka
    ```
