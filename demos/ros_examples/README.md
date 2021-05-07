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

You can also directly use the launchfile that starts any requirement for the demo to run correctly:

```console
rolaunch ros_examples task_space_control_loop.launch
```

## `task_space_control_loop`
* **Showcased libraries:** `state_representation`, `dynamical_systems`

This simple demonstration shows how to create a control loop with a `Linear` dynamical system in task space (`CartesianState`).
It moves a pose towards a random attractor in a `100Hz` control loop.
At each control step, the scripts publishes the current and attractor poses using ROS `tf` for a visualization in `rviz`.
