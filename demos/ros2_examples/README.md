# `ros2_examples` demonstration scripts

## Table of contents:
* [Running a demo script](#running-demonstration-scripts)
* [Task Space control loop example](#task_space_control_loop)

## Running demonstration scripts
This package contains the same control loop examples as in the [`control_loop_examples`](../control_loop_examples) demos
folder, but adapted for usage in a ROS2 environment.
The whole folder is a fully functional ROS2 package that can be directly copied in a ROS2 workspace.
There is also a provided `Dockerfile` that encapsulates the whole in a containerized ROS2 environment.
Running the `run-demo.sh` script opens an interactive container with the ROS2 environment already setup for running
the demonstration scripts.

As opposed to ROS, there is no `roscore` in ROS2.
Running the scripts uses ROS2 commands, e.g. to run the `task_space_control_loop` script:

```console
ros2 run ros2_examples task_space_control_loop
```

You can also directly use the launch file that starts any requirement for the demo to run correctly:

```console
ros2 launch ros2_examples task_space_control_loop.py
```

## `task_space_control_loop`
* **Showcased libraries:** `state_representation`, `dynamical_systems`

This simple demonstration shows how to create a control loop with a `Linear` dynamical system in task space (`CartesianState`).
It moves a pose towards a random attractor in a `100Hz` control loop.
At each control step, the scripts publishes the current and attractor poses using ROS2 `tf2` for a visualization in `rviz2`.
