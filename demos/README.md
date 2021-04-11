# `control_libraries` demonstration scripts

## Table of contents:
* [Running a demo script](#running-demonstration-scripts)
* [Task Space control loop example](#task_space_control_loop)

## Running demonstration scripts
This package contains a set of demonstration scripts that showcase the functionalities introduced in the different
libraries of `control_libraries`.
The easiest way to run them is to use the `run-demo-script.sh` file.
Without arguments, this script create a demo container and opens in interactive mode, allowing you to browse the different
demo scripts and run the one of your choice:

```console
./run-demo-script.sh
root@xxxxxxxx:/tmp/demos/build# ls
CMakeCache.txt  CMakeFiles  Makefile  cmake_install.cmake  task_space_control_loop ...
root@xxxxxxxx:/tmp/demos/build# ./task_space_control_loop
```

You can also directly specify the name of the demo script to execute in the arguments of the script. In this way,
it runs the demo as command in the container and exit when the demo script finishes:

```console
./run-demo-script.sh task_space_control_loop
```

## `task_space_control_loop`
* **Showcased libraries:** `state_representation`, `dynamical_systems`

This simple demonstration shows how to create a control loop with a `Linear` dynamical system in task space (`CartesianState`).
It moves a pose towards a random attractor in a `100Hz` control loop.
The scripts outputs the current pose and distance to the attractor at each timestep, and the final pose on reaching it:

```console
...
frame CartesianPose expressed in world frame
position: (0.596701, 0.822976, -0.604581)
orientation: (0.246491, -0.314866, -0.896822, 0.189239) <=> theta: 2.64348, axis: (-0.32489, -0.925375, 0.195264)
distance to attractor: 0.001004
-----------
frame CartesianPose expressed in world frame
position: (0.596702, 0.822979, -0.604584)
orientation: (0.246488, -0.314866, -0.896823, 0.189239) <=> theta: 2.64348, axis: (-0.324891, -0.925375, 0.195264)
distance to attractor: 0.000994
-----------
##### TARGET #####
frame CartesianPose expressed in world frame
position: (0.59688, 0.823295, -0.604897)
orientation: (0.246242, -0.314924, -0.896867, 0.189256) <=> theta: 2.64399, axis: (-0.324929, -0.92536, 0.195269)
##### CURRENT POSE #####
frame CartesianPose expressed in world frame
position: (0.596702, 0.822979, -0.604584)
orientation: (0.246488, -0.314866, -0.896823, 0.189239) <=> theta: 2.64348, axis: (-0.324891, -0.925375, 0.195264)
```
