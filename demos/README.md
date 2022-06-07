# `control_loop_examples` demonstration scripts

## Table of contents:
* [Running a demo script](#running-demonstration-scripts)
* [Task Space control loop example](#task_space_control_loop)
* [Robot kinematics control loop example](#robot_kinematics_control_loop)

## Running demonstration scripts
This package contains a set of demonstration scripts that showcase the functionalities introduced in the different
libraries of `control-libraries`.
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
The script outputs the current pose and distance to the attractor at each timestep, and the final pose on reaching it:

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

## `robot_kinematics_control_loop`
* **Showcased libraries:** `state_representation`, `dynamical_systems`, `robot_model`

This demonstration reuses the previous linear dynamical system control loop but adds the robot component.
The desired command, i.e. desired twist of the robot end-effector (eef) is sent to a dummy robot.
At each timestep, the new joint state and eef state of the robot is computed using the robot kinematics from the robot
model corresponding to the provided URDF.
The robot is assumed to perfectly follow the computed desired state matching the eef desired twist.
Similarly to the previous demonstration, the script outputs the current eef pose and distance to the attractor at each
timestep, and the final pose on reaching it:

```
franka JointState
names: [panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint5, panda_joint6, panda_joint7, ]
positions: [-1.38116, 1.75208, 1.77134, -2.26031, 1.55573, 1.84016, -0.858975, ]
velocities: [-0.00535077, -0.0246532, -0.00449986, 0.0149801, -0.0205184, 0.0108183, 0.00971553, ]
accelerations: [-0.00259107, -0.0113026, -0.00313417, -0.00478477, -0.0268934, -0.019793, 0.00379833, ]
torques: [-0.0131606, -0.0179626, -0.0166403, -0.0313393, 0.0205044, 0.021926, 0.0116933, ]
panda_link8 CartesianState expressed in world frame
position: (0.500567, 0.000338813, 0.500072)
orientation: (1, -0.000121156, 5.94241e-05, -8.19635e-05) <=> theta: 0.000315772, axis: (-0.767363, 0.376373, -0.519131)
linear velocity: (0, 0, 0)
angular velocity: (0, 0, 0)
linear acceleration: (0, 0, 0)
angular acceleration: (0, 0, 0)
force: (0, 0, 0)
torque: (0, 0, 0)
distance to attractor: 0.000980
-----------
##### TARGET #####
panda_link8 CartesianPose expressed in world frame
position: (0.5, 0, 0.5)
orientation: (1, 0, 0, 0) <=> theta: 0, axis: (1, 0, 0)
##### CURRENT STATES #####
franka JointState
names: [panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint5, panda_joint6, panda_joint7, ]
positions: [-1.38265, 1.75477, 1.77195, -2.26216, 1.55451, 1.84208, -0.858673, ]
velocities: [-0.00535077, -0.0246532, -0.00449986, 0.0149801, -0.0205184, 0.0108183, 0.00971553, ]
accelerations: [-0.00259107, -0.0113026, -0.00313417, -0.00478477, -0.0268934, -0.019793, 0.00379833, ]
torques: [-0.0131606, -0.0179626, -0.0166403, -0.0313393, 0.0205044, 0.021926, 0.0116933, ]
panda_link8 CartesianState expressed in world frame
position: (0.500567, 0.000338813, 0.500072)
orientation: (1, -0.000121156, 5.94241e-05, -8.19635e-05) <=> theta: 0.000315772, axis: (-0.767363, 0.376373, -0.519131)
linear velocity: (0, 0, 0)
angular velocity: (0, 0, 0)
linear acceleration: (0, 0, 0)
angular acceleration: (0, 0, 0)
force: (0, 0, 0)
torque: (0, 0, 0)
```