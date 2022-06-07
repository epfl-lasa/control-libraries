# `control_loop_examples` demonstration scripts

## Table of contents:
* [Running a demo script](#running-demonstration-scripts)
* [Task Space control loop example](#task_space_control_loop)
* [Robot kinematics control loop example](#robot_kinematics_control_loop)

## Running demonstration scripts
This package contains a set of demonstration scripts in both C++ and Python that showcase the functionalities introduced
in the different libraries of `control-libraries`.
The easiest way to run them is to use the `run-demo.sh` file.
Without arguments, this script creates a container and opens in interactive mode, allowing you to browse the different
demo scripts and run the one of your choice:

```console
./run-demo-script.sh
# Run a python script
developer@xxxxxxxxx:~/control_loop_examples$ python3 python_scripts/<script>.py
# Run a cpp script from the build folder
developer@xxxxxxxxx:~/control_loop_examples$ ./build/<script>
```

The available scripts are:

### `task_space_control_loop`
- **Showcased libraries:** `state_representation`, `dynamical_systems`

This simple demonstration shows how to create a control loop with a `PointAttractor` dynamical system in task space
(`CartesianState`). It moves a pose towards a random attractor in a `100Hz` control loop.
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

### `robot_kinematics_control_loop`
- **Showcased libraries:** `state_representation`, `dynamical_systems`, `robot_model`

This demonstration reuses the previous dynamical system control loop but adds the robot component.
The desired command, i.e. desired twist of the robot end-effector (eef) is sent to a dummy robot.
At each timestep, the new joint state and eef state of the robot is computed using the robot kinematics from the robot
model corresponding to the provided URDF.
The robot is assumed to perfectly follow the computed desired state matching the eef desired twist.
Similarly to the previous demonstration, the script outputs the current joint positions, eef pose and distance to the
attractor at each timestep, and the final joint positions and pose on reaching it:

```
franka JointPositions
names: [panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint5, panda_joint6, panda_joint7, ]
positions: [-1.39453, 0.34338, 1.78372, -1.21918, -0.365912, 1.1687, -2.88819, ]
panda_link8 CartesianPose expressed in panda_link0 frame
position: (0.500692, 4.30377e-05, 0.750494)
orientation: (-9.36521e-05, 8.70069e-05, 1, -0.000152405) <=> theta: 3.14141, axis: (-8.70069e-05, -1, 0.000152405)
distance to attractor: 0.001249
-----------
franka JointPositions
names: [panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint5, panda_joint6, panda_joint7, ]
positions: [-1.39627, 0.352394, 1.79511, -1.2227, -0.374677, 1.16757, -2.88157, ]
panda_link8 CartesianPose expressed in panda_link0 frame
position: (0.500192, 1.07333e-05, 0.750183)
orientation: (-1.9014e-05, 4.77102e-05, 1, -7.96025e-05) <=> theta: 3.14155, axis: (-4.77102e-05, -1, 7.96025e-05)
distance to attractor: 0.000455
-----------
##### TARGET #####
panda_link8 CartesianPose expressed in panda_link0 frame
position: (0.5, 0, 0.75)
orientation: (6.12323e-17, 0, 1, 0) <=> theta: 3.14159, axis: (0, 1, 0)
##### CURRENT STATES #####
franka JointPositions
names: [panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint5, panda_joint6, panda_joint7, ]
positions: [-1.39215, 0.362964, 1.80315, -1.22376, -0.385704, 1.16489, -2.87388, ]
panda_link8 CartesianPose expressed in panda_link0 frame
position: (0.500192, 1.07333e-05, 0.750183)
orientation: (-1.9014e-05, 4.77102e-05, 1, -7.96025e-05) <=> theta: 3.14155, axis: (-4.77102e-05, -1, 7.96025e-05)

```