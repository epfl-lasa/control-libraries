# Robot Model

The `robot_model` library is a wrapper for the [Pinocchio](https://github.com/stack-of-tasks/pinocchio) library, used
to compute dynamics model of a robot.
It is specifically designed to work seamlessly with the `state_representation` library, offering conversion functions
between the states and input/output needed for `pinocchio`.

## Table of contents:
* [Model initialization](#model-initialization)
* [Robot kinematics](#robot-kinematics)
* [Robot dynamics](#robot-dynamics)

## Model initialization

The library offers a single class `Model` that encapsulates most of the functionalities from `pinocchio`, as well
as extra functions.
A `Model` is defined by a robot name, and initialized from a `URDF` description file of the robot.

```cpp
robot_model::Model model("myrobot", "path/to/urdf/myrobot.urdf")
```

At creation, it uses the parser from `pinocchio` to read the `URDF` file and fill the needed parameters for computation.

## Robot kinematics

Most common robotics functionalities are implemented such `forward_kinematics`, `inverse_kinematics`,
`forward_velocity, `inverse_velocity`, ...

```cpp
// forward kinematics: pose of end-effector frame from the robot joint positions
state_representation::JointPositions jp = state_representation::JointPositions::Random("myrobot", 7);
state_representation::CartesianPose pose = model.forward_kinematics(jp);
// inverse kinematics: joint positions from end-effector pose
state_representation::CartesianPose cp = state_representation::CartesianPose::Random("eef");
state_representation::JointPositions jp = model.inverse_kinematics(cp);
// forward velocity kinematics: twist of end-effector frame from the robot joint velocities and positions
state_representation::JointState js = state_representation::JointState::Random("myrobot", 7);
state_representation::CartesianTwist twist = model.forward_velocity(js);
// inverse velocity kinematics: joint velocities from end-effector twist and current state of the robot
state_representation::CartesianTwist ct = state_representation::CartesianTwist::Random("eef");
state_representation::JointPositions jp = state_representation::JointPositions::Random("myrobot", 7);
state_representation::JointVelocities jv = model.inverse_velocity(ct, jp);
```

All of those functions check for inconsistencies between the model and the inputs (incompatibility of joints, frame
non existent in the robot model, ...).
Thus, they can be called on other frames than the end-effector (which is the default value), or even vector of frames
for bulk operations.

```cpp
// pose of multiple robot frames from the robot joint positions
state_representation::JointPositions jp = state_representation::JointPositions::Random("myrobot", 7);
auto poses = model.forward_kinematics(jp, std::vector<std::string>{"joint2", "eef_link"});
```

The Jacobian of the robot can also be computed and stored in the `state_representation::Jacobian` wrapper:

```cpp
state_representation::JointPositions jp = state_representation::JointPositions::Random("myrobot", 7);
state_representation::Jacobian = model.compute_jacobian(jp);
```

It can also be computed on lower frames than the end-effector, returning then a subset of the matrix, where non used
joints are filled with 0 values.

```cpp
state_representation::JointPositions jp = state_representation::JointPositions::Random("myrobot", 7);
state_representation::Jacobian = model.compute_jacobian(jp, "joint3");
```

## Robot dynamics

Dynamic modeling of the robot is available within `pinocchio` and allows the computation of the gravity, coriolis, and
inertia torques as well as their matrix counterpart.

```cpp
// inertia matrix from joint positions
state_representation::JointPositions jp = state_representation::JointPositions::Random("myrobot", 7);
Eigen::MatrixXd inertia = model.compute_inertia_matrix(jp);
// inertia torques, equivalent to inertia multiplied by joint accelerations. The joint positions part of the state
// is used to compute the inertia matrix
state_representation::JointState js = state_representation::JointState::Random("myrobot", 7);
state_representation::JointTorques inertia_t = model.compute_inertia_torques(js);

// coriolis matrix from joint positions and velocities
state_representation::JointState js = state_representation::JointState::Random("myrobot", 7);
Eigen::MatrixXd coriolis = model.compute_coriolis_matrix(js);
// coriolis torques, equivalent to coriolis multiplied by joint velocities. The joint positions part of the state
// is used to compute the coriolis matrix
state_representation::JointState js = state_representation::JointState::Random("myrobot", 7);
state_representation::JointTorques coriolis_t = model.compute_coriolis_torques(js);

// gravity torques from joint positions
state_representation::JointPositions jp = state_representation::JointPositions::Random("myrobot", 7);
state_representation::JointTorques gravity_t = model.compute_gravity_torques(jp);
```
