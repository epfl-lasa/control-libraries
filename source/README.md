# Core libraries

## `state_representation`

This library provides a set of classes to represent **states** in **Cartesian** or **joint** spaces.
The classes define and combine variables such as position, velocity, acceleration and force into
a consistent internal representation used across the other libraries.

Dependencies: [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page)

[**Documentation**](source/state_representation/README.md)

---

## `dynamical_systems`

This library provides a collection of classes that behave as differential equations to calculate
a derivative of a state variable. For example, a position state input might
yield a desired velocity output. This can be used to generate time-invariant trajectories.

Dependencies: `state_representation`

[**Documentation**](source/dynamical_systems/README.md)

---

## `robot_model`

This library allows the creation of a robot model from a URDF file and defines many helpful
rigid-body algorithms for solving kinematic and dynamic problems.

It is a wrapper for [Pinocchio](https://github.com/stack-of-tasks/pinocchio)
that is compatible with the internal `state_representation` types.

Dependencies: `state_representation`, [Pinocchio](https://stack-of-tasks.github.io/pinocchio/download.html)

[**Documentation**](source/robot_model/README.md)

---

## `controllers`

This library provides classes designed to convert an input state to an output command in order to control
a robot.

Dependencies: `state_representation`

[**Documentation**](source/controllers/README.md)