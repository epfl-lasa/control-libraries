# CHANGELOG

Release Versions:
- [3.1.0](#310)
- [3.0.0](#300)
- [2.0.0](#200)
- [1.0.0](#100)

## Upcoming changes (in development)

- Add set_data function (#163)
- Move set_data declaration to State and add it for Ellipsoid (#166)

## 3.1.0

Version 3.1.0 contains a few improvements to the behaviour and usage
of the libraries.

### Fixes and improvements

**python**
- Fix CartesianPose constructor in Python bindings and revise run script (#157)
- Add CI workflow for Python bindings (#159)

**demos**
- Update ROS1 example with a more developed simulation (#160)

**state_representation**
- Preserve emptiness upon copy construction of CartesianState and JointState types (#152)
- Define `inverse` and `*` operators for Cartesian types explicitly (#158)

**dynamical_systems**
- Add emtpy constructors for Circular and Ring DS (#154)

**general**
- Use release configuration in install script (#155)

### Pending TODOs for the next release

- Revise `*=` and `*` operators in Cartesian types before the next release with 
  breaking changes (some are marked *deprecated*, and some are left as is, but 
  they should be deleted) (#156)
- Add the wrench computation in the `*` operator and `inverse` function (#134)
- Refactor and improve unittests for state_representation (especially JointState
  and CartesianState)

## 3.0.0

Version 3.0.0 introduces Python bindings for the most commonly used
`state_representation` classes, a ROS demo showcasing `robot_model`,
a simple `CartesianTwistController` for 6DOF twist, and a handful
of useful improvements to the`state_representation` API.

### Breaking changes

A very minor breaking change to the API is the removal of two setters
from the `state_representation::Jacobian` class.

**state_representation**

The following functions have been removed:
- `Jacobian::set_rows`
- `Jacobian::set_cols`

### Features

**python**

A top-level `python` directory has been added to provide an
installable Python version of `state_representation`, including
the many useful transformation operators between state types.
- Add initial Python bindings for state_representation classes (#113, #137, #140)

**demos**
- Add a joint velocity control demo in the ROS demo package that 
showcases the `robot_model` module (#136, #139)

**controllers**
- Add a concrete CartesianTwistController to easily control
linear and angular velocity with a set of 4 gains (#135)

**state_representation**
- Add a function to check compatibility between a state and a dynamical system (#142)
- Add constructor for the CartesianPose with only a quaternion provided (#145)
- Add empty constructor for the Jacobian (#146)
- Add empty constructors for JointState and CartesianState linear DS and improve unittests (#147)

### Fixes and improvements
- Remove `set_rows` and `set_cols` from Jacobian class due to inexpedience (#144)
- Fix operators in JointState (#148)

### Behind the scenes
- RSA authentication for development server and better launch script (#149)

## 2.0.0

Version 2.0.0 introduces usage demos for CMake and ROS/ROS2 projects,
a number of new helpful methods for the `robot_model::Model` class, and 
a new type of `VelocityImpedance` controller.

Many additional fixes and improvements have been made across the modules.

The major version increment is due to some breaking changes in the `robot_model` API.

### Breaking changes

**robot_model**

Breaking API changes include the renaming of some
functions in the namespace `robot_model::Model` in an effort
to improve consistency and reduce confusion. 

The following functions have been renamed:
- `forward_geometry()` -> `forward_kinematics()`
- `forward_kinematic()` -> `forward_velocity()`
- `inverse_kinematic()` -> `inverse_velocity()`


### Features

**demos**

A top-level `demos` directory has been added to contain various usage examples
and demonstrations for the control libraries.
- Add CMake control loop examples with robot kinematics and task-space control (#96, #97)
- Add ROS demo package (#115)
- Add ROS2 demo package (#117)

**state_representation**
- Implement Jacobian operations with other matrices and Jacobian objects (#92)

**robot_model**
- Add an inverse kinematics function to calculate joint positions for a cartesian position of a robot model (#46)
- Add a function to check if a joint state is in range for a given robot model (#91)
- Add a getter function to retrieve the pinocchio model of a given robot model (#111)
- Add a function to calculate the Jacobian time derivative (#118)
- Add basic and QP variant functions for inverse velocity (#123)

**controllers**
- Add a velocity impedance controller (#94)

### Fixes and improvements
- Fix Ring DS rotation offset and boundary condition (#90)
- Correct copy constructor from derived Cartesian classes (#100)
- Correct copy constructor from derived Joint classes (#101)
- Delete inaccessible getters and setters (#102, #103)
- Fix Quaternion distance calculation (#105)
- Correct joint names potential mismatch in inverse kinematics (#106)
- Do not run the demos container if building fails (#107)
- Remove the undefined operators that were cause linking issues (#108)
- Revise install script (#89, #121)
- Fix the inverse velocity (#123)

### Behind the scenes
- Improve kinematics test coverage (#86)
- Fix indentations issues and minor code duplication (#93)
- CI job to build and test in Release configuration (#87)
- Mark functions as static and const where possible (#95)
- Refactor kinematics and geometry (#104)
- Fix Model kinematics tests (#112)
- Add function to test robot frame existence and return its id (#128)

## 1.0.0
A set of libraries to facilitate the creation of full control loop algorithms,
including trajectory planning, kinematics, dynamics and control.

### Features

**state_representation**

This library provides a set of classes to represent states in Cartesian or joint spaces.
The classes define and combine variables such as position, velocity, acceleration and force into
a consistent internal representation used across the other libraries.

**dynamical_systems**

This library provides a collection of classes that behave as differential equations to calculate
a derivative of a state variable. For example, a position state input might
yield a desired velocity output. This can be used to generate time-invariant trajectories.

**robot_model**

This library allows the creation of a robot model from a URDF file and defines many helpful
rigid-body algorithms for solving kinematic and dynamic problems.

It is a wrapper for Pinocchio that is compatible with the internal state_representation types.

**controllers**

This library provides classes designed to convert an input state to an output command in order to control
a robot.
