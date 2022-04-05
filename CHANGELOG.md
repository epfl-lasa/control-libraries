# CHANGELOG

Release Versions:
- [5.1.0](#510)
- [5.0.0](#500)
- [4.1.0](#410)
- [4.0.0](#400)
- [3.1.0](#310)
- [3.0.0](#300)
- [2.0.0](#200)
- [1.0.0](#100)

## Upcoming changes (in development)

- Throw exception if setting state variable from vector with wrong size (#273)
- Improve installation script and python installation guide (#274)
- Fix path in protocol installation guide (#275)
- Add force limit parameter to Impedance controller (#276)

## 5.1.0

Version 5.1.0 contains a few new features and improvements to the behaviour and usage of the libraries.

### Features

**python**
- Add python bindings for robot model (#263)
- Controllers bindings (#266, #269, #271)

### Fixes and improvements

**protocol**
- Add Docker resources for testing and serving for protocol (#267)

**python**
- Bind ParameterMap (#265, #268)

**state_representation**
- ParameterInterface accessors to underlying parameters (#256)

**general**
- Incremental versioning (#260)
- Refactor cmake to export package (#259, #261)
- Improve CI checks for pull requests (#262)
- Remove previous eigen3 installation (#252, 264)

## 5.0.0

Version 5.0.0 refactors the dynamical systems and controllers libraries with a
factory pattern and parameter interface for easier creation, manipulation and
substitution of these classes. This major change breaks any implementations using
dynamical systems or controllers from the prior version.

See the updated documentation for usage guidelines for the new DynamicalSystemsFactory
and ControllerFactory.

This release also includes substantial improvements to the python bindings, including
class bindings for the dynamical systems library. Additional fixes and improvements
have been made throughout the framework.

### Breaking changes

This release contains the following breaking changes:
- Relocate `Jacobian` and `JointState` family headers
- Refactor dynamical system classes to use `IDynamicalSystem` base interface
- Refactor controller classes to use `IController` base interface

**state_representation**

To make joint-space files more structurally consistent with Cartesian-space files,
the following headers have been relocated.
- `state_representation/robot/Joint*.hpp` headers are now in `state_representation/space/joint/Joint*.hpp`
- `state_representation/robot/Jacobian.hpp` header is now in `state_representation/space/Jacobian.hpp`

Any C++ implementations that include the files from a prior version should update the file paths
in the `#include` directives. The namespaces themselves in C++ and Python are unaffected.

**dynamical_systems**

The previous DS classes inheriting from the concrete `DynamicalSystem` base class have been
refactored and partially renamed to inherit from the new abstract `IDynamicalSystem` base class.

- `dynamical_systems::Blending` has been removed
- `dynamical_systems::Circular` has been refactored
- `dynamical_systems::DynamicalSystem` has been refactored and renamed to `dynamical_systems::IDynamicalSystem`
- `dynamical_systems::Linear` has been refactored and renamed to `dynamical_systems::PointAttractor`
- `dynamical_systems::Ring` has been refactored
- `dynamical_systems::DefaultDynamicalSystem` has been introduced

It is no longer recommended to directly instantiate and use these classes. The new factory method
`dynamical_systems::DynamicalSystemFactory<S>::create_dynamical_system(type)` should be used instead.
See the documentation for more information.

**controllers**

The previous controller classes inheriting from the concrete `Controller` base class have been
refactored and partially renamed to inherit from the new abstract `IController` base class.

- `controllers::Controller` has been refactored and renamed to `controllers::Controller`
- `controllers::CartesianTwistController` has been refactored and renamed to `controllers::CompliantTwist`
- `controllers::Dissipative` has been refactored
- `controllers::Impedance` has been refactored
- `controllers::VelocityImpedance` has been refactored

It is no longer recommended to directly instantiate and use these classes. The new factory method
`controllers::ControllerFactory<S>::create_controller(type)` should be used instead.
See the documentation for more information.

### Features

**state_representation**
- Add CartesianAcceleration class in state representation (#248)

**dynamical_systems**
- Create DS interface and DS factory classes (#227)
- Refactor Linear DS to PointAttractor DS with factory (#229)
- Refactor Circular DS with factory (#230)
- Refactor Ring DS with factory and remove old DS base class (#231)
- Refactor dynamical systems using factory pattern (#233)
- Propagate DS refactor to demos (#234)
- Avoid exception with the default DS in evaluate (#237)
- Add the `set_parameter_value` function in the DS base class (#236)
- Remove the `set_base_frame` logic for `JointState` based DS
  and override `is_compatible` for PointAttractor DS (#236, #239)
- Return a state that has same name as input in PointAttractor (#241)
- Update README of dynamical systems (#242)

**controllers**
- Add IController and ControllerFactory and refactor Controllers (#253, #254, #255)

**python**
- Add python bindings for dynamical_systems module (#238)

**clproto**
- Update pybindings and clproto with CartesianAcceleration (#250)

### Fixes and improvements

**state_representation**
- Add static method to create Parameter pointer (#226)
- Templated get_value method for Parameter (#228)
- Mark the JointState as filled when one index was set (#245)
- Add ParameterMap base class (#247)
- Relocate Jacobian and JointState family files to the 'space/' directory (#249, #251)

**python**
- Add support for copy module in Python bindings (#232)
- Add empty constructors and python bindings for the
  Shape and Ellipsoid classes (#235)

**general**
- Install Eigen version 3.4 manually (#240)

### Behind the scenes

- Fix tag generation for release docs (#225)
- Add user sshd configuration and user 'developer' in base Dockerfile
  and update all the Dockerfiles and scripts (#244, #246)

## 4.1.0

Version 4.1.0 contains a few improvements to the behaviour and usage of the libraries,
namely to the Python bindings and the `clproto` serialization library.

### Fixes and improvements

**clproto**
- JSON conversion support for clproto (#205)
- Support empty state objects in clproto (#207)
- Add int and int array parameters (#208)
- Add build testing option to clproto install (#216)
- Fix the field size type for clproto pack_fields (#222)

**python**
- Python bindings for clproto (#204)
- Add Python bindings for Parameter class (#209)
- Add possibility to have a ssh server for Python testing (#211)
- Add Python bindings for clproto encode / decode of Parameter class (#214)

**state_representation**
- Add method to get a joint state by name or index of the joint,
  and to get the index of the joint by its name (#210)
- Improve CartesianState tests in C++ and Python (#213)
- Add method to set a joint state by name or index of the joint (#217)
- Add the `set_timestamp` method to the `State` base class (#218)

**general**
- Add proto Dockerfile to copy protobuf files from (#221)

## 4.0.0

Version 4.0.0 introduces some powerful new features for using `state_representation` objects
in real applications, including the brand new `clproto` C++ serialization library on the basis of Protobuf.

Many additional fixes and improvements have been made across the modules, and some deprecated methods
have now been removed with this major version release.

This release also marks the repository being renamed to `control-libraries` (formerly `control_libraries`). 

See the following notes for more details.

### Breaking changes

This release contains the following breaking changes:
- Rename repository to control-libraries
- Remove previously deprecated from_std_vector function (#186)
- Remove invalid multiplication operators for CartesianState and its derived classes (#195)

**Repository namespace**

The repository and all associated references have been renamed from `epfl-lasa/control_libraries` to
`epfl-lasa/control-libraries`. This is to better match the GitHub repository standard style and to match
the pattern in the container names. This breaking change will necessitate downstream users to update their 
installation paths when cloning from GitHub.

**state_representation**

The following functions have been removed:
- `CartesianState::from_std_vector`
- `CartesianPose::from_std_vector`
- `Ellipsoid::from_std_vector`
- `JointState::from_std_vector`
- `JointPositions::from_std_vector`

The following multiplication operations are no longer permitted:
- `CartesianState *= CartesianPose | CartesianTwist | CartesianWrench`
- `CartesianPose *= CartesianTwist | CartesianWrench | CartesianState`
- `CartesianTwist *= CartesianPose | CartesianTwist | CartesianWrench | CartesianState`
- `CartesianTwist * CartesianPose | CartesianTwist | CartesianWrench | CartesianState`
- `CartesianWrench *= CartesianPose | CartesianTwist | CartesianWrench | CartesianState`
- `CartesianWrench * CartesianPose | CartesianTwist | CartesianWrench | CartesianState`

The permitted multiplication operations are:
- `CartesianState *= CartesianState`
- `CartesianState * CartesianPose | CartesianTwist | CartesianWrench | CartesianState`
- `CartesianPose *= CartesianPose`
- `CartesianPose * CartesianPose | CartesianTwist | CartesianWrench | CartesianState`

### Features

The biggest feature to come with 4.0.0 is the Protobuf schema for `state_representation` types and the 
associated `clproto` C++ serialization library. See the documentation for more information.

Speaking of documentation, documentation is now generated and hosted for each release and for the main and develop
branches at:
[https://epfl-lasa.github.io/control-libraries](https://epfl-lasa.github.io/control-libraries)

Similarly, docker images for development and downstream use are now also built and hosted automatically.

The `state_representation` API has seen a number of smaller features to introduce new classes, methods and operators.

- Protobuf message protocol and C++ binding library `clproto` for serializing and deserializing control library objects
  (#168, #175, #177, #179, #180, #190)
- Build and push development dependencies image in CI and related restructuring of Docker resources (#169)
- Add automatic documentation generation and deployment to GitHub Pages (#170, #199)
- Add set_data function declaration to all State objects (#163, #166)
- Add class JointAccelerations (#173)
- Methods for packing and unpacking multiple encoded state messages
  into serialized message packets (#182)
- Add scalar division operator for CartesianState and its derived classes (#192)

### Fixes and improvements

- Correct an error in the `makefile` of the protobuf bindings and remove
  the generated bindings from the repository, while providing installation scripts (#174)
- Fix clamp_state_variable function for CartesianState and JointState (#176, #191)
- Install tagged versions of osqp and osqpEigen (#184)
- Add missing integration constructor from JointAccelerations for JointVelocities (#185)
- Fix path to Dockerfile in demos (#197)

### Behind the scenes

- Refactor JointState tests and split them into separate test suites (#183, #187)
- Refactor CartesianState tests and split them into separate test suites (#188)

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
