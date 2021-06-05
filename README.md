<table border="0" width="100%" height="120">
    <tr>
        <td width="25%">Branch</td>
        <td width="75%">Status</td>
    </tr>
    <tr>
        <td width="25%"><a href="https://github.com/epfl-lasa/control_libraries/tree/main">Main</a></td>
        <td width="75%"><img src="https://github.com/epfl-lasa/control_libraries/actions/workflows/build-test.yml/badge.svg?branch=main"></td>
    </tr>
    <tr>
        <td width="25%"><a href="https://github.com/epfl-lasa/control_libraries/tree/develop">Development</a></td>
        <td width="75%"><img src="https://github.com/epfl-lasa/control_libraries/actions/workflows/build-test.yml/badge.svg?branch=develop"></td>
    </tr>
</table>

# `control_libraries`
A set of libraries to facilitate the creation of full control loop algorithms,
including trajectory planning, kinematics, dynamics and control.

## `state_representation`

This library provides a set of classes to represent **states** in **Cartesian** or **joint** spaces.
The classes define and combine variables such as position, velocity, acceleration and force into
a consistent internal representation used across the other libraries.

Dependencies: [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page)

[**Documentation**](source/state_representation/README.md)

## `dynamical_systems`

This library provides a collection of classes that behave as differential equations to calculate
a derivative of a state variable. For example, a position state input might
yield a desired velocity output. This can be used to generate time-invariant trajectories.

Dependencies: `state_representation`

[**Documentation**](source/dynamical_systems/README.md)

## `robot_model`

This library allows the creation of a robot model from a URDF file and defines many helpful
rigid-body algorithms for solving kinematic and dynamic problems.

It is a wrapper for [Pinocchio](https://github.com/stack-of-tasks/pinocchio)
that is compatible with the internal `state_representation` types.

Dependencies: `state_representation`, [Pinocchio](https://stack-of-tasks.github.io/pinocchio/download.html)

[**Documentation**](source/robot_model/README.md)

## `controllers`

This library provides classes designed to convert an input state to an output command in order to control
a robot. 

Dependencies: `state_representation`

[**Documentation**](source/controllers/README.md)

---
## Installation

### Supported platforms

These libraries have been developed and tested on Linux Ubuntu 18.04 and 20.04. 
They should also work on macOS and Windows, though the installation 
steps may differ. At this time no guarantees are made for library support on
non-Linux systems.

### Installation with the install script
This project uses CMake to generate static library objects for each of the modules. 

To facilitate the installation process, an [install script](source/install.sh) is provided. Users who are interested in 
the manual installation steps and/or have already installed Pinocchio refer to the 
[manual installation steps](#manual-installation-steps) in the next section.

The install script takes care of all the installation steps, including the installation and configuration of Pinocchio. 
It can be run with several optional arguments:
- `-y`, `--auto`: Any input prompts will be suppressed and install steps automatically approved.
- `-d [path]`, `--dir [path]`: If provided, the installation directory will be changed to `[path]`.
- `--no-controllers`: The controllers library will be excluded from the installation.
- `--no-dynamical-systems`: The dynamical systems library will be excluded from the installation.
- `--no-robot-model`: The robot model library, and therefore Pinocchio, will be excluded from the installation.
- `--build-tests`: The unittest targets will be included in the installation.
- `--clean`: Any previously installed header files from `/usr/local/include` and any shared library files from
`/usr/local/lib` will be deleted before the installation.
- `--cleandir [path]`: Any previously installed header files shared library files from `[path]` will be deleted before 
  the installation.

### Manual installation steps

Eigen3 is required for building `state_representation` and all other libraries.
You can install it with:
```shell script
apt-get install libeigen3-dev
```

Pinocchio is required for building the `robot_model` library. Installing this requires
some additional steps; see the [install script](source/install.sh) for reference.
If the `robot_model` library is not needed, you can skip the installation of Pinocchio.

Once the dependencies are installed, build and install the libraries by navigating
to the source folder and invoking `cmake` and `make` as shown below.
The library files are installed to `usr/local/lib`, and the library header files 
are copied to `/usr/local/include`.

```shell script
cd control_libraries/source
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j
make install
```

The CMake configuration flags `BUILD_CONTROLLERS`, `BUILD_DYNAMICAL_SYSTEMS` and `BUILD_ROBOT_MODEL` 
determine which libraries are built, and are all defined as `ON` by default. 
The building of the `state_representation` library cannot be disabled, as all other libraries depend on it.

To selectively disable the build of a particular library, set the flag to `=OFF`.
For example, the following flags will prevent the `robot_model` library from being built,
which is useful if the Pinocchio dependency is not fulfilled on your system.

```shell script
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_ROBOT_MODEL=OFF ..
```

To also build the library tests, add the CMake flag `-DBUILD_TESTING=ON`.
This requires GTest to be installed on your system. You can then use `make test` to run all test targets.

Alternatively, you can include the source code for each library as submodules in your own CMake project,
using the CMake directive `add_subdirectory(...)` to link it with your project.

## Troubleshooting

This section lists common problems that might come up when using the `control_libraries` modules.

### Boost container limit compile error in ROS
When using the `robot_model` module in ROS and trying to `catkin_make` the workspace, it might produce the following error:
```bash
/opt/openrobots/include/pinocchio/container/boost-container-limits.hpp:29:7: error: #error "BOOST_MPL_LIMIT_LIST_SIZE value is lower than the value of PINOCCHIO_BOOST_MPL_LIMIT_CONTAINER_SIZE"
#23 2.389    29 |     # error "BOOST_MPL_LIMIT_LIST_SIZE value is lower than the value of PINOCCHIO_BOOST_MPL_LIMIT_CONTAINER_SIZE"
```
In order to avoid this error and successfully `catkin_make` the workspace, make sure that the `CMakeList.txt` of the ROS 
package contains all the necessary directives, i.e. on top of
```bash

list(APPEND CMAKE_PREFIX_PATH /opt/openrobots)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(osqp REQUIRED)

find_library(state_representation REQUIRED)
find_library(dynamical_systems REQUIRED)
find_library(controllers REQUIRED)
find_library(robot_model REQUIRED)
...
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${STATE_REPRESENTATION_INCLUDE_DIR}
    ${DYNAMICAL_SYSTEMS_INCLUDE_DIR}
    ${ROBOT_MODEL_INCLUDE_DIR}
    ${CONTROLLERS_INCLUDE_DIR}
    ${PINOCCHIO_INCLUDE_DIR}
    ${OsqpEigen_INCLUDE_DIR}
    /opt/openrobots/include
)
```
it should also have
```bash
find_package(Boost REQUIRED COMPONENTS system)
add_compile_definitions(BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS)
add_compile_definitions(BOOST_MPL_LIMIT_LIST_SIZE=30)
```
For a comprehensive example, please check the [`CMakeLists.txt` of the ROS demos](/demos/ros_examples/CMakeLists.txt).
