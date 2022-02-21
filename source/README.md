# Core Libraries

## `state_representation`

This library provides a set of classes to represent **states** in **Cartesian** or **joint** spaces.
The classes define and combine variables such as position, velocity, acceleration and force into
a consistent internal representation used across the other libraries.

Source: [state_representation](./state_representation)

Dependencies: [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page)

---

## `dynamical_systems`

This library provides a collection of classes that behave as differential equations to calculate
a derivative of a state variable. For example, a position state input might
yield a desired velocity output. This can be used to generate time-invariant trajectories.

Source: [dynamical_systems](./dynamical_systems)

Dependencies: `state_representation`

---

## `robot_model`

This library allows the creation of a robot model from a URDF file and defines many helpful
rigid-body algorithms for solving kinematic and dynamic problems.

It is a wrapper for [Pinocchio](https://github.com/stack-of-tasks/pinocchio)
that is compatible with the internal `state_representation` types.

Source: [robot_model](./robot_model)

Dependencies: `state_representation`, [Pinocchio](https://stack-of-tasks.github.io/pinocchio/download.html)

---

## `controllers`

This library provides classes designed to convert an input state to an output command in order to control
a robot.

Source: [controllers](./controllers)

Dependencies: `state_representation`, `robot_model`

---


## Installation

### Supported platforms

These libraries have been developed and tested on Linux Ubuntu 18.04 and 20.04.
They should also work on macOS and Windows, though the installation
steps may differ. At this time no guarantees are made for library support on
non-Linux systems.

### Installation with the install script
This project uses CMake to generate static library objects for each of the modules.

To facilitate the installation process, an [install script](./install.sh) is provided. Users who are interested in
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

Eigen3 (release [3.4.0](https://gitlab.com/libeigen/eigen/-/releases/3.4.0)) is required for
building `state_representation` and all other libraries. You can install it with:
```shell script
wget -c https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz -O - | tar -xz
cd eigen-3.4.0 && mkdir build && cd build && cmake .. && make install
cd ../.. && rm -r eigen-3.4.0
```

Pinocchio is required for building the `robot_model` library. Installing this requires
some additional steps; see the [install script](./install.sh) for reference.
If the `robot_model` library is not needed, you can skip the installation of Pinocchio.

Once the dependencies are installed, build and install the libraries by navigating
to the source folder and invoking `cmake` and `make` as shown below.
The library files are installed to `usr/local/lib`, and the library header files
are copied to `/usr/local/include`.

```shell script
cd control-libraries/source
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

## Usage in a cmake project

If you have a target library or executable `my_target`, you can link all required libraries
and include directories automatically with the following commands in the CMakeLists file:

```cmake
find_package(control_libraries [VERSION] CONFIG REQUIRED)

target_link_libraries(my_target ${control_libraries_LIBRARIES})
```

`VERSION` is an optional version number such as `5.0.0`, which should be included for safety. If the installed
control libraries version has a different major increment (for example, `4.1.0` compared to `5.0.0`), cmake
will give the according error.

The `CONFIG` flag tells cmake to directly search for a `control_librariesConfig.cmake` file, which is installed
as part of the package. It is not required but makes the cmake search algorithm slightly more efficient.

The `REQUIRED` flag will give an error if the package is not found, which just prevents build errors from happening
at later stages.

### Advanced specification

For more fine control in checking required components when importing control libraries, it is possible to
supply the individual required libraries to the end of the `find_package` directive. This aborts the cmake 
configuration early if the required component was not found (for example, when the control libraries installation
selectively disabled the `controllers` library).

Additional `OPTIONAL_COMPONENTS` can also be appended, which will silently check if they are installed without aborting.
This is generally not needed.

Some examples below:

```cmake
# ensure that both `dynamical_systems` and `robot_model` are installed and available
find_package(control_libraries 5.0.0 CONFIG REQUIRED dynamical_systems robot_model)

# ensure that the `controllers` library is available, and also check for dynamical_systems in the background
find_package(control_libraries 5.0.0 CONFIG REQUIRED controllers OPTIONAL_COMPONENTS dynamical_systems)
```

### robot_model and Pinocchio

If the `robot_model` library is used and `pinocchio` has been installed to a location that is
not on the default include and link paths, that location must also be provided.
The default installation of `pinocchio` is at `/opt/openrobots`. For this reason it is normal to add the following
lines to the CMakeLists file:

```cmake
list(APPEND CMAKE_PREFIX_PATH /opt/openrobots)
include_directories(/opt/openrobots/include)
```

## Troubleshooting

This section lists common problems that might come up when using the `control-libraries` modules.

### Boost container limit compile error in ROS
When using the `robot_model` module in ROS and trying to `catkin_make` the workspace, it might produce the following error:
```bash
/opt/openrobots/include/pinocchio/container/boost-container-limits.hpp:29:7: error: #error "BOOST_MPL_LIMIT_LIST_SIZE value is lower than the value of PINOCCHIO_BOOST_MPL_LIMIT_CONTAINER_SIZE"
#23 2.389    29 |     # error "BOOST_MPL_LIMIT_LIST_SIZE value is lower than the value of PINOCCHIO_BOOST_MPL_LIMIT_CONTAINER_SIZE"
```
In order to avoid this error and successfully `catkin_make` the workspace, make sure that the `CMakeList.txt` of the ROS
package contains all the necessary directives, i.e. on top of

```cmake
list(APPEND CMAKE_PREFIX_PATH /opt/openrobots)
include_directories(/opt/openrobots/include)
find_package(control_libraries REQUIRED)
```
it should also have
```bash
find_package(Boost REQUIRED COMPONENTS system)
add_compile_definitions(BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS)
add_compile_definitions(BOOST_MPL_LIMIT_LIST_SIZE=30)
```
For a comprehensive example, please check the [`CMakeLists.txt` of the ROS demos](../demos/ros_examples/CMakeLists.txt).
