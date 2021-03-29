# Python Bindings

This directory defines Python bindings for the control libraries.

## Current status

The Python binding project is currently under development.
Bindings exist for the following libraries and components:

- `py_state_representation`: state_representation
  - CartesianState
  - JointState

## About

[PyBind11](https://PyBind11.readthedocs.io/en/stable/index.html) is used to generate
Python bindings for the classes and functions in control libraries.

A docker container configures a pipenv environment with the necessary PyBind and numpy installations.
This is then used to compile a CMake project using PyBind macros.

The project sources are in the source directory and define the bindings between the Python module and 

THe output of the CMake project is a shared library file specifically formatted for use in Python.
When this library is on the path, it can be imported and used in Python projects.

The tests directory contains some Python scripts that import and check the bindings.

## Generating the Python module 

By executing the [run.sh](./run.sh) script, the docker image will be built, and the container
will be launched to automatically install the pipenv environment, build and install the python library
to a `lib` directory, run the scripts in the `tests` directory, and finally place the user
in an interactive pipenv shell for further testing.