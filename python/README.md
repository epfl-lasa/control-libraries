# Python Bindings

This directory defines Python bindings for the control libraries.

## Installation

You must first install `control-libraries` before you can install the Python bindings.
Refer to the installation instructions in the top-level [README](../README.md) for more information.

Additionally, the installation of the bindings requires the following prerequisites:
- `python3` >= 3.0
- `pip3` >= 10.0.0

The installation itself is then quite straightforward:
```shell script
git clone https://github.com/epfl-lasa/control-libraries

## install control-libraries (skip this stage if already done)
bash control-libraries/source/install.sh

## install the bindings using the pip installer
pip3 install control-libraries/python
```

The example above installs the module to the default dist-packages location.
You can see more information about the installed module using `pip3 show control-libraries`.

The process also works with Python virtual environments. For example, with `pipenv`:
```shell script
## pip3 install pipenv

pipenv install control-libraries/python
```

Once installed, you can simply import the module with an optional short alias:
```python
#!/usr/bin/env python
import state_representation as sr

print(sr.__version__)

A = sr.CartesianState.Random("A")
print(A)
```

Or, directly import specific classes from the module.
```python
#!/usr/bin/env python
from state_representation import JointState

B = JointState.Random("B", 3)
```

If the `clproto` C++ library is installed, the installation steps above will automatically install the `clproto`
Python module which can be used to encode and decode objects into bytes of serialized data.
```python
#!/usr/bin/env python
from state_representation import JointState
import clproto

B = JointState.Random("B", 3)
encoded_msg = clproto.encode(B, clproto.MessageType.JOINT_STATE_MESSAGE)

decoded_object = clproto.decode(encoded_msg)
```

## Current status

The Python binding project is currently under development.
Bindings exist for the following modules, classes and methods:

- `state_representation`:
  - `State`
  - `SpatialState`
  - `CartesianState`
  - `CartesianPose`
  - `CartesianTwist`
  - `CartesianWrench`
  - `JointState`
  - `JointPositions`
  - `JointVelocities`
  - `JointTorques`
  - `Jacobian`
- `clproto`:
  - `MessageType`
  - `ParameterMessageType`
  - `is_valid(msg)`
  - `check_message_type(msg)`
  - `check_parameter_message_type(msg)`
  - `encode(obj, type)`
  - `decode(msg)`
  - `pack_fields(encoded_fields)`
  - `unpack_fields(packet)`

## About

[PyBind11](https://PyBind11.readthedocs.io/en/stable/index.html) is used to generate
Python bindings for the classes and functions in control libraries.

The generated package is named `control-libraries`, but contains specific modules for importing. 
These are named the same as the standard modules of control libraries (e.g. `state_representation`).

The contents of the [`source`](./source) directory define the bindings between
each Python module and the respective C++ library. The source files to bind each module are
contained within a subdirectory of the same name.

The `setup.py` and `pyproject.toml` files are used to configure the build and installation
of the Python bindings. The `.toml` file allows `pip` to automatically fetch the 
installation dependencies (namely `setuptools` and `pybind11`) in a temporary cache,
allowing the subsequent `setup.py` to be evaluated without needing a local installation of `pybind11`.
This feature requires a [`pip`](https://pypi.org/project/pip/) version 10.0 or newer.

The [`test`](./test) directory contains some Python scripts that import and check the bindings
using the Python `unittest` framework. They are not currently comprehensive.

## Dockerfile

A Dockerfile and run script are provided to test the installation of the bindings.

The docker image installs the core control libraries and subsequently installs the python bindings.

The [`run.sh`](./run.sh) script will build the docker image and launch an interactive container
with the test files in the [`test`](./test) directory copied to the local path.

The run script tries to the clone the current local git branch when installing the control libraries
in the Dockerfile. If the local branch does not exist on the remote, or if you want to test the 
python bindings against a difference control libraries source branch, you can supply a specific
branch as the first argument to the run script. For example, `./run.sh develop` to use the `develop` branch.

You can run a single test with `python <test_name.py>`, or just enter a python shell with `python`.
Run all tests with `python -m unittest discover <path_to_test_dir>`, or just `python -m unittest` if
the [`test`](./test) directory in your current working directory.
