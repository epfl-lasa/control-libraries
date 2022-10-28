# License

The source files in this repository are made freely available under the GNU General Public License v3.0 (GPLv3).

Any distribution of this software, in its original or modified form, or of derived works which include this software,
must retain the GPLv3 license and make all source code available with a statement of modifications, if applicable.

## Dependencies

The compilation of these source files into binary form and their subsequent execution relies on several open source
softwares. Any distribution of these dependencies must adhere to their respective license agreements. For convenience,
the dependencies are summarized below, and their respective license texts are copied in this directory.

### Compilation Dependencies

This project makes use of open source software provided by Google Inc. under the BSD-3-Clause "New" or "Revised"
License (BSD3) for testing and compilation.

- [Google Test](https://github.com/google/googletest/blob/main/LICENSE) - BSD3, see [COPYING.GTEST](./COPYING.GTEST)
- [Google Protobuf](https://github.com/protocolbuffers/protobuf) - BSD3, see [COPYING.PROTOBUF](./COPYING.PROTOBUF)

The Python bindings of the library are compiled with pybind11, which is also made available under BSD3 license.

- [Pybind11](https://github.com/pybind/pybind11) - BSD3, see [COPYING.PYBIND11](./COPYING.PYBIND11)

## Runtime dependencies

Execution of the compiled software requires Eigen, which is made available under the Mozilla Public License 2.0 (MPL2).

- [Eigen](https://eigen.tuxfamily.org) - see [COPYING.EIGEN](./COPYING.EIGEN)

The `robot_model` and `controllers` modules have additional runtime dependencies: Boost defines a custom permissive
license file; the OSQP library is available under the Apache License 2.0 (Apache-2.0); urdfdom and osqp-eigen are
available under BSD3; Pinocchio is available under the BSD-2-Clause license (BSD2).

- [Boost](https://www.boost.org) - custom license, see [COPYING.BOOST](./COPYING.BOOST)
- [urdfdom](https://github.com/ros/urdfdom) - BSD3, see [COPYING.URDFDOM](./COPYING.URDFDOM)
- [osqp](https://github.com/osqp/osqp) - Apache-2.0, see [COPYING.OSQP](./COPYING.OSQP) and [NOTICE.OSQP](./NOTICE.OSQP)
- [osqp-eigen](https://github.com/robotology/osqp-eigen) - BSD3, see [COPYING.OSQP_EIGEN](./COPYING.OSQP_EIGEN)
- [Pinocchio](https://github.com/stack-of-tasks/pinocchio) - BSD2, see [COPYING.PINOCCHIO](./COPYING.PINOCCHIO)
