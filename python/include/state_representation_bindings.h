#pragma once

#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>

#include <eigen3/Eigen/Core>

#include <state_representation/State.hpp>

namespace py = pybind11;
using namespace pybind11::literals;
using namespace state_representation;

void bind_state(py::module_& m);
void bind_cartesian_space(py::module_& m);
void bind_joint_space(py::module_& m);
void bind_jacobian(py::module_& m);
void bind_parameters(py::module_& m);
void bind_geometry(py::module_& m);
