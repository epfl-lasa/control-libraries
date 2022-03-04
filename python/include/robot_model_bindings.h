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

#include <robot_model/Model.hpp>

namespace py = pybind11;
using namespace pybind11::literals;
using namespace robot_model;

void bind_model(py::module_& m);