#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <controllers/IController.hpp>

#include "parameter_container.h"

namespace py = pybind11;
using namespace pybind11::literals;
using namespace controllers;

void bind_controller_type(py::module_& m);
void bind_computational_space(py::module_& m);
void bind_cartesian_controllers(py::module_& m);
void bind_joint_controllers(py::module_& m);
