#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <dynamical_systems/IDynamicalSystem.hpp>

namespace py = pybind11;
using namespace pybind11::literals;
using namespace dynamical_systems;

void bind_type(py::module_& m);
void bind_cartesian(py::module_& m);
void bind_joint(py::module_& m);
