#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <dynamical_systems/IDynamicalSystem.hpp>

#include "parameter_container.h"

namespace py = pybind11;
using namespace pybind11::literals;
using namespace dynamical_systems;

void bind_ds_type(py::module_& m);
void bind_cartesian_ds(py::module_& m);
void bind_joint_ds(py::module_& m);
