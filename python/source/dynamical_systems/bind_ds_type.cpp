#include "dynamical_systems_bindings.h"

#include <dynamical_systems/DynamicalSystemType.hpp>

using namespace state_representation;

void bind_ds_type(py::module_& m) {
  py::enum_<DYNAMICAL_SYSTEM_TYPE>(m, "DYNAMICAL_SYSTEM_TYPE")
      .value("NONE", DYNAMICAL_SYSTEM_TYPE::NONE)
      .value("CIRCULAR", DYNAMICAL_SYSTEM_TYPE::CIRCULAR)
      .value("POINT_ATTRACTOR", DYNAMICAL_SYSTEM_TYPE::POINT_ATTRACTOR)
      .value("RING", DYNAMICAL_SYSTEM_TYPE::RING)
      .export_values();
}
