#include "dynamical_systems_bindings.h"

#include <dynamical_systems/DynamicalSystemFactory.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>

using namespace state_representation;

void bind_type(py::module_& m) {
  py::enum_<DynamicalSystemFactory<CartesianState>::DYNAMICAL_SYSTEM>(m, "DYNAMICAL_SYSTEM")
      .value("NONE", DynamicalSystemFactory<CartesianState>::DYNAMICAL_SYSTEM::NONE)
      .value("CIRCULAR", DynamicalSystemFactory<CartesianState>::DYNAMICAL_SYSTEM::CIRCULAR)
      .value("POINT_ATTRACTOR", DynamicalSystemFactory<CartesianState>::DYNAMICAL_SYSTEM::POINT_ATTRACTOR)
      .value("RING", DynamicalSystemFactory<CartesianState>::DYNAMICAL_SYSTEM::RING)
      .export_values();
}
