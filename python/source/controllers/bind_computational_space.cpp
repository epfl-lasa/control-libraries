#include "controllers_bindings.h"

#include <controllers/impedance/Dissipative.hpp>

void bind_computational_space(py::module_& m) {
  py::enum_<impedance::ComputationalSpaceType>(m, "COMPUTATIONAL_SPACE")
      .value("LINEAR", impedance::ComputationalSpaceType::LINEAR)
      .value("ANGULAR", impedance::ComputationalSpaceType::ANGULAR)
      .value("DECOUPLED_TWIST", impedance::ComputationalSpaceType::DECOUPLED_TWIST)
      .value("FULL", impedance::ComputationalSpaceType::FULL)
      .export_values();
}
