#include "dynamical_systems_bindings.h"

#include <dynamical_systems/Circular.hpp>
#include <dynamical_systems/DefaultDynamicalSystem.hpp>
#include <dynamical_systems/DynamicalSystemFactory.hpp>
#include <dynamical_systems/IDynamicalSystem.hpp>
#include <dynamical_systems/PointAttractor.hpp>
#include <dynamical_systems/Ring.hpp>
#include <state_representation/parameters/Parameter.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>

#include "parameter_container.h"
#include "py_dynamical_system.h"

using namespace state_representation;

void cartesian(py::module_& m) {
  py::class_<IDynamicalSystem<CartesianState>, ParameterMap, PyDynamicalSystem<CartesianState>> c(m, "ICartesianDS");

  c.def(py::init<>());

  c.def("is_compatible", &IDynamicalSystem<CartesianState>::is_compatible);
  c.def("evaluate", &IDynamicalSystem<CartesianState>::evaluate);
  c.def("get_base_frame", &IDynamicalSystem<CartesianState>::get_base_frame);
  c.def("set_base_frame", &IDynamicalSystem<CartesianState>::set_base_frame);
}

void bind_cartesian(py::module_& m) {
  cartesian(m);
  py::class_<Circular, IDynamicalSystem<CartesianState>>(m, "CartesianCircularDS").def(py::init<>());
  py::class_<DefaultDynamicalSystem<CartesianState>, IDynamicalSystem<CartesianState>>(m, "CartesianDefaultDS").def(py::init<>());
  py::class_<PointAttractor<CartesianState>, IDynamicalSystem<CartesianState>>(m, "CartesianPointAttractorDS").def(py::init<>());
  py::class_<Ring, IDynamicalSystem<CartesianState>>(m, "CartesianRingDS").def(py::init<>());

  m.def("create_cartesian_ds", [](DynamicalSystemFactory<CartesianState>::DYNAMICAL_SYSTEM type) -> py::object {
    switch (type) {
      case DynamicalSystemFactory<CartesianState>::DYNAMICAL_SYSTEM::CIRCULAR:
        return py::cast(Circular());
        break;
      case DynamicalSystemFactory<CartesianState>::DYNAMICAL_SYSTEM::POINT_ATTRACTOR:
        return py::cast(PointAttractor<CartesianState>());
        break;
      case DynamicalSystemFactory<CartesianState>::DYNAMICAL_SYSTEM::RING:
        return py::cast(Ring());
        break;
      default:
      case DynamicalSystemFactory<CartesianState>::DYNAMICAL_SYSTEM::NONE:
        return py::cast(DefaultDynamicalSystem<CartesianState>());
        break;
    }
  });
}
