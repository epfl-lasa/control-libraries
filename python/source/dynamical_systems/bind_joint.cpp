#include "dynamical_systems_bindings.h"

#include <dynamical_systems/DefaultDynamicalSystem.hpp>
#include <dynamical_systems/DynamicalSystemFactory.hpp>
#include <dynamical_systems/IDynamicalSystem.hpp>
#include <dynamical_systems/PointAttractor.hpp>
#include <state_representation/parameters/Parameter.hpp>
#include <state_representation/space/joint/JointState.hpp>

#include "parameter_container.h"
#include "py_dynamical_system.h"

using namespace state_representation;

void joint(py::module_& m) {
  py::class_<IDynamicalSystem<JointState>, ParameterMap, PyDynamicalSystem<JointState>> c(m, "IJointDS");

  c.def(py::init<>());

  c.def("is_compatible", &IDynamicalSystem<JointState>::is_compatible);
  c.def("evaluate", &IDynamicalSystem<JointState>::evaluate);
  c.def("get_base_frame", &IDynamicalSystem<JointState>::get_base_frame);
  c.def("set_base_frame", &IDynamicalSystem<JointState>::set_base_frame);
}

void bind_joint(py::module_& m) {
  joint(m);
  py::class_<DefaultDynamicalSystem<JointState>, IDynamicalSystem<JointState>>(m, "JointDefaultDS").def(py::init<>());
  py::class_<PointAttractor<JointState>, IDynamicalSystem<JointState>>(m, "JointPointAttractorDS").def(py::init<>());

  m.def("create_joint_ds", [](DynamicalSystemFactory<CartesianState>::DYNAMICAL_SYSTEM type) -> py::object {
    switch (type) {
      case DynamicalSystemFactory<CartesianState>::DYNAMICAL_SYSTEM::POINT_ATTRACTOR:
        return py::cast(PointAttractor<JointState>());
        break;
      default:
      case DynamicalSystemFactory<CartesianState>::DYNAMICAL_SYSTEM::NONE:
        return py::cast(DefaultDynamicalSystem<JointState>());
        break;
    }
  });
}
