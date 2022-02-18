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
  py::class_<IDynamicalSystem<JointState>, PyDynamicalSystem<JointState>> c(m, "IJointDS");

  c.def(py::init<>());

  c.def("is_compatible", &IDynamicalSystem<JointState>::is_compatible);
  c.def("evaluate", &IDynamicalSystem<JointState>::evaluate);
  c.def("get_base_frame", &IDynamicalSystem<JointState>::get_base_frame);
  c.def("set_base_frame", &IDynamicalSystem<JointState>::set_base_frame);

  c.def("get_parameter", [](IDynamicalSystem<JointState>& self, const std::string& name) {
    return parameter_interface_ptr_to_container(self.get_parameter(name));
  });
  c.def("get_parameters", [](IDynamicalSystem<JointState>& self) {
    py::dict dict;
    for (const auto& param_it : self.get_parameters()) {
      dict[py::str(param_it.first)] = parameter_interface_ptr_to_container(param_it.second);
    }
    return dict;
  });
  c.def("get_parameter_value", [](IDynamicalSystem<JointState>& self, const std::string& name) {
    return parameter_interface_ptr_to_container(self.get_parameter(name)).get_value();
  });
  c.def("get_parameter_list", [](IDynamicalSystem<JointState>& self) {
    py::list list;
    for (const auto& param_it : self.get_parameters()) {
      list.append(parameter_interface_ptr_to_container(param_it.second));
    }
    return list;
  });

  c.def("set_parameter", [](IDynamicalSystem<JointState>& self, const ParameterContainer& parameter) {
    self.set_parameter(container_to_parameter_interface_ptr(parameter));
  });
  c.def("set_parameters", [](IDynamicalSystem<JointState>& self, const std::vector<ParameterContainer>& parameters) {
    for (auto param : parameters) {
      self.set_parameter(container_to_parameter_interface_ptr(param));
    }
  });
  c.def("set_parameters", [](IDynamicalSystem<JointState>& self, const std::map<std::string, ParameterContainer>& parameters) {
    for (const auto& param_it: parameters) {
      self.set_parameter(container_to_parameter_interface_ptr(param_it.second));
    }
  });
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
