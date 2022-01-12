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
  py::class_<IDynamicalSystem<CartesianState>, PyDynamicalSystem<CartesianState>> c(m, "ICartesianDS");

  c.def(py::init<>());

  c.def("is_compatible", &IDynamicalSystem<CartesianState>::is_compatible);
  c.def("evaluate", &IDynamicalSystem<CartesianState>::evaluate);
  c.def("get_base_frame", &IDynamicalSystem<CartesianState>::get_base_frame);
  c.def("set_base_frame", &IDynamicalSystem<CartesianState>::set_base_frame);

  c.def("get_parameter", [](IDynamicalSystem<CartesianState>& self, const std::string& name) {
    return parameter_interface_ptr_to_container(self.get_parameter(name));
  });
  c.def("get_parameters", [](IDynamicalSystem<CartesianState>& self) {
    py::dict dict;
    for (const auto& param_it : self.get_parameters()) {
      dict[py::str(param_it.first)] = parameter_interface_ptr_to_container(param_it.second);
    }
    return dict;
  });
  c.def("get_parameter_value", [](IDynamicalSystem<CartesianState>& self, const std::string& name) {
    return parameter_interface_ptr_to_container(self.get_parameter(name)).get_value();
  });
  c.def("get_parameter_list", [](IDynamicalSystem<CartesianState>& self) {
    py::list list;
    for (const auto& param_it : self.get_parameters()) {
      list.append(parameter_interface_ptr_to_container(param_it.second));
    }
    return list;
  });

  c.def("set_parameter", [](IDynamicalSystem<CartesianState>& self, const ParameterContainer& parameter) {
    self.set_parameter(container_to_parameter_interface_ptr(parameter));
  });
  c.def("set_parameters", [](IDynamicalSystem<CartesianState>& self, const std::vector<ParameterContainer>& parameters) {
    for (auto param : parameters) {
      self.set_parameter(container_to_parameter_interface_ptr(param));
    }
  });
  c.def("set_parameters", [](IDynamicalSystem<CartesianState>& self, const std::map<std::string, ParameterContainer>& parameters) {
    for (const auto& param_it: parameters) {
      self.set_parameter(container_to_parameter_interface_ptr(param_it.second));
    }
  });
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
