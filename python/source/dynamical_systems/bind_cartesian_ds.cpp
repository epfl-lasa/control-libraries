#include "dynamical_systems_bindings.h"

#include <dynamical_systems/DynamicalSystemFactory.hpp>
#include <dynamical_systems/IDynamicalSystem.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>

#include "py_dynamical_system.h"

using namespace state_representation;
using namespace py_parameter;

void cartesian(py::module_& m) {
  py::object parameter_map = py::module_::import("state_representation").attr("ParameterMap");
  py::class_<IDynamicalSystem<CartesianState>, std::shared_ptr<IDynamicalSystem<CartesianState>>, PyDynamicalSystem<CartesianState>> c(m, "ICartesianDS", parameter_map);

  c.def("is_compatible", &IDynamicalSystem<CartesianState>::is_compatible);
  c.def("evaluate", &IDynamicalSystem<CartesianState>::evaluate);
  c.def("get_base_frame", &IDynamicalSystem<CartesianState>::get_base_frame);
  c.def("set_base_frame", &IDynamicalSystem<CartesianState>::set_base_frame);
}

void bind_cartesian_ds(py::module_& m) {
  cartesian(m);

  m.def("create_cartesian_ds", [](DYNAMICAL_SYSTEM_TYPE type, const std::list<ParameterContainer>& parameters) -> py::object {
    return py::cast(CartesianDynamicalSystemFactory::create_dynamical_system(type, container_to_interface_ptr_list(parameters)));
  }, "Create a dynamical system of the desired type with initial parameters.", "type"_a, "parameters"_a);

  m.def("create_cartesian_ds", [](DYNAMICAL_SYSTEM_TYPE type) -> py::object {
    return py::cast(CartesianDynamicalSystemFactory::create_dynamical_system(type, std::list<std::shared_ptr<ParameterInterface>>()));
  }, "Create a dynamical system of the desired type.", "type"_a);
}
