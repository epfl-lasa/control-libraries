#include "controllers_bindings.h"

#include <controllers/ControllerFactory.hpp>
#include <controllers/IController.hpp>
#include <robot_model/Model.hpp>
#include <state_representation/space/Jacobian.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/space/joint/JointPositions.hpp>
#include <state_representation/space/joint/JointState.hpp>

#include "py_controller.h"

using namespace state_representation;
using namespace py_parameter;

void cartesian_controller(py::module_& m) {
  py::class_<IController<CartesianState>, std::shared_ptr<IController<CartesianState>>, ParameterMap, PyController<CartesianState>> c(m, "ICartesianController");

  c.def(
      "compute_command", py::overload_cast<const CartesianState&, const CartesianState&>(&IController<CartesianState>::compute_command),
      "Compute the command output based on the commanded state and a feedback state.", "command_state"_a, "feedback_state"_a);
  c.def(
      "compute_command", py::overload_cast<const CartesianState&, const CartesianState&, const Jacobian&>(&IController<CartesianState>::compute_command),
      "Compute the command output in joint space from command and feedback states in task space.", "command_state"_a, "feedback_state"_a, "jacobian"_a);
  c.def(
      "compute_command", py::overload_cast<const CartesianState&, const CartesianState&, const JointPositions&, const std::string&>(&IController<CartesianState>::compute_command),
      "Compute the command output in joint space from command and feedback states in task space.", "command_state"_a, "feedback_state"_a, "joint_positions"_a, "frame_name"_a = std::string(""));

  c.def("get_robot_model", &IController<CartesianState>::get_robot_model, "Get the robot model associated with the controller.");
  c.def("set_robot_model", &IController<CartesianState>::set_robot_model, "Set the robot model associated with the controller.", "robot_model"_a);
}

void bind_cartesian_controllers(py::module_& m) {
  cartesian_controller(m);

  m.def("create_cartesian_controller", [](CONTROLLER_TYPE type, const std::list<ParameterContainer>& parameters) -> py::object {
    return py::cast(CartesianControllerFactory::create_controller(type, container_to_interface_ptr_list(parameters)));
  }, "Create a controller of the desired type with initial parameters.", "type"_a, "parameters"_a);

  m.def("create_cartesian_controller", [](CONTROLLER_TYPE type) -> py::object {
    return py::cast(CartesianControllerFactory::create_controller(type, std::list<std::shared_ptr<ParameterInterface>>()));
  }, "Create a controller of the desired type.", "type"_a);

  m.def("create_cartesian_controller", [](CONTROLLER_TYPE type, const std::list<ParameterContainer>& parameters, const robot_model::Model& robot_model) -> py::object {
    return py::cast(CartesianControllerFactory::create_controller(type, container_to_interface_ptr_list(parameters), robot_model));
  }, "Create a controller of the desired type with initial parameters and an associated robot model.", "type"_a, "parameters"_a, "robot_model"_a);

  m.def("create_cartesian_controller", [](CONTROLLER_TYPE type, const robot_model::Model& robot_model) -> py::object {
    return py::cast(CartesianControllerFactory::create_controller(type, std::list<std::shared_ptr<ParameterInterface>>(), robot_model));
  }, "Create a controller of the desired type with an associated robot model.", "type"_a, "robot_model"_a);
}
