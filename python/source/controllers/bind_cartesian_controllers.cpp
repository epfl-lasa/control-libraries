#include "controllers_bindings.h"

#include <controllers/ControllerFactory.hpp>
#include <controllers/IController.hpp>
#include <controllers/exceptions/InvalidControllerException.hpp>
#include <controllers/impedance/CompliantTwist.hpp>
#include <controllers/impedance/Dissipative.hpp>
#include <controllers/impedance/Impedance.hpp>
#include <controllers/impedance/VelocityImpedance.hpp>
#include <robot_model/Model.hpp>
#include <state_representation/space/Jacobian.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/space/joint/JointPositions.hpp>
#include <state_representation/space/joint/JointState.hpp>

#include "py_controller.h"

using namespace state_representation;
using namespace py_parameter;

py::object
cartesian_controller_factory(CONTROLLER_TYPE type, std::list<std::shared_ptr<ParameterInterface>> parameter_list) {
  switch (type) {
    case CONTROLLER_TYPE::IMPEDANCE:
      return py::cast(impedance::Impedance<CartesianState>(parameter_list));
    case CONTROLLER_TYPE::VELOCITY_IMPEDANCE:
      return py::cast(impedance::VelocityImpedance<CartesianState>(parameter_list));
    case CONTROLLER_TYPE::DISSIPATIVE:
      return py::cast(impedance::Dissipative<CartesianState>(parameter_list, impedance::ComputationalSpaceType::FULL));
    case CONTROLLER_TYPE::DISSIPATIVE_LINEAR:
      return py::cast(
          impedance::Dissipative<CartesianState>(parameter_list, impedance::ComputationalSpaceType::LINEAR));
    case CONTROLLER_TYPE::DISSIPATIVE_ANGULAR:
      return py::cast(
          impedance::Dissipative<CartesianState>(parameter_list, impedance::ComputationalSpaceType::ANGULAR));
    case CONTROLLER_TYPE::DISSIPATIVE_DECOUPLED:
      return py::cast(
          impedance::Dissipative<CartesianState>(parameter_list, impedance::ComputationalSpaceType::DECOUPLED_TWIST));
    case CONTROLLER_TYPE::COMPLIANT_TWIST:
      return py::cast(impedance::CompliantTwist(parameter_list));
    default:
    case CONTROLLER_TYPE::NONE:
      return py::cast(nullptr);
  }
}

py::object cartesian_controller_factory(
    CONTROLLER_TYPE type, std::list<std::shared_ptr<ParameterInterface>> parameter_list,
    const robot_model::Model& robot_model
) {
  switch (type) {
    case CONTROLLER_TYPE::IMPEDANCE: {
      auto ctrl = impedance::Impedance<CartesianState>(parameter_list);
      ctrl.set_robot_model(robot_model);
      return py::cast(ctrl);
    }
    case CONTROLLER_TYPE::VELOCITY_IMPEDANCE: {
      auto ctrl = impedance::VelocityImpedance<CartesianState>(parameter_list);
      ctrl.set_robot_model(robot_model);
      return py::cast(ctrl);
    }
    case CONTROLLER_TYPE::DISSIPATIVE: {
      auto ctrl = impedance::Dissipative<CartesianState>(parameter_list, impedance::ComputationalSpaceType::FULL);
      ctrl.set_robot_model(robot_model);
      return py::cast(ctrl);
    }
    case CONTROLLER_TYPE::DISSIPATIVE_LINEAR: {
      auto ctrl = impedance::Dissipative<CartesianState>(parameter_list, impedance::ComputationalSpaceType::LINEAR);
      ctrl.set_robot_model(robot_model);
      return py::cast(ctrl);
    }
    case CONTROLLER_TYPE::DISSIPATIVE_ANGULAR: {
      auto ctrl = impedance::Dissipative<CartesianState>(parameter_list, impedance::ComputationalSpaceType::ANGULAR);
      ctrl.set_robot_model(robot_model);
      return py::cast(ctrl);
    }
    case CONTROLLER_TYPE::DISSIPATIVE_DECOUPLED: {
      auto ctrl =
          impedance::Dissipative<CartesianState>(parameter_list, impedance::ComputationalSpaceType::DECOUPLED_TWIST);
      ctrl.set_robot_model(robot_model);
      return py::cast(ctrl);
    }
    case CONTROLLER_TYPE::COMPLIANT_TWIST: {
      auto ctrl = impedance::CompliantTwist(parameter_list);
      ctrl.set_robot_model(robot_model);
      return py::cast(ctrl);
    }
    default:
    case CONTROLLER_TYPE::NONE:
      throw controllers::exceptions::InvalidControllerException("Cannot assign robot model to this controller!");
  }
}

void cartesian_controller(py::module_& m) {
  py::class_<IController<CartesianState>, ParameterMap, PyController<CartesianState>> c(m, "ICartesianController");

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

void cartesian_impedance_controllers(py::module_& m) {
  using namespace impedance;

  py::class_<Impedance<CartesianState>, IController<CartesianState>>(m, "CartesianImpedanceController");
  py::class_<VelocityImpedance<CartesianState>, IController<CartesianState>>(m, "CartesianVelocityImpedanceController");
  py::class_<Dissipative<CartesianState>, IController<CartesianState>>(m, "CartesianDissipativeController");
  py::class_<CompliantTwist, IController<CartesianState>>(m, "CartesianCompliantTwistController");
}

void bind_cartesian_controllers(py::module_& m) {
  cartesian_controller(m);
  cartesian_impedance_controllers(m);

  m.def("create_cartesian_controller", [](CONTROLLER_TYPE type, const std::list<ParameterContainer>& parameters) -> py::object {
    return cartesian_controller_factory(type, container_to_interface_ptr_list(parameters));
  }, "Create a controller of the desired type with initial parameters.", "type"_a, "parameters"_a);

  m.def("create_cartesian_controller", [](CONTROLLER_TYPE type) -> py::object {
    return cartesian_controller_factory(type, std::list<std::shared_ptr<ParameterInterface>>());
  }, "Create a controller of the desired type.", "type"_a);

  m.def("create_cartesian_controller", [](CONTROLLER_TYPE type, const std::list<ParameterContainer>& parameters, const robot_model::Model& robot_model) -> py::object {
    return cartesian_controller_factory(type, container_to_interface_ptr_list(parameters), robot_model);
  }, "Create a controller of the desired type with initial parameters and an associated robot model.", "type"_a, "parameters"_a, "robot_model"_a);

  m.def("create_cartesian_controller", [](CONTROLLER_TYPE type, const robot_model::Model& robot_model) -> py::object {
    return cartesian_controller_factory(type, std::list<std::shared_ptr<ParameterInterface>>(), robot_model);
  }, "Create a controller of the desired type with an associated robot model.", "type"_a, "robot_model"_a);
}
