#include "controllers_bindings.h"

#include <controllers/ControllerFactory.hpp>
#include <controllers/IController.hpp>
#include <controllers/exceptions/InvalidControllerException.hpp>
#include <controllers/impedance/Dissipative.hpp>
#include <controllers/impedance/Impedance.hpp>
#include <controllers/impedance/VelocityImpedance.hpp>
#include <robot_model/Model.hpp>
#include <state_representation/space/Jacobian.hpp>
#include <state_representation/space/joint/JointPositions.hpp>
#include <state_representation/space/joint/JointState.hpp>

#include "py_controller.h"

using namespace state_representation;
using namespace py_parameter;

py::object joint_controller_factory(
    CONTROLLER_TYPE type, std::list<std::shared_ptr<ParameterInterface>> parameter_list, unsigned int dimensions = 6
) {
  switch (type) {
    case CONTROLLER_TYPE::IMPEDANCE:
      return py::cast(impedance::Impedance<JointState>(parameter_list, dimensions));
    case CONTROLLER_TYPE::VELOCITY_IMPEDANCE:
      return py::cast(impedance::VelocityImpedance<JointState>(parameter_list, dimensions));
    case CONTROLLER_TYPE::DISSIPATIVE:
      return py::cast(
          impedance::Dissipative<JointState>(parameter_list, impedance::ComputationalSpaceType::FULL, dimensions));
    case CONTROLLER_TYPE::DISSIPATIVE_LINEAR:
    case CONTROLLER_TYPE::DISSIPATIVE_ANGULAR:
    case CONTROLLER_TYPE::DISSIPATIVE_DECOUPLED:
      throw controllers::exceptions::InvalidControllerException(
          "The JointState dissipative controller cannot be decoupled into linear and angular terms."
      );
    case CONTROLLER_TYPE::COMPLIANT_TWIST:
      throw controllers::exceptions::InvalidControllerException(
          "The compliant twist controller only works in Cartesian space."
      );
    default:
    case CONTROLLER_TYPE::NONE:
      return py::cast(nullptr);
  }
}

py::object joint_controller_factory(
    CONTROLLER_TYPE type, std::list<std::shared_ptr<ParameterInterface>> parameter_list,
    const robot_model::Model& robot_model
) {
  switch (type) {
    case CONTROLLER_TYPE::IMPEDANCE: {
      auto ctrl = impedance::Impedance<JointState>(parameter_list, robot_model.get_number_of_joints());
      ctrl.set_robot_model(robot_model);
      return py::cast(ctrl);
    }
    case CONTROLLER_TYPE::VELOCITY_IMPEDANCE: {
      auto ctrl = impedance::VelocityImpedance<JointState>(parameter_list, robot_model.get_number_of_joints());
      ctrl.set_robot_model(robot_model);
      return py::cast(ctrl);
    }
    case CONTROLLER_TYPE::DISSIPATIVE: {
      auto ctrl = impedance::Dissipative<JointState>(
          parameter_list, impedance::ComputationalSpaceType::FULL, robot_model.get_number_of_joints());
      ctrl.set_robot_model(robot_model);
      return py::cast(ctrl);
    }
    case CONTROLLER_TYPE::DISSIPATIVE_LINEAR:
    case CONTROLLER_TYPE::DISSIPATIVE_ANGULAR:
    case CONTROLLER_TYPE::DISSIPATIVE_DECOUPLED:
      throw controllers::exceptions::InvalidControllerException(
          "The JointState dissipative controller cannot be decoupled into linear and angular terms."
      );
    case CONTROLLER_TYPE::COMPLIANT_TWIST:
      throw controllers::exceptions::InvalidControllerException(
          "The compliant twist controller only works in Cartesian space."
      );
    default:
    case CONTROLLER_TYPE::NONE:
      throw controllers::exceptions::InvalidControllerException("Cannot assign robot model to this controller!");
  }
}

void joint_controller(py::module_& m) {
  py::class_<IController<JointState>, ParameterMap, PyController<JointState>> c(m, "IJointController");

  c.def(
      "compute_command", py::overload_cast<const JointState&, const JointState&>(&IController<JointState>::compute_command),
      "Compute the command output based on the commanded state and a feedback state.", "command_state"_a, "feedback_state"_a);

  c.def("get_robot_model", &IController<JointState>::get_robot_model, "Get the robot model associated with the controller.");
  c.def("set_robot_model", &IController<JointState>::set_robot_model, "Set the robot model associated with the controller.", "robot_model"_a);
}

void joint_impedance_controllers(py::module_& m) {
  using namespace impedance;

  py::class_<Impedance<JointState>, IController<JointState>> (m, "JointImpedanceController");
  py::class_<VelocityImpedance<JointState>, IController<JointState>>(m, "JointVelocityImpedanceController");
  py::class_<Dissipative<JointState>, IController<JointState>>(m, "JointDissipativeController");
}

void bind_joint_controllers(py::module_& m) {
  joint_controller(m);
  joint_impedance_controllers(m);

  m.def("create_joint_controller", [](CONTROLLER_TYPE type, const std::list<ParameterContainer>& parameters, unsigned int dimensions = 6) -> py::object {
    return joint_controller_factory(type, container_to_interface_ptr_list(parameters), dimensions);
  }, "Create a controller of the desired type with initial parameters.", "type"_a, "parameters"_a, "dimensions"_a = int(6));

  m.def("create_joint_controller", [](CONTROLLER_TYPE type, unsigned int dimensions = 6) -> py::object {
    return joint_controller_factory(type, std::list<std::shared_ptr<ParameterInterface>>(), dimensions);
  }, "Create a controller of the desired type.", "type"_a, "dimensions"_a = int(6));

  m.def("create_joint_controller", [](CONTROLLER_TYPE type, const std::list<ParameterContainer>& parameters, const robot_model::Model& robot_model) -> py::object {
    return joint_controller_factory(type, container_to_interface_ptr_list(parameters), robot_model);
  }, "Create a controller of the desired type with initial parameters and an associated robot model.", "type"_a, "parameters"_a, "robot_model"_a);

  m.def("create_joint_controller", [](CONTROLLER_TYPE type, const robot_model::Model& robot_model) -> py::object {
    return joint_controller_factory(type, std::list<std::shared_ptr<ParameterInterface>>(), robot_model);
  }, "Create a controller of the desired type with an associated robot model.", "type"_a, "robot_model"_a);
}
