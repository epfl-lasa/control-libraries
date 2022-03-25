#include "controllers_bindings.h"

#include <controllers/ControllerFactory.hpp>
#include <controllers/IController.hpp>
#include <robot_model/Model.hpp>
#include <state_representation/space/joint/JointState.hpp>

#include "py_controller.h"

using namespace state_representation;
using namespace py_parameter;

void joint_controller(py::module_& m) {
  py::class_<IController<JointState>, std::shared_ptr<IController<JointState>>, ParameterMap, PyController<JointState>> c(m, "IJointController");

  c.def(
      "compute_command", py::overload_cast<const JointState&, const JointState&>(&IController<JointState>::compute_command),
      "Compute the command output based on the commanded state and a feedback state.", "command_state"_a, "feedback_state"_a);

  c.def("get_robot_model", &IController<JointState>::get_robot_model, "Get the robot model associated with the controller.");
  c.def("set_robot_model", &IController<JointState>::set_robot_model, "Set the robot model associated with the controller.", "robot_model"_a);
}

void bind_joint_controllers(py::module_& m) {
  joint_controller(m);

  m.def("create_joint_controller", [](CONTROLLER_TYPE type, const std::list<ParameterContainer>& parameters, unsigned int dimensions = 6) -> py::object {
    return py::cast(JointControllerFactory::create_controller(type, container_to_interface_ptr_list(parameters), dimensions));
  }, "Create a controller of the desired type with initial parameters.", "type"_a, "parameters"_a, "dimensions"_a = int(6));

  m.def("create_joint_controller", [](CONTROLLER_TYPE type, unsigned int dimensions = 6) -> py::object {
    return py::cast(JointControllerFactory::create_controller(type, std::list<std::shared_ptr<ParameterInterface>>(), dimensions));
  }, "Create a controller of the desired type.", "type"_a, "dimensions"_a = int(6));

  m.def("create_joint_controller", [](CONTROLLER_TYPE type, const std::list<ParameterContainer>& parameters, const robot_model::Model& robot_model) -> py::object {
    return py::cast(JointControllerFactory::create_controller(type, container_to_interface_ptr_list(parameters), robot_model));
  }, "Create a controller of the desired type with initial parameters and an associated robot model.", "type"_a, "parameters"_a, "robot_model"_a);

  m.def("create_joint_controller", [](CONTROLLER_TYPE type, const robot_model::Model& robot_model) -> py::object {
    return py::cast(JointControllerFactory::create_controller(type, std::list<std::shared_ptr<ParameterInterface>>(), robot_model));
  }, "Create a controller of the desired type with an associated robot model.", "type"_a, "robot_model"_a);
}
