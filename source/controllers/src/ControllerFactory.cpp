#include "controllers/ControllerFactory.hpp"

#include "controllers/exceptions/InvalidControllerException.hpp"

using namespace state_representation;

namespace controllers {

template<>
std::shared_ptr<IController<CartesianState>> ControllerFactory<CartesianState>::create_controller(
    CONTROLLER_TYPE type, const std::list<std::shared_ptr<state_representation::ParameterInterface>>&,
    unsigned int
) {
  switch (type) {
    default:
    case CONTROLLER_TYPE::NONE:
      return nullptr;
  }
}

template<>
std::shared_ptr<IController<JointState>> ControllerFactory<JointState>::create_controller(
    CONTROLLER_TYPE type, const std::list<std::shared_ptr<state_representation::ParameterInterface>>&,
    unsigned int
) {
  switch (type) {
    default:
    case CONTROLLER_TYPE::NONE:
      return nullptr;
  }
}

template<>
std::shared_ptr<IController<CartesianState>> ControllerFactory<CartesianState>::create_controller(
    CONTROLLER_TYPE type, const std::list<std::shared_ptr<state_representation::ParameterInterface>>& parameters,
    const robot_model::Model& robot_model
) {
  auto ctrl = ControllerFactory<CartesianState>::create_controller(type, parameters);
  if (ctrl == nullptr) {
    throw exceptions::InvalidControllerException("Cannot assign robot model to this controller!");
  }
  ctrl->set_robot_model(robot_model);
  return ctrl;
}

template<>
std::shared_ptr<IController<JointState>> ControllerFactory<JointState>::create_controller(
    CONTROLLER_TYPE type, const std::list<std::shared_ptr<state_representation::ParameterInterface>>& parameters,
    const robot_model::Model& robot_model
) {
  auto ctrl = ControllerFactory<JointState>::create_controller(type, parameters, robot_model.get_number_of_joints());
  ctrl->set_robot_model(robot_model);
  return ctrl;
}

}// namespace controllers
