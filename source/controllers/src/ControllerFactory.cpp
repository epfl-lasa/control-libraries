#include "controllers/ControllerFactory.hpp"

#include "controllers/exceptions/InvalidControllerException.hpp"
#include "controllers/impedance/Impedance.hpp"
#include "controllers/impedance/VelocityImpedance.hpp"
#include "controllers/impedance/Dissipative.hpp"
#include "controllers/impedance/CompliantTwist.hpp"

using namespace state_representation;

namespace controllers {

template<>
std::shared_ptr<IController<CartesianState>> ControllerFactory<CartesianState>::create_controller(
    CONTROLLER_TYPE type, const std::list<std::shared_ptr<ParameterInterface>>& parameters, unsigned int
) {
  switch (type) {
    case CONTROLLER_TYPE::IMPEDANCE:
      return std::make_shared<impedance::Impedance<CartesianState>>(parameters);
    case CONTROLLER_TYPE::VELOCITY_IMPEDANCE:
      return std::make_shared<impedance::VelocityImpedance<CartesianState>>(parameters);
    case CONTROLLER_TYPE::DISSIPATIVE:
      return std::make_shared<impedance::Dissipative<CartesianState>>(
          parameters, impedance::ComputationalSpaceType::FULL);
    case CONTROLLER_TYPE::DISSIPATIVE_LINEAR:
      return std::make_shared<impedance::Dissipative<CartesianState>>(
          parameters, impedance::ComputationalSpaceType::LINEAR);
    case CONTROLLER_TYPE::DISSIPATIVE_ANGULAR:
      return std::make_shared<impedance::Dissipative<CartesianState>>(
          parameters, impedance::ComputationalSpaceType::ANGULAR);
    case CONTROLLER_TYPE::DISSIPATIVE_DECOUPLED:
      return std::make_shared<impedance::Dissipative<CartesianState>>(
          parameters, impedance::ComputationalSpaceType::DECOUPLED_TWIST);
    case CONTROLLER_TYPE::COMPLIANT_TWIST:
      return std::make_shared<impedance::CompliantTwist>(parameters);
    default:
    case CONTROLLER_TYPE::NONE:
      return nullptr;
  }
}

template<>
std::shared_ptr<IController<JointState>> ControllerFactory<JointState>::create_controller(
    CONTROLLER_TYPE type, const std::list<std::shared_ptr<ParameterInterface>>& parameters, unsigned int dimensions
) {
  switch (type) {
    case CONTROLLER_TYPE::IMPEDANCE:
      return std::make_shared<impedance::Impedance<JointState>>(parameters, dimensions);
    case CONTROLLER_TYPE::VELOCITY_IMPEDANCE:
      return std::make_shared<impedance::VelocityImpedance<JointState>>(parameters, dimensions);
    case CONTROLLER_TYPE::DISSIPATIVE:
      return std::make_shared<impedance::Dissipative<JointState>>(
          parameters, impedance::ComputationalSpaceType::FULL, dimensions);
    case CONTROLLER_TYPE::DISSIPATIVE_LINEAR:
    case CONTROLLER_TYPE::DISSIPATIVE_ANGULAR:
    case CONTROLLER_TYPE::DISSIPATIVE_DECOUPLED:
      throw exceptions::InvalidControllerException(
          "The JointState dissipative controller cannot be decoupled into linear and angular terms");
    case CONTROLLER_TYPE::COMPLIANT_TWIST:
      throw exceptions::InvalidControllerException(
          "The compliant twist controller only works in Cartesian space");
    default:
    case CONTROLLER_TYPE::NONE:
      return nullptr;
  }
}

template<>
std::shared_ptr<IController<CartesianState>> ControllerFactory<CartesianState>::create_controller(
    CONTROLLER_TYPE type, const std::list<std::shared_ptr<ParameterInterface>>& parameters,
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
    CONTROLLER_TYPE type, const std::list<std::shared_ptr<ParameterInterface>>& parameters,
    const robot_model::Model& robot_model
) {
  auto ctrl = ControllerFactory<JointState>::create_controller(type, parameters, robot_model.get_number_of_joints());
  if (ctrl == nullptr) {
    throw exceptions::InvalidControllerException("Cannot assign robot model to this controller!");
  }
  ctrl->set_robot_model(robot_model);
  return ctrl;
}

}// namespace controllers
