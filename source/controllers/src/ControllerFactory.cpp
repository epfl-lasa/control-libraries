#include "controllers/ControllerFactory.hpp"

#include "controllers/exceptions/InvalidControllerException.hpp"
#include "controllers/impedance/NewImpedance.hpp"
#include "controllers/impedance/NewVelocityImpedance.hpp"
#include "controllers/impedance/NewDissipative.hpp"
#include "controllers/impedance/CompliantTwist.hpp"

using namespace state_representation;

namespace controllers {

template<>
std::shared_ptr<IController<CartesianState>> ControllerFactory<CartesianState>::create_controller(
    CONTROLLER_TYPE type, const std::list<std::shared_ptr<state_representation::ParameterInterface>>& parameters,
    unsigned int
) {
  switch (type) {
    case CONTROLLER_TYPE::IMPEDANCE:
      return std::make_shared<impedance::NewImpedance<CartesianState>>(parameters);
    case CONTROLLER_TYPE::VELOCITY_IMPEDANCE:
      return std::make_shared<impedance::NewVelocityImpedance<CartesianState>>(parameters);
    case CONTROLLER_TYPE::DISSIPATIVE:
      return std::make_shared<impedance::NewDissipative<CartesianState>>(
          parameters, impedance::ComputationalSpaceType::FULL);
    case CONTROLLER_TYPE::DISSIPATIVE_LINEAR:
      return std::make_shared<impedance::NewDissipative<CartesianState>>(
          parameters, impedance::ComputationalSpaceType::LINEAR);
    case CONTROLLER_TYPE::DISSIPATIVE_ANGULAR:
      return std::make_shared<impedance::NewDissipative<CartesianState>>(
          parameters, impedance::ComputationalSpaceType::ANGULAR);
    case CONTROLLER_TYPE::DISSIPATIVE_DECOUPLED:
      return std::make_shared<impedance::NewDissipative<CartesianState>>(
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
    CONTROLLER_TYPE type, const std::list<std::shared_ptr<state_representation::ParameterInterface>>& parameters,
    unsigned int dimensions
) {
  switch (type) {
    case CONTROLLER_TYPE::IMPEDANCE:
      return std::make_shared<impedance::NewImpedance<JointState>>(parameters, dimensions);
    case CONTROLLER_TYPE::VELOCITY_IMPEDANCE:
      return std::make_shared<impedance::NewVelocityImpedance<JointState>>(parameters, dimensions);
    case CONTROLLER_TYPE::DISSIPATIVE:
      return std::make_shared<impedance::NewDissipative<JointState>>(
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
    CONTROLLER_TYPE type, const std::list<std::shared_ptr<state_representation::ParameterInterface>>& parameters,
    const robot_model::Model& robot_model
) {
  auto ctrl = ControllerFactory<CartesianState>::create_controller(type, parameters);
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
