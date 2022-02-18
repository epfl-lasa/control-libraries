#pragma once

#include "controllers/IController.hpp"
#include "controllers/ControllerType.hpp"
#include "robot_model/Model.hpp"

#include "state_representation/space/cartesian/CartesianState.hpp"
#include "state_representation/space/joint/JointState.hpp"

namespace controllers {

/**
 * @class ControllerFactory
 * @brief Factory to create a shared pointer to a controller.
 * @tparam S Underlying state type of the controller
 */
template<class S>
class ControllerFactory {
public:

  /**
   * @brief Create a controller of the desired type.
   * @param type The type of controller
   * @param dimensions The dimensionality of the controller (6 for Cartesian space, number of joints for joint space)
   * @return The shared pointer to the controller
   */
  static std::shared_ptr<IController<S>> create_controller(CONTROLLER_TYPE type, unsigned int dimensions = 6);

  /**
   * @brief Create a controller of the desired type with initial parameters
   * @param type The type of controller
   * @param parameters A list of parameters to set on the controller
   * @param dimensions The dimensionality of the controller (6 for Cartesian space, number of joints for joint space)
   * @return The shared pointer to the controller
   */
  static std::shared_ptr<IController<S>> create_controller(
      CONTROLLER_TYPE type, const std::list<std::shared_ptr<state_representation::ParameterInterface>>& parameters,
      unsigned int dimensions = 6
  );

  /**
   * @brief Create a controller of the desired type with an associated robot model.
   * @param type The type of controller
   * @param robot_model The robot model to associate with the controller. This is used to infer the dimensionality
   * of the controller and influence controller behaviour depending on the controller type.
   * @return The shared pointer to the controller
   */
  static std::shared_ptr<IController<S>> create_controller(
      CONTROLLER_TYPE type, const robot_model::Model& robot_model
  );

  /**
   * @brief Create a controller of the desired type with initial parameters and an associated robot model.
   * @param type The type of controller
   * @param parameters A list of parameters to set on the controller
   * @param robot_model The robot model to associate with the controller
   * @return The shared pointer to the controller
   */
  static std::shared_ptr<IController<S>> create_controller(
      CONTROLLER_TYPE type, const std::list<std::shared_ptr<state_representation::ParameterInterface>>& parameters,
      const robot_model::Model& robot_model
  );
};

template<class S>
std::shared_ptr<IController<S>> ControllerFactory<S>::create_controller(CONTROLLER_TYPE type, unsigned int dimensions) {
  std::list<std::shared_ptr<state_representation::ParameterInterface>> parameters;
  return ControllerFactory<S>::create_controller(type, parameters, dimensions);
}

template<class S>
std::shared_ptr<IController<S>>
ControllerFactory<S>::create_controller(CONTROLLER_TYPE type, const robot_model::Model& robot_model) {
  std::list<std::shared_ptr<state_representation::ParameterInterface>> parameters;
  return ControllerFactory<S>::create_controller(type, parameters, robot_model);
}

typedef ControllerFactory<state_representation::CartesianState> CartesianControllerFactory;
typedef ControllerFactory<state_representation::JointState> JointControllerFactory;

}// namespace controllers
