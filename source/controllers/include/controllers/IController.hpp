#pragma once

#include <list>
#include <map>
#include <memory>

#include "controllers/exceptions/NoRobotModelException.hpp"

#include "robot_model/Model.hpp"
#include "state_representation/parameters/ParameterMap.hpp"
#include "state_representation/space/Jacobian.hpp"
#include "state_representation/space/joint/JointPositions.hpp"
#include "state_representation/space/joint/JointState.hpp"

/**
 * @namespace controllers
 * @brief Systems to determine a command output from measured and desired inputs
 */
namespace controllers {
/**
 * @class IController
 * @brief Abstract class to define a controller in a desired state type, such as joint or Cartesian spaces
 * @tparam S The state type of the controller
 */
template<class S>
class IController : public state_representation::ParameterMap {
public:
  /**
   * @brief Empty constructor
   */
  IController() = default;

  /**
   * @brief Empty destructor
   */
  virtual ~IController() = default;

  /**
   * @brief Compute the command output based on the commanded state and a feedback state
   * To be redefined based on the actual controller implementation.
   * @param command_state The input state to the controller
   * @param feedback_state The current state of the system given as feedback
   * @return The output command in the same state space as the input
   */
  [[nodiscard]] virtual S compute_command(const S& command_state, const S& feedback_state) = 0;

  /**
   * @brief Compute the command output in joint space
   * @param command_state The input state to the controller
   * @param feedback_state The current state of the system given as feedback
   * @param jacobian The Jacobian matrix relating Cartesian forces to joint space
   * @return The output command in joint space
   */
  [[nodiscard]] state_representation::JointState compute_command(
      const S& command_state, const S& feedback_state, const state_representation::Jacobian& jacobian
  );

  /**
   * @brief Compute the command output in joint space from command and feedback states in task space
   * @details This method requires the controller to have an associated robot model to compute the Jacobian.
   * @param command_state The input state to the controller
   * @param feedback_state The current state of the system given as feedback
   * @param joint_positions The current joint positions, which are required for computing the Jacobian
   * @param frame The name of the robot frame at which to compute the Jacobian
   * @return The output command in joint space
   */
  [[nodiscard]] state_representation::JointState compute_command(
      const S& command_state, const S& feedback_state, const state_representation::JointPositions& joint_positions,
      const std::string& frame = ""
  );

  /**
   * @brief Get the robot model associated with the controller.
   * @return The robot model
   */
  [[nodiscard]] const robot_model::Model& get_robot_model();

  /**
   * @brief Set the robot model associated with the controller.
   * @param robot_model The robot model
   */
  void set_robot_model(const robot_model::Model& robot_model);

protected:
  std::shared_ptr<robot_model::Model> robot_model_; ///< The robot model associated with the controller
};

template<class S>
const robot_model::Model& IController<S>::get_robot_model() {
  if (this->robot_model_ == nullptr) {
    throw exceptions::NoRobotModelException("No robot model");
  }
  return *this->robot_model_;
}

template<class S>
void IController<S>::set_robot_model(const robot_model::Model& robot_model) {
  this->robot_model_ = std::make_shared<robot_model::Model>(robot_model);
}
}// namespace controllers
