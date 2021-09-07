#pragma once

#include "controllers/exceptions/NotImplementedException.hpp"
#include "state_representation/parameters/ParameterInterface.hpp"
#include "state_representation/robot/Jacobian.hpp"
#include <list>
#include <memory>

/**
 * @namespace controllers
 * @brief Systems to determine a command output from measured and desired inputs
 */
namespace controllers {
/**
 * @class Controller
 * @brief Abstract class to define a controller either in joint or cartesian spaces
 * @tparam SIn the input state type of the controller
 * @tparam SOut the output command type of the controller
 */
template<class S>
class Controller {
public:
  /**
   * @brief Empty constructor
   */
  explicit Controller();

  /**
   * @brief Compute the command based on the input state
   * To be redefined based on the actual controller implementation
   * @param state the input state of the system. This function accept any number of extra arguments.
   * @return the output command at the input state
   */
  virtual S compute_command(const S& state, ...);

  /**
   * @brief Compute the command based on the desired state and a feedback state
   * To be redefined based on the actual controller implementation.
   * @param desired_state the desired state of the system.
   * @param feedback_state the real state of the system as read from feedback loop
   * @return the output command at the input state
   */
  virtual S compute_command(const S& desired_state, const S& feedback_state);

  /**
   * @brief Compute the command based on the desired state and a feedback state
   * To be redefined based on the actual controller implementation.
   * @param desired_state the desired state of the system.
   * @param feedback_state the real state of the system as read from feedback loop
   * @param jacobian the Jacobian matrix of the robot to convert from one state to the other
   * @return the output command at the input state
   */
  virtual state_representation::JointState compute_command(const S& desired_state,
                                                           const S& feedback_state,
                                                           const state_representation::Jacobian& jacobian);

  /**
   * @brief Return a list of all the parameters of the controller
   * @return the list of parameters
   */
  virtual std::list<std::shared_ptr<state_representation::ParameterInterface>> get_parameters() const;
};

template<class S>
Controller<S>::Controller() {}

template<class S>
std::list<std::shared_ptr<state_representation::ParameterInterface>> Controller<S>::get_parameters() const {
  std::list<std::shared_ptr<state_representation::ParameterInterface>> param_list;
  return param_list;
}

template<class S>
S Controller<S>::compute_command(const S&, ...) {
  throw exceptions::NotImplementedException("compute_command(state, ...) not implemented for the base controller class");
}

template<class S>
S Controller<S>::compute_command(const S&, const S&) {
  throw exceptions::NotImplementedException(
      "compute_command(desired_state, feedback_state) not implemented for the base controller class");
}

template<class S>
state_representation::JointState Controller<S>::compute_command(const S&,
                                                                const S&,
                                                                const state_representation::Jacobian&) {
  throw exceptions::NotImplementedException(
      "compute_command(desired_state, feedback_state) not implemented for the base controller class");
}
}// namespace controllers
