#pragma once

#include "controllers/exceptions/NotImplementedException.hpp"
#include "state_representation/Parameters/ParameterInterface.hpp"
#include <list>
#include <memory>

namespace controllers {
/**
 * @class Controller
 * @brief Abstract class to define a controller either in joint or cartesian spaces
 * @tparam SIn the input state type of the controller
 * @tparam SOut the output command type of the controller
 */
template <class SIn, class SOut>
class Controller {
public:
  /**
   * @brief Empty constructor
   */
  explicit Controller();

  /**
   * @brief Compute the command based on the input state in a non const fashion
   * To be redefined based on the actual controller implementation
   * @param state the input state of the system. This function accept any number of extra arguments.
   * @return the output command at the input state
   */
  virtual SOut compute_command(const SIn& state, ...);

  /**
   * @brief Compute the command based on the desired state and a feedback state in a non const fashion
   * To be redefined based on the actual controller implementation.
   * @param desired_state the desired state of the system.
   * @param feedback_state the real state of the system as read from feedback loop
   * @return the output command at the input state
   */
  virtual SOut compute_command(const SIn& desired_state, const SIn& feedback_state);

  /**
   * @brief Return a list of all the parameters of the controller
   * @return the list of parameters
   */
  virtual std::list<std::shared_ptr<StateRepresentation::ParameterInterface>> get_parameters() const;
};

template <class SIn, class SOut>
Controller<SIn, SOut>::Controller() {}

template <class SIn, class SOut>
std::list<std::shared_ptr<StateRepresentation::ParameterInterface>> Controller<SIn, SOut>::get_parameters() const {
  std::list<std::shared_ptr<StateRepresentation::ParameterInterface>> param_list;
  return param_list;
}

template <class SIn, class SOut>
SOut Controller<SIn, SOut>::compute_command(const SIn&, ...) {
  throw exceptions::NotImplementedException("compute_command(state, ...) not implemented for the base controller class");
}

template <class SIn, class SOut>
SOut Controller<SIn, SOut>::compute_command(const SIn&, const SIn&) {
  throw exceptions::NotImplementedException("compute_command(desired_state, feedback_state) not implemented for the base controller class");
}
}// namespace controllers
