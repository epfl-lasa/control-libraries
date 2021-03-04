/**
 * @author Baptiste Busch
 * @date 2019/07/18
 *
 */

#pragma once

#include "state_representation/Parameters/ParameterInterface.hpp"
#include <list>
#include <memory>

namespace DynamicalSystems {
/**
 * @class DynamicalSystem
 * @brief Abstract class to define a DynamicalSystem either in joint or cartesian spaces
 */
template <class S>
class DynamicalSystem {
private:
  S reference_frame_;

protected:
  /**
   * @brief Compute the dynamics of the input state.
   * Internal function, to be redefined based on the
   * type of dynamical system, called by the evaluate
   * function
   * @param state the input state
   * @return the output state
   */
  virtual S compute_dynamics(const S& state) const = 0;

public:
  /**
   * @brief Empty constructor
   */
  explicit DynamicalSystem();

  /**
   * @brief Constructor with a provided reference frame
   * @param reference_frame the reference frame in which the dynamics is computed
   */
  explicit DynamicalSystem(const S& reference_frame);

  /**
   * @brief Evaluate the value of the dynamical system at a given state
   * @param state state at wich to perform the evaluation
   * @return the state (velocity) to move toward the attractor
   */
  S evaluate(const S& state) const;

  /**
   * @brief Return a list of all the parameters of the dynamical system
   * @return the list of parameters
   */
  virtual std::list<std::shared_ptr<state_representation::ParameterInterface>> get_parameters() const;

  const S& get_reference_frame() const;

  void set_reference_frame(const S& reference_frame);
};

template <class S>
DynamicalSystem<S>::DynamicalSystem(const S& reference_frame) : reference_frame_(reference_frame) {}

template <class S>
std::list<std::shared_ptr<state_representation::ParameterInterface>> DynamicalSystem<S>::get_parameters() const {
  std::list<std::shared_ptr<state_representation::ParameterInterface>> param_list;
  return param_list;
}

template <class S>
inline const S& DynamicalSystem<S>::get_reference_frame() const {
  return this->reference_frame_;
}

template <class S>
inline void DynamicalSystem<S>::set_reference_frame(const S& reference_frame) {
  this->reference_frame_ = reference_frame;
}
}// namespace DynamicalSystems
