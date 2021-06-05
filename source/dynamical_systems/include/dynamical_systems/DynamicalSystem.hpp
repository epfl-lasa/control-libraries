/**
 * @author Baptiste Busch
 * @date 2019/07/18
 *
 */

#pragma once

#include "state_representation/parameters/ParameterInterface.hpp"
#include <list>
#include <memory>

namespace dynamical_systems {
/**
 * @class DynamicalSystem
 * @brief Abstract class to define a DynamicalSystem either in joint or cartesian spaces
 */
template<class S>
class DynamicalSystem {
private:
  S base_frame_; ///< frame in which the dynamical system is expressed

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
   * @param base_frame the reference frame in which the dynamics is computed
   */
  explicit DynamicalSystem(const S& base_frame);

  /**
   * @brief Constructor with a provided reference frame name
   * @param base_frame the name of the reference frame in which the dynamics is computed
   */
  explicit DynamicalSystem(const std::string& base_frame);

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

  /**
   * @brief Return the base frame of the dynamical system
   * @return the base frame
   */
  const S& get_base_frame() const;

  /**
   * @brief Set the base frame of the dynamical system
   * @param base_frame the new base frame
   */
  virtual void set_base_frame(const S& base_frame);

  /**
   * @brief Check compatibility between a state and the dynamical system
   * @param state the state to check for compatibility
   * @return true if the state is compatible with the dynamical system
   */
  virtual bool is_compatible(const S& state) const;
};

template<class S>
DynamicalSystem<S>::DynamicalSystem(const S& base_frame) : base_frame_(base_frame) {}

template<class S>
std::list<std::shared_ptr<state_representation::ParameterInterface>> DynamicalSystem<S>::get_parameters() const {
  std::list<std::shared_ptr<state_representation::ParameterInterface>> param_list;
  return param_list;
}

template<class S>
inline const S& DynamicalSystem<S>::get_base_frame() const {
  return this->base_frame_;
}

template<class S>
inline void DynamicalSystem<S>::set_base_frame(const S& base_frame) {
  this->base_frame_ = base_frame;
}
}// namespace dynamical_systems
