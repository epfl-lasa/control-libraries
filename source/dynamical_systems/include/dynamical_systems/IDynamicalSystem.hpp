#pragma once

#include <list>
#include <map>
#include <memory>

#include "state_representation/parameters/ParameterMap.hpp"

/**
 * @namespace dynamical_systems
 * @brief Systems of equations relating state variables to their derivatives
 */
namespace dynamical_systems {

/**
 * @class IDynamicalSystem
 * @brief Abstract class for a dynamical system.
 * @tparam S Underlying state type of the dynamical system
 */
template<class S>
class IDynamicalSystem : public state_representation::ParameterMap {
public:
  /**
   * @brief Empty constructor
   */
  IDynamicalSystem() = default;

  /**
   * @brief Check compatibility between a state and the dynamical system.
   * @param state The state to check for compatibility
   * @return True if the state is compatible with the dynamical system
   */
  [[nodiscard]] virtual bool is_compatible(const S& state) const;

  /**
   * @brief Evaluate the value of the dynamical system at a given state.
   * @param state State at which to perform the evaluation
   * @return The resulting state (velocity) of the dynamical system
   */
  [[nodiscard]] S evaluate(const S& state) const;

  /**
   * @brief Return the base frame of the dynamical system.
   * @return The base frame
   */
  [[nodiscard]] S get_base_frame() const;

  /**
   * @brief Set the base frame of the dynamical system.
   * @param base_frame The new base frame
   */
  virtual void set_base_frame(const S& base_frame);

protected:
  /**
   * @brief Compute the dynamics of the input state. Internal function,
   * to be redefined based on the type of dynamical system, called
   * by the evaluate function.
   * @param state The input state
   * @return The output state
   */
  [[nodiscard]] virtual S compute_dynamics(const S& state) const = 0;

private:
  S base_frame_; ///< frame in which the dynamical system is expressed
};

template<class S>
S IDynamicalSystem<S>::get_base_frame() const {
  return this->base_frame_;
}

template<class S>
void IDynamicalSystem<S>::set_base_frame(const S& base_frame) {
  this->base_frame_ = base_frame;
}

}// namespace dynamical_systems
