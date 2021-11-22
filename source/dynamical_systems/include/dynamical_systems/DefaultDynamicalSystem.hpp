#pragma once

#include "dynamical_systems/IDynamicalSystem.hpp"
#include "dynamical_systems/exceptions/InvalidParameterException.hpp"

namespace dynamical_systems {

/**
 * @class DefaultDynamicalSystem
 * @brief A default dynamical system that just returns an empty state.
 * @tparam S Underlying state type of the dynamical system
 */
template<class S>
class DefaultDynamicalSystem : public IDynamicalSystem<S> {
public:
  /**
   * @brief Compute the dynamics of the input state.
   * @param state The input state
   * @return The output state
   */
  [[nodiscard]] S compute_dynamics(const S& state) const override;
private:
  void validate_parameter(const std::shared_ptr<state_representation::ParameterInterface>& parameter) override;
};

template<class S>
void DefaultDynamicalSystem<S>::validate_parameter(const std::shared_ptr<state_representation::ParameterInterface>&) {
  throw exceptions::InvalidParameterException("No parameter to be set on this type of DS.");
}

template<class S>
S DefaultDynamicalSystem<S>::compute_dynamics(const S&) const {
  return S();
}
}// namespace dynamical_systems
