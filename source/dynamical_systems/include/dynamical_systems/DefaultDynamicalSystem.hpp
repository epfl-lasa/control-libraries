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
   * @copydoc IDynamicalSystem::compute_dynamics
   */
  [[nodiscard]] S compute_dynamics(const S& state) const override;
private:
  /**
   * @copydoc IDynamicalSystem::validate_and_set_parameter
   */
  void validate_and_set_parameter(const std::shared_ptr<state_representation::ParameterInterface>& parameter) override;
};

template<class S>
void DefaultDynamicalSystem<S>::validate_and_set_parameter(const std::shared_ptr<state_representation::ParameterInterface>&) {
  throw exceptions::InvalidParameterException("No parameter to be set on this type of DS.");
}

template<class S>
S DefaultDynamicalSystem<S>::compute_dynamics(const S&) const {
  return S();
}
}// namespace dynamical_systems
