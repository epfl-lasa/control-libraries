#pragma once

#include "IDynamicalSystem.hpp"

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
};

template<class S>
S DefaultDynamicalSystem<S>::compute_dynamics(const S&) const {
  return S();
}
}// namespace dynamical_systems
