#pragma once

#include "dynamical_systems/IDynamicalSystem.hpp"

namespace dynamical_systems {

/**
 * @class DynamicalSystemFactory
 * @brief Factory to create a shared pointer to a dynamical system.
 * @tparam S Underlying state type of the dynamical system
 */
template<class S>
class DynamicalSystemFactory {
public:
  /**
   * @brief Enumeration of the implemented dynamical systems
   */
  enum DYNAMICAL_SYSTEM {
    NONE, CIRCULAR, POINT_ATTRACTOR, RING
  };

  /**
   * @brief Create a dynamical system of the desired type.
   * @param type Underlying state type of the dynamical system
   * @return The shared pointer to the dynamical system
   */
  static std::shared_ptr<IDynamicalSystem<S>> create_dynamical_system(DYNAMICAL_SYSTEM type);
};
}// namespace dynamical_systems
