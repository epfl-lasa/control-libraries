#pragma once

#include "dynamical_systems/DynamicalSystemType.hpp"
#include "dynamical_systems/IDynamicalSystem.hpp"

#include "state_representation/space/cartesian/CartesianState.hpp"
#include "state_representation/space/joint/JointState.hpp"

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
   * @brief Create a dynamical system of the desired type.
   * @param type The type of dynamical system
   * @return The shared pointer to the dynamical system
   */
  static std::shared_ptr<IDynamicalSystem<S>> create_dynamical_system(DYNAMICAL_SYSTEM_TYPE type);

  /**
 * @brief Create a dynamical system of the desired type with initial parameters.
 * @param type The type of dynamical system
 * @param parameters A list of parameters to set on the dynamical system
 * @return The shared pointer to the dynamical system
 */
  static std::shared_ptr<IDynamicalSystem<S>> create_dynamical_system(
      DYNAMICAL_SYSTEM_TYPE type, const std::list<std::shared_ptr<state_representation::ParameterInterface>>& parameters
  );
};

template<class S>
std::shared_ptr<IDynamicalSystem<S>> DynamicalSystemFactory<S>::create_dynamical_system(DYNAMICAL_SYSTEM_TYPE type) {
  std::list<std::shared_ptr<state_representation::ParameterInterface>> parameters;
  return DynamicalSystemFactory<S>::create_dynamical_system(type, parameters);
}

typedef DynamicalSystemFactory<state_representation::CartesianState> CartesianDynamicalSystemFactory;
typedef DynamicalSystemFactory<state_representation::JointState> JointDynamicalSystemFactory;

}// namespace dynamical_systems
