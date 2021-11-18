#include "dynamical_systems/DynamicalSystemFactory.hpp"

#include "dynamical_systems/DefaultDynamicalSystem.hpp"
#include "state_representation/robot/JointState.hpp"

using namespace state_representation;

namespace dynamical_systems {

template<class S>
std::shared_ptr<IDynamicalSystem<S>>
DynamicalSystemFactory<S>::create_dynamical_system(DYNAMICAL_SYSTEM type) {
  switch (type) {
    default:
    case DYNAMICAL_SYSTEM::NONE:
      return std::make_shared<DefaultDynamicalSystem<S>>();
  }
}

template class DynamicalSystemFactory<CartesianState>;
template class DynamicalSystemFactory<JointState>;
}// namespace dynamical_systems
