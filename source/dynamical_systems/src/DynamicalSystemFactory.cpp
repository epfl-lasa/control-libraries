#include "dynamical_systems/DynamicalSystemFactory.hpp"

#include "dynamical_systems/Circular.hpp"
#include "dynamical_systems/DefaultDynamicalSystem.hpp"
#include "dynamical_systems/PointAttractor.hpp"
#include "dynamical_systems/Ring.hpp"
#include "state_representation/space/joint/JointState.hpp"

using namespace state_representation;

namespace dynamical_systems {

template<>
std::shared_ptr<IDynamicalSystem<CartesianState>>
DynamicalSystemFactory<CartesianState>::create_dynamical_system(DYNAMICAL_SYSTEM type) {
  switch (type) {
    case DYNAMICAL_SYSTEM::POINT_ATTRACTOR:
      return std::make_shared<PointAttractor<CartesianState>>();
    case DYNAMICAL_SYSTEM::CIRCULAR:
      return std::make_shared<Circular>();
    case DYNAMICAL_SYSTEM::RING:
      return std::make_shared<Ring>();
    default:
    case DYNAMICAL_SYSTEM::NONE:
      return std::make_shared<DefaultDynamicalSystem<CartesianState>>();
  }
}

template<>
std::shared_ptr<IDynamicalSystem<JointState>>
DynamicalSystemFactory<JointState>::create_dynamical_system(DYNAMICAL_SYSTEM type) {
  switch (type) {
    case DYNAMICAL_SYSTEM::POINT_ATTRACTOR:
      return std::make_shared<PointAttractor<JointState>>();
    default:
    case DYNAMICAL_SYSTEM::NONE:
      return std::make_shared<DefaultDynamicalSystem<JointState>>();
  }
}
}// namespace dynamical_systems
