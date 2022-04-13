#include "dynamical_systems/DynamicalSystemFactory.hpp"

#include "dynamical_systems/Circular.hpp"
#include "dynamical_systems/DefaultDynamicalSystem.hpp"
#include "dynamical_systems/PointAttractor.hpp"
#include "dynamical_systems/Ring.hpp"
#include "dynamical_systems/exceptions/InvalidDynamicalSystemException.hpp"
#include "state_representation/space/joint/JointState.hpp"

using namespace state_representation;

namespace dynamical_systems {

template<>
std::shared_ptr<IDynamicalSystem<CartesianState>> DynamicalSystemFactory<CartesianState>::create_dynamical_system(
    DYNAMICAL_SYSTEM_TYPE type, const std::list<std::shared_ptr<state_representation::ParameterInterface>>& parameters
) {
  switch (type) {
    case DYNAMICAL_SYSTEM_TYPE::POINT_ATTRACTOR:
      return std::make_shared<PointAttractor<CartesianState>>(parameters);
    case DYNAMICAL_SYSTEM_TYPE::CIRCULAR:
      return std::make_shared<Circular>(parameters);
    case DYNAMICAL_SYSTEM_TYPE::RING:
      return std::make_shared<Ring>(parameters);
    default:
    case DYNAMICAL_SYSTEM_TYPE::NONE:
      return std::make_shared<DefaultDynamicalSystem<CartesianState>>();
  }
}

template<>
std::shared_ptr<IDynamicalSystem<JointState>> DynamicalSystemFactory<JointState>::create_dynamical_system(
    DYNAMICAL_SYSTEM_TYPE type, const std::list<std::shared_ptr<state_representation::ParameterInterface>>& parameters
) {
  switch (type) {
    case DYNAMICAL_SYSTEM_TYPE::POINT_ATTRACTOR:
      return std::make_shared<PointAttractor<JointState>>(parameters);
    case DYNAMICAL_SYSTEM_TYPE::CIRCULAR:
    case DYNAMICAL_SYSTEM_TYPE::RING:
      throw exceptions::InvalidDynamicalSystemException("This JointState DS is not valid");
    default:
    case DYNAMICAL_SYSTEM_TYPE::NONE:
      return std::make_shared<DefaultDynamicalSystem<JointState>>();
  }
}
}// namespace dynamical_systems
