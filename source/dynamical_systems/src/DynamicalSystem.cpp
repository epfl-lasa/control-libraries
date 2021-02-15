#include "dynamical_systems/DynamicalSystem.hpp"
#include "state_representation/Robot/JointState.hpp"
#include "state_representation/Space/Cartesian/CartesianPose.hpp"
#include "state_representation/Space/Cartesian/CartesianState.hpp"
#include "state_representation/Space/Cartesian/CartesianTwist.hpp"

using namespace StateRepresentation;

namespace DynamicalSystems {
template <class S>
DynamicalSystem<S>::DynamicalSystem() {}

template DynamicalSystem<JointState>::DynamicalSystem();

template <>
DynamicalSystem<CartesianState>::DynamicalSystem() : reference_frame_(CartesianPose::Identity("world")) {}

template <class S>
S DynamicalSystem<S>::evaluate(const S& state) const {
  return this->compute_dynamics(state);
}

template JointState DynamicalSystem<JointState>::evaluate(const JointState& state) const;

template <>
CartesianState DynamicalSystem<CartesianState>::evaluate(const CartesianState& state) const {
  if (this->get_reference_frame().get_name() != state.get_reference_frame()) {
    CartesianState result = this->get_reference_frame().inverse() * state;
    result = this->compute_dynamics(result);
    return this->get_reference_frame() * result;
  } else {
    return this->compute_dynamics(state);
  }
}
}// namespace DynamicalSystems