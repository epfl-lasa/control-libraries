#include "dynamical_systems/DynamicalSystem.hpp"
#include "state_representation/robot/JointState.hpp"
#include "state_representation/space/cartesian/CartesianState.hpp"
#include "state_representation/exceptions/IncompatibleReferenceFramesException.hpp"

using namespace state_representation;

namespace DynamicalSystems {
template<class S>
DynamicalSystem<S>::DynamicalSystem() {}

template<>
DynamicalSystem<JointState>::DynamicalSystem() : base_frame_(JointState()) {}

template<>
DynamicalSystem<CartesianState>::DynamicalSystem() : base_frame_(CartesianState::Identity("world")) {}

template<>
DynamicalSystem<CartesianState>::DynamicalSystem(const std::string& base_frame) :
    base_frame_(CartesianState::Identity(base_frame, base_frame)) {}

template<class S>
S DynamicalSystem<S>::evaluate(const S& state) const {
  return this->compute_dynamics(state);
}

template JointState DynamicalSystem<JointState>::evaluate(const JointState& state) const;

template<>
CartesianState DynamicalSystem<CartesianState>::evaluate(const CartesianState& state) const {
  if (state.get_reference_frame() != this->get_base_frame().get_name()) {
    if (state.get_reference_frame() != this->get_base_frame().get_reference_frame()) {
      throw state_representation::exceptions::IncompatibleReferenceFramesException(
          "The evaluated state " + state.get_name() + " in frame " + state.get_reference_frame()
              + " is incompatible with the base frame of the dynamical system "
              + this->get_base_frame().get_name() + " in frame " + this->get_base_frame().get_reference_frame() + "."
      );
    }
    CartesianState result = this->get_base_frame().inverse() * state;
    result = this->compute_dynamics(result);
    return this->get_base_frame() * result;
  } else {
    return this->compute_dynamics(state);
  }
}
}// namespace DynamicalSystems