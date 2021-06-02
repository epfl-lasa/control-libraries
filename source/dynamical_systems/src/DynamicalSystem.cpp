#include "dynamical_systems/DynamicalSystem.hpp"
#include "state_representation/robot/JointState.hpp"
#include "state_representation/space/cartesian/CartesianState.hpp"
#include "state_representation/exceptions/IncompatibleReferenceFramesException.hpp"
#include "dynamical_systems/exceptions/NotImplementedException.hpp"
#include "dynamical_systems/exceptions/EmptyBaseFrameException.hpp"

using namespace state_representation;

namespace dynamical_systems {
template<class S>
DynamicalSystem<S>::DynamicalSystem() {}

template<>
DynamicalSystem<JointState>::DynamicalSystem() : base_frame_(JointState()) {}

template<>
DynamicalSystem<CartesianState>::DynamicalSystem() : base_frame_(CartesianState()) {}

template<>
DynamicalSystem<CartesianState>::DynamicalSystem(const std::string& base_frame) :
    base_frame_(CartesianState::Identity(base_frame, base_frame)) {}

template<class S>
bool DynamicalSystem<S>::is_compatible(const S&) {
  throw exceptions::NotImplementedException("is_compatible(state) not implemented for this type of state");
}

template<>
bool DynamicalSystem<state_representation::CartesianState>::is_compatible(const state_representation::CartesianState& state) {
  return !(state.get_reference_frame() != this->get_base_frame().get_name()
      && state.get_reference_frame() != this->get_base_frame().get_reference_frame());
}

template<class S>
S DynamicalSystem<S>::evaluate(const S& state) const {
  return this->compute_dynamics(state);
}

template JointState DynamicalSystem<JointState>::evaluate(const JointState& state) const;

template<>
CartesianState DynamicalSystem<CartesianState>::evaluate(const CartesianState& state) const {
  if (this->get_base_frame().is_empty()) {
    throw exceptions::EmptyBaseFrameException("The base frame of the dynamical system is empty.");
  }
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
}// namespace dynamical_systems