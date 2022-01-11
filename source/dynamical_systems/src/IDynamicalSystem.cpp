#include "dynamical_systems/IDynamicalSystem.hpp"

#include "dynamical_systems/exceptions/EmptyBaseFrameException.hpp"
#include "dynamical_systems/exceptions/NotImplementedException.hpp"

#include "state_representation/exceptions/IncompatibleReferenceFramesException.hpp"
#include "state_representation/exceptions/IncompatibleStatesException.hpp"
#include "state_representation/robot/JointState.hpp"
#include "state_representation/space/cartesian/CartesianState.hpp"

using namespace state_representation;

namespace dynamical_systems {
template<class S>
bool IDynamicalSystem<S>::is_compatible(const S&) const {
  throw exceptions::NotImplementedException("is_compatible(state) not implemented for this type of state.");
}

template<>
bool IDynamicalSystem<CartesianState>::is_compatible(const CartesianState& state) const {
  return !(state.get_reference_frame() != this->get_base_frame().get_name()
      && state.get_reference_frame() != this->get_base_frame().get_reference_frame());
}

template<>
bool IDynamicalSystem<JointState>::is_compatible(const JointState&) const {
  return true;
}

template<class S>
S IDynamicalSystem<S>::evaluate(const S& state) const {
  return this->compute_dynamics(state);
}

template<>
CartesianState IDynamicalSystem<CartesianState>::evaluate(const CartesianState& state) const {
  if (this->get_base_frame().is_empty()) {
    throw exceptions::EmptyBaseFrameException("The base frame of the dynamical system is empty.");
  }
  if (state.get_reference_frame() != this->get_base_frame().get_name()) {
    if (state.get_reference_frame() != this->get_base_frame().get_reference_frame()) {
      throw state_representation::exceptions::IncompatibleReferenceFramesException(
          "The evaluated state " + state.get_name() + " in frame " + state.get_reference_frame()
              + " is incompatible with the base frame of the dynamical system " + this->get_base_frame().get_name()
              + " in frame " + this->get_base_frame().get_reference_frame() + "."
      );
    }
    CartesianState result = this->get_base_frame().inverse() * state;
    result = this->compute_dynamics(result);
    return this->get_base_frame() * result;
  } else {
    return this->compute_dynamics(state);
  }
}

template<>
JointState IDynamicalSystem<JointState>::evaluate(const JointState& state) const {
  if (!this->is_compatible(state)) {
    throw state_representation::exceptions::IncompatibleStatesException(
        "The attractor and the provided state are not compatible."
    );
  }
  return this->compute_dynamics(state);
}

}// namespace dynamical_systems
