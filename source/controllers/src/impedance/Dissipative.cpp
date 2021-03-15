#include "controllers/impedance/Dissipative.hpp"

using namespace state_representation;

namespace controllers::impedance {
template<class S>
Dissipative<S>::Dissipative(const ComputationalSpaceType& computational_space):
    Dissipative<S>(computational_space, 6) {}

template Dissipative<CartesianState>::Dissipative(const ComputationalSpaceType&);

template<class S>
Dissipative<S>::Dissipative(unsigned int nb_dimensions):
    Dissipative<S>(ComputationalSpaceType::FULL, nb_dimensions) {}

template Dissipative<JointState>::Dissipative(unsigned int);

template<class S>
S Dissipative<S>::compute_command(const S&, const S&) {
  throw exceptions::NotImplementedException(
      "compute_command(desired_state, feedback_state) not implemented for this input class");
  return S();
}

template<>
CartesianState Dissipative<CartesianState>::compute_command(const CartesianState& desired_state, const CartesianState& feedback_state) {
  // compute the damping matrix out of the desired_state twist
  this->compute_damping(desired_state.get_twist());
  // apply the impedance control law
  return this->Impedance<CartesianState>::compute_command(desired_state, feedback_state);
}

template<>
JointState Dissipative<JointState>::compute_command(const JointState& desired_state, const JointState& feedback_state) {
  // compute the damping matrix out of the desired_state twist
  this->compute_damping(desired_state.get_velocities());
  // apply the impedance control law
  return this->Impedance<JointState>::compute_command(desired_state, feedback_state);
}
}// namespace controllers