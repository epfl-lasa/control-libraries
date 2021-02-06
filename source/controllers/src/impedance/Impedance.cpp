#include "controllers/impedance/Impedance.hpp"
#include "state_representation/Robot/JointState.hpp"
#include "state_representation/Robot/JointTorques.hpp"
#include "state_representation/Space/Cartesian/CartesianState.hpp"
#include "state_representation/Space/Cartesian/CartesianWrench.hpp"

namespace controllers {
namespace impedance {
template <>
const StateRepresentation::CartesianState Impedance<StateRepresentation::CartesianState>::compute_command(const StateRepresentation::CartesianState& desired_state,
                                                                                                          const StateRepresentation::CartesianState& feedback_state) const {
  StateRepresentation::CartesianState state_error = desired_state - feedback_state;
  // compute the wrench using the forlmula W = K * e_pos + D * e_vel + I * acc_desired
  StateRepresentation::CartesianWrench command;
  // compute force
  command.set_force(this->get_stiffness() * state_error.get_position() + this->get_damping() * state_error.get_linear_velocity() + this->get_inertia() * desired_state.get_linear_acceleration());
  // compute torque (orientation requires special care)
  command.set_torque(this->get_stiffness() * state_error.get_orientation().vec() + this->get_damping() * state_error.get_angular_velocity() + this->get_inertia() * desired_state.get_angular_acceleration());
  return command;
}
}// namespace impedance
}// namespace controllers