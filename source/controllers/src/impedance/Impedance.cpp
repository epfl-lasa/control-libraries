#include "controllers/impedance/Impedance.hpp"

#include "controllers/exceptions/NotImplementedException.hpp"
#include "state_representation/space/joint/JointState.hpp"
#include "state_representation/space/cartesian/CartesianState.hpp"

using namespace state_representation;

namespace controllers::impedance {

template<class S>
S Impedance<S>::compute_command(const S&, const S&) {
  throw exceptions::NotImplementedException("compute_command is not implemented for this state variable");
}

template<>
CartesianState Impedance<CartesianState>::compute_command(
    const CartesianState& command_state, const CartesianState& feedback_state
) {
  CartesianState state_error = command_state - feedback_state;
  // compute the wrench using the formula W = I * acc_desired + K * e_pose + D * e_twist + e_wrench
  CartesianState command(feedback_state.get_name(), feedback_state.get_reference_frame());
  // compute force
  Eigen::Vector3d position_control = this->stiffness_->get_value().topLeftCorner<3, 3>() * state_error.get_position()
      + this->damping_->get_value().topLeftCorner<3, 3>() * state_error.get_linear_velocity()
      + this->inertia_->get_value().topLeftCorner<3, 3>() * command_state.get_linear_acceleration();
  Eigen::Vector3d commanded_force = position_control + state_error.get_force();
  command.set_force(commanded_force);
  // compute torque (orientation requires special care)
  Eigen::Vector3d orientation_control =
      this->stiffness_->get_value().bottomRightCorner<3, 3>() * state_error.get_orientation().vec()
          + this->damping_->get_value().bottomRightCorner<3, 3>() * state_error.get_angular_velocity()
          + this->inertia_->get_value().bottomRightCorner<3, 3>() * command_state.get_angular_acceleration();
  Eigen::Vector3d commanded_torque = orientation_control + state_error.get_torque();
  command.set_torque(commanded_torque);
  return command;
}

template<>
JointState Impedance<JointState>::compute_command(
    const JointState& command_state, const JointState& feedback_state
) {
  JointState state_error = command_state - feedback_state;
  // compute the wrench using the formula T = I * acc_desired + K * e_pos + D * e_vel + e_tor
  JointState command(feedback_state.get_name(), feedback_state.get_names());
  // compute torques
  Eigen::VectorXd state_control = this->stiffness_->get_value() * state_error.get_positions()
      + this->damping_->get_value() * state_error.get_velocities()
      + this->inertia_->get_value() * command_state.get_accelerations();
  Eigen::VectorXd commanded_torques = state_control + state_error.get_torques();
  command.set_torques(commanded_torques);
  return command;
}

}// namespace controllers