#include "controllers/IController.hpp"

#include "controllers/exceptions/NotImplementedException.hpp"

using namespace state_representation;

namespace controllers {

template<class S>
JointState IController<S>::compute_command(const S&, const S&, const Jacobian&) {
  throw exceptions::NotImplementedException(
      "Computation of a joint-space command with a Jacobian is not implemented for this controller.");
}

template<class S>
JointState IController<S>::compute_command(const S&, const S&, const JointPositions&, const std::string&) {
  throw exceptions::NotImplementedException(
      "Computation of a joint-space command from joint positions is not implemented for this controller.");
}

template<>
JointState IController<CartesianState>::compute_command(
    const CartesianState& command_state, const CartesianState& feedback_state,
    const Jacobian& jacobian
) {
  return jacobian.transpose() * CartesianWrench(this->compute_command(command_state, feedback_state));
}

template<>
JointState IController<CartesianState>::compute_command(
    const CartesianState& command_state, const CartesianState& feedback_state, const JointPositions& joint_positions,
    const std::string& frame_name
) {
  if (this->robot_model_ == nullptr) {
    throw exceptions::NoRobotModelException(
        "A robot model is required to convert the control command from Cartesian to joint space");
  }

  auto jacobian = this->robot_model_->compute_jacobian(joint_positions, frame_name);
  return this->compute_command(command_state, feedback_state, jacobian);
}

}