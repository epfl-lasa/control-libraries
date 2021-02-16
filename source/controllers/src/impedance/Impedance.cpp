#include "controllers/impedance/Impedance.hpp"
#include "state_representation/Robot/JointState.hpp"
#include "state_representation/Robot/JointTorques.hpp"
#include "state_representation/Space/Cartesian/CartesianState.hpp"
#include "state_representation/Space/Cartesian/CartesianWrench.hpp"

namespace controllers {
namespace impedance {

template <class S>
Impedance<S>::Impedance(const Eigen::MatrixXd& stiffness, const Eigen::MatrixXd& damping, const Eigen::MatrixXd& inertia) : stiffness_(std::make_shared<StateRepresentation::Parameter<Eigen::MatrixXd>>("stiffness", stiffness)),
                                                                                                                            damping_(std::make_shared<StateRepresentation::Parameter<Eigen::MatrixXd>>("damping", damping)),
                                                                                                                            inertia_(std::make_shared<StateRepresentation::Parameter<Eigen::MatrixXd>>("inertia", inertia)) {}

template Impedance<StateRepresentation::CartesianState>::Impedance(const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::MatrixXd&);
template Impedance<StateRepresentation::JointState>::Impedance(const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::MatrixXd&);

template <>
StateRepresentation::CartesianState Impedance<StateRepresentation::CartesianState>::compute_command(const StateRepresentation::CartesianState& desired_state,
                                                                                                    const StateRepresentation::CartesianState& feedback_state) const {
  StateRepresentation::CartesianState state_error = desired_state - feedback_state;
  // compute the wrench using the forlmula W = K * e_pos + D * e_vel + I * acc_desired
  StateRepresentation::CartesianWrench command(feedback_state.get_name(), feedback_state.get_reference_frame());

  // compute force
  command.set_force(this->get_stiffness().block<3, 3>(0, 0) * state_error.get_position()
                    + this->get_damping().block<3, 3>(0, 0) * state_error.get_linear_velocity()
                    + this->get_inertia().block<3, 3>(0, 0) * desired_state.get_linear_acceleration());
  // compute torque (orientation requires special care)
  command.set_torque(this->get_stiffness().block<3, 3>(3, 3) * state_error.get_orientation().vec()
                     + this->get_damping().block<3, 3>(3, 3) * state_error.get_angular_velocity()
                     + this->get_inertia().block<3, 3>(3, 3) * desired_state.get_angular_acceleration());
  return command;
}

template <>
StateRepresentation::JointState Impedance<StateRepresentation::JointState>::compute_command(const StateRepresentation::JointState& desired_state,
                                                                                            const StateRepresentation::JointState& feedback_state) const {
  StateRepresentation::JointState state_error = desired_state - feedback_state;
  // compute the wrench using the forlmula W = K * e_pos + D * e_vel + I * acc_desired
  StateRepresentation::JointTorques command(feedback_state.get_name(), feedback_state.get_names());
  // compute torques
  command.set_torques(this->get_stiffness() * state_error.get_positions() + this->get_damping() * state_error.get_velocities() + this->get_inertia() * desired_state.get_accelerations());
  return command;
}
}// namespace impedance
}// namespace controllers