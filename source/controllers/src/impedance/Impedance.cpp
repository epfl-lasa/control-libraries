#include "controllers/impedance/Impedance.hpp"
#include "state_representation/robot/JointState.hpp"
#include "state_representation/robot/JointTorques.hpp"
#include "state_representation/Space/Cartesian/CartesianState.hpp"
#include "state_representation/Space/Cartesian/CartesianWrench.hpp"

using namespace state_representation;

namespace controllers::impedance {
template<class S>
Impedance<S>::Impedance(const Eigen::MatrixXd& stiffness,
                        const Eigen::MatrixXd& damping,
                        const Eigen::MatrixXd& inertia)
    : stiffness_(std::make_shared<Parameter<Eigen::MatrixXd>>("stiffness", stiffness)),
      damping_(std::make_shared<Parameter<Eigen::MatrixXd>>("damping", damping)),
      inertia_(std::make_shared<Parameter<Eigen::MatrixXd>>("inertia", inertia)) {}

template Impedance<CartesianState>::Impedance(const Eigen::MatrixXd&,
                                              const Eigen::MatrixXd&,
                                              const Eigen::MatrixXd&);
template Impedance<JointState>::Impedance(const Eigen::MatrixXd&,
                                          const Eigen::MatrixXd&,
                                          const Eigen::MatrixXd&);

template<class S>
S Impedance<S>::compute_command(const S&, const S&) {
  throw exceptions::NotImplementedException(
      "compute_command(desired_state, feedback_state) not implemented for this input class");
}

template<>
CartesianState Impedance<CartesianState>::compute_command(const CartesianState& desired_state,
                                                          const CartesianState& feedback_state) {
  CartesianState state_error = desired_state - feedback_state;
  // compute the wrench using the formula W = K * e_pos + D * e_vel + I * acc_desired
  CartesianWrench command(feedback_state.get_name(), feedback_state.get_reference_frame());
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

template<>
JointState Impedance<JointState>::compute_command(const JointState& desired_state,
                                                  const JointState& feedback_state) {
  JointState state_error = desired_state - feedback_state;
  // compute the wrench using the formula W = K * e_pos + D * e_vel + I * acc_desired
  JointTorques command(feedback_state.get_name(), feedback_state.get_names());
  // compute torques
  command.set_torques(this->get_stiffness() * state_error.get_positions()
                          + this->get_damping() * state_error.get_velocities()
                          + this->get_inertia() * desired_state.get_accelerations());
  return command;
}

template<class S>
JointState Impedance<S>::compute_command(const S&,
                                         const S&,
                                         const state_representation::Jacobian&) {
  throw exceptions::NotImplementedException(
      "compute_command(desired_state, feedback_state) not implemented for this input class");
  return JointState();
}

template<>
state_representation::JointState Impedance<CartesianState>::compute_command(const CartesianState& desired_state,
                                                                            const CartesianState& feedback_state,
                                                                            const Jacobian& jacobian) {
  CartesianWrench command = compute_command(desired_state, feedback_state);
  return jacobian.transpose() * command;
}
}// namespace controllers