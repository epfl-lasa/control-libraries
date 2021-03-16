#include "dynamical_systems/Linear.hpp"

namespace DynamicalSystems {
template <>
inline void Linear<state_representation::CartesianState>::set_gain(double iso_gain) {
  this->gain_->set_value(iso_gain * Eigen::MatrixXd::Identity(6, 6));
}

template <>
inline void Linear<state_representation::JointState>::set_gain(double iso_gain) {
  int nb_joints = this->get_attractor().get_size();
  this->gain_->set_value(iso_gain * Eigen::MatrixXd::Identity(nb_joints, nb_joints));
}

template <>
inline void Linear<state_representation::CartesianState>::set_gain(const std::vector<double>& diagonal_coefficients) {
  if (diagonal_coefficients.size() != 6) {
    throw Exceptions::IncompatibleSizeException("The provided diagonal coefficients do not correspond to the expected size of 6 elements");
  }
  Eigen::VectorXd diagonal = Eigen::VectorXd::Map(diagonal_coefficients.data(), 6);
  this->gain_->set_value(diagonal.asDiagonal());
}

template <>
inline void Linear<state_representation::JointState>::set_gain(const std::vector<double>& diagonal_coefficients) {
  size_t nb_joints = this->get_attractor().get_size();
  if (diagonal_coefficients.size() != nb_joints) {
    throw Exceptions::IncompatibleSizeException("The provided diagonal coefficients do not correspond to the expected size of " + std::to_string(nb_joints) + " elements");
  }
  Eigen::VectorXd diagonal = Eigen::VectorXd::Map(diagonal_coefficients.data(), nb_joints);
  this->gain_->set_value(diagonal.asDiagonal());
}

template <>
inline void Linear<state_representation::CartesianState>::set_gain(const Eigen::MatrixXd& gain_matrix) {
  if (gain_matrix.rows() != 6 && gain_matrix.cols() != 6) {
    throw Exceptions::IncompatibleSizeException("The provided gain matrix do not have the expected size of 6x6 elements");
  }
  this->gain_->set_value(gain_matrix);
}

template <>
inline void Linear<state_representation::JointState>::set_gain(const Eigen::MatrixXd& gain_matrix) {
  int nb_joints = this->get_attractor().get_size();
  if (gain_matrix.rows() != nb_joints && gain_matrix.cols() != nb_joints) {
    throw Exceptions::IncompatibleSizeException("The provided gain matrix do not have the expected size of " + std::to_string(nb_joints) + "x" + std::to_string(nb_joints) + " elements");
  }
  this->gain_->set_value(gain_matrix);
}

template <>
Linear<state_representation::CartesianState>::Linear(const state_representation::CartesianState& attractor, double iso_gain) : DynamicalSystem<state_representation::CartesianState>(state_representation::CartesianPose::Identity(attractor.get_reference_frame())),
                                                                                                                               attractor_(std::make_shared<state_representation::Parameter<state_representation::CartesianState>>(state_representation::Parameter<state_representation::CartesianPose>("attractor", attractor))),
                                                                                                                               gain_(std::make_shared<state_representation::Parameter<Eigen::MatrixXd>>("gain")) {
  this->set_gain(iso_gain);
}

template <>
Linear<state_representation::JointState>::Linear(const state_representation::JointState& attractor, double iso_gain) : DynamicalSystem<state_representation::JointState>(),
                                                                                                                       attractor_(std::make_shared<state_representation::Parameter<state_representation::JointState>>(state_representation::Parameter<state_representation::JointPositions>("attractor", attractor))),
                                                                                                                       gain_(std::make_shared<state_representation::Parameter<Eigen::MatrixXd>>("gain")) {
  this->set_gain(iso_gain);
}

template <>
Linear<state_representation::CartesianState>::Linear(const state_representation::CartesianState& attractor, const std::vector<double>& diagonal_coefficients) : DynamicalSystem<state_representation::CartesianState>(state_representation::CartesianPose::Identity(attractor.get_reference_frame())),
                                                                                                                                                                attractor_(std::make_shared<state_representation::Parameter<state_representation::CartesianState>>(state_representation::Parameter<state_representation::CartesianPose>("attractor", attractor))),
                                                                                                                                                                gain_(std::make_shared<state_representation::Parameter<Eigen::MatrixXd>>("gain")) {
  this->set_gain(diagonal_coefficients);
}

template <>
Linear<state_representation::JointState>::Linear(const state_representation::JointState& attractor, const std::vector<double>& diagonal_coefficients) : DynamicalSystem<state_representation::JointState>(),
                                                                                                                                                        attractor_(std::make_shared<state_representation::Parameter<state_representation::JointState>>(state_representation::Parameter<state_representation::JointPositions>("attractor", attractor))),
                                                                                                                                                        gain_(std::make_shared<state_representation::Parameter<Eigen::MatrixXd>>("gain")) {
  this->set_gain(diagonal_coefficients);
}

template <>
Linear<state_representation::CartesianState>::Linear(const state_representation::CartesianState& attractor, const Eigen::MatrixXd& gain_matrix) : DynamicalSystem<state_representation::CartesianState>(state_representation::CartesianPose::Identity(attractor.get_reference_frame())),
                                                                                                                                                  attractor_(std::make_shared<state_representation::Parameter<state_representation::CartesianState>>(state_representation::Parameter<state_representation::CartesianPose>("attractor", attractor))),
                                                                                                                                                  gain_(std::make_shared<state_representation::Parameter<Eigen::MatrixXd>>("gain")) {
  this->set_gain(gain_matrix);
}

template <>
Linear<state_representation::JointState>::Linear(const state_representation::JointState& attractor, const Eigen::MatrixXd& gain_matrix) : DynamicalSystem<state_representation::JointState>(),
                                                                                                                                          attractor_(std::make_shared<state_representation::Parameter<state_representation::JointState>>(state_representation::Parameter<state_representation::JointPositions>("attractor", attractor))),
                                                                                                                                          gain_(std::make_shared<state_representation::Parameter<Eigen::MatrixXd>>("gain")) {
  this->set_gain(gain_matrix);
}

template <>
state_representation::CartesianState Linear<state_representation::CartesianState>::compute_dynamics(const state_representation::CartesianState& state) const {
  state_representation::CartesianTwist twist = static_cast<const state_representation::CartesianPose&>(this->get_attractor()) - static_cast<const state_representation::CartesianPose&>(state);
  twist *= this->get_gain();
  return twist;
}

template <>
state_representation::JointState Linear<state_representation::JointState>::compute_dynamics(const state_representation::JointState& state) const {
  state_representation::JointVelocities velocities = static_cast<const state_representation::JointPositions&>(this->get_attractor()) - static_cast<const state_representation::JointPositions&>(state);
  velocities *= this->get_gain();
  return velocities;
}
}// namespace DynamicalSystems