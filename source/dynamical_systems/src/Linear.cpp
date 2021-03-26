#include "dynamical_systems/Linear.hpp"

using namespace state_representation;

namespace dynamical_systems {

template<>
void Linear<CartesianState>::set_gain(double iso_gain) {
  this->gain_->set_value(iso_gain * Eigen::MatrixXd::Identity(6, 6));
}

template<>
void Linear<JointState>::set_gain(double iso_gain) {
  int nb_joints = this->get_attractor().get_size();
  this->gain_->set_value(iso_gain * Eigen::MatrixXd::Identity(nb_joints, nb_joints));
}

template<>
void Linear<CartesianState>::set_gain(const std::vector<double>& diagonal_coefficients) {
  if (diagonal_coefficients.size() != 6) {
    throw exceptions::IncompatibleSizeException(
        "The provided diagonal coefficients do not correspond to the expected size of 6 elements");
  }
  Eigen::VectorXd diagonal = Eigen::VectorXd::Map(diagonal_coefficients.data(), 6);
  this->gain_->set_value(diagonal.asDiagonal());
}

template<>
void Linear<JointState>::set_gain(const std::vector<double>& diagonal_coefficients) {
  size_t nb_joints = this->get_attractor().get_size();
  if (diagonal_coefficients.size() != nb_joints) {
    throw exceptions::IncompatibleSizeException(
        "The provided diagonal coefficients do not correspond to the expected size of "
            + std::to_string(nb_joints) + " elements");
  }
  Eigen::VectorXd diagonal = Eigen::VectorXd::Map(diagonal_coefficients.data(), nb_joints);
  this->gain_->set_value(diagonal.asDiagonal());
}

template<>
void Linear<CartesianState>::set_gain(const Eigen::MatrixXd& gain_matrix) {
  if (gain_matrix.rows() != 6 && gain_matrix.cols() != 6) {
    throw exceptions::IncompatibleSizeException("The provided gain matrix do not have the expected size of 6x6 elements");
  }
  this->gain_->set_value(gain_matrix);
}

template<>
void Linear<JointState>::set_gain(const Eigen::MatrixXd& gain_matrix) {
  int nb_joints = this->get_attractor().get_size();
  if (gain_matrix.rows() != nb_joints && gain_matrix.cols() != nb_joints) {
    throw exceptions::IncompatibleSizeException(
        "The provided gain matrix do not have the expected size of " + std::to_string(nb_joints) + "x"
            + std::to_string(nb_joints) + " elements");
  }
  this->gain_->set_value(gain_matrix);
}

template<>
Linear<CartesianState>::Linear(const CartesianState& attractor, double iso_gain) :
    DynamicalSystem<CartesianState>(attractor.get_reference_frame()),
    attractor_(std::make_shared<Parameter<CartesianState>>(Parameter<CartesianPose>("attractor", attractor))),
    gain_(std::make_shared<Parameter<Eigen::MatrixXd>>("gain")) {
  this->set_gain(iso_gain);
}

template<>
Linear<JointState>::Linear(const JointState& attractor, double iso_gain) :
    DynamicalSystem<JointState>(),
    attractor_(std::make_shared<Parameter<JointState>>(Parameter<JointPositions>("attractor", attractor))),
    gain_(std::make_shared<Parameter<Eigen::MatrixXd>>("gain")) {
  this->set_gain(iso_gain);
}

template<>
Linear<CartesianState>::Linear(const CartesianState& attractor, const std::vector<double>& diagonal_coefficients) :
    DynamicalSystem<CartesianState>(attractor.get_reference_frame()),
    attractor_(std::make_shared<Parameter<CartesianState>>(Parameter<CartesianPose>("attractor", attractor))),
    gain_(std::make_shared<Parameter<Eigen::MatrixXd>>("gain")) {
  this->set_gain(diagonal_coefficients);
}

template<>
Linear<JointState>::Linear(const JointState& attractor, const std::vector<double>& diagonal_coefficients) :
    DynamicalSystem<JointState>(),
    attractor_(std::make_shared<Parameter<JointState>>(Parameter<JointPositions>("attractor", attractor))),
    gain_(std::make_shared<Parameter<Eigen::MatrixXd>>("gain")) {
  this->set_gain(diagonal_coefficients);
}

template<>
Linear<CartesianState>::Linear(const CartesianState& attractor, const Eigen::MatrixXd& gain_matrix) :
    DynamicalSystem<CartesianState>(attractor.get_reference_frame()),
    attractor_(std::make_shared<Parameter<CartesianState>>(Parameter<CartesianPose>("attractor", attractor))),
    gain_(std::make_shared<Parameter<Eigen::MatrixXd>>("gain")) {
  this->set_gain(gain_matrix);
}

template<>
Linear<JointState>::Linear(const JointState& attractor, const Eigen::MatrixXd& gain_matrix)
    : DynamicalSystem<JointState>(),
      attractor_(std::make_shared<Parameter<JointState>>(Parameter<JointPositions>("attractor", attractor))),
      gain_(std::make_shared<Parameter<Eigen::MatrixXd>>("gain")) {
  this->set_gain(gain_matrix);
}

template<>
CartesianState Linear<CartesianState>::compute_dynamics(const CartesianState& state) const {
  CartesianTwist twist = CartesianPose(this->get_attractor()) - CartesianPose(state);
  twist *= this->get_gain();
  return twist;
}

template<>
JointState Linear<JointState>::compute_dynamics(const JointState& state) const {
  JointVelocities velocities = JointPositions(this->get_attractor()) - JointPositions(state);
  velocities *= this->get_gain();
  return velocities;
}
}// namespace dynamical_systems