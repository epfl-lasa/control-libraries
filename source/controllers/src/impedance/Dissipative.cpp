#include "controllers/impedance/Dissipative.hpp"

namespace controllers {
namespace impedance {
Dissipative::Dissipative(const ComputationalSpaceType& computational_space) : Impedance<StateRepresentation::CartesianState>(Eigen::MatrixXd::Identity(6, 6),
                                                                                                                             Eigen::MatrixXd::Zero(6, 6),
                                                                                                                             Eigen::MatrixXd::Zero(6, 6)),
                                                                              computational_space_(computational_space),
                                                                              basis_(Eigen::MatrixXd::Random(6, 6)),
                                                                              damping_eigenvalues_(std::make_shared<StateRepresentation::Parameter<Eigen::VectorXd>>("damping_eigenvalues", Eigen::ArrayXd::Ones(6))) {}

Eigen::MatrixXd Dissipative::compute_orthonormal_basis(const Eigen::MatrixXd& basis, const Eigen::VectorXd& main_eigenvector) const {
  Eigen::MatrixXd orthonormal_basis = basis;
  orthonormal_basis.col(0) = main_eigenvector;
  // orthonormalization using Gram-Schmidt algorithm
  orthonormal_basis = orthonormal_basis.householderQr().householderQ();
  return orthonormal_basis;
}

void Dissipative::compute_damping(const Eigen::VectorXd& desired_twist) {
  switch (this->computational_space_) {
    case ComputationalSpaceType::LINEAR:
      //return only the linear block
      this->basis_.block<3, 3>(0, 0) = this->compute_orthonormal_basis(this->basis_.block<3, 3>(0, 0), desired_twist.segment(0, 3));
      this->basis_.block<3, 3>(3, 3).setZero();
      this->basis_.block<3, 3>(3, 0).setZero();
      this->basis_.block<3, 3>(0, 3).setZero();
      break;
    case ComputationalSpaceType::ANGULAR:
      // return only the angular block
      this->basis_.block<3, 3>(0, 0).setZero();
      this->basis_.block<3, 3>(3, 3) = this->compute_orthonormal_basis(this->basis_.block<3, 3>(3, 3), desired_twist.segment(3, 3));
      this->basis_.block<3, 3>(3, 0).setZero();
      this->basis_.block<3, 3>(0, 3).setZero();
      break;
    case ComputationalSpaceType::TWIST:
      // return the full damping matrix
      this->basis_ = this->compute_orthonormal_basis(this->basis_, desired_twist);
      break;
    case ComputationalSpaceType::DECOUPLED_TWIST:
      // compute per block
      this->basis_.block<3, 3>(0, 0) = this->compute_orthonormal_basis(this->basis_.block<3, 3>(0, 0), desired_twist.segment(0, 3));
      this->basis_.block<3, 3>(3, 3) = this->compute_orthonormal_basis(this->basis_.block<3, 3>(3, 3), desired_twist.segment(3, 3));
      this->basis_.block<3, 3>(3, 0).setZero();
      this->basis_.block<3, 3>(0, 3).setZero();
      break;
  }
  this->set_damping(this->basis_ * this->get_diagonal_eigenvalues() * this->basis_.transpose());
}

StateRepresentation::CartesianState Dissipative::compute_command(const StateRepresentation::CartesianState& desired_state,
                                                                 const StateRepresentation::CartesianState& feedback_state) {
  // compute the damping matrix out of the desired_state twist
  this->compute_damping(desired_state.get_twist());
  // apply the impedance control law
  return this->Impedance<StateRepresentation::CartesianState>::compute_command(desired_state, feedback_state);
}
}// namespace impedance
}// namespace controllers