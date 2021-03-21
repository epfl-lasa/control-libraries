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
Eigen::MatrixXd Dissipative<S>::compute_orthonormal_basis(const S&) {
  throw exceptions::NotImplementedException(
      "compute_orthonormal_basis(desired_velocity) not implemented for this input class");
  return Eigen::MatrixXd::Zero(this->nb_dimensions_, this->nb_dimensions_);
}

template<>
Eigen::MatrixXd Dissipative<CartesianState>::compute_orthonormal_basis(const CartesianState& desired_velocity) {
  double tolerance = 1e-4;
  Eigen::MatrixXd updated_basis = Eigen::MatrixXd::Zero(this->nb_dimensions_, this->nb_dimensions_);
  switch (this->computational_space_) {
    case ComputationalSpaceType::LINEAR: {
      const Eigen::Vector3d& linear_velocity = desired_velocity.get_linear_velocity();
      // only update the damping if the commanded linear velocity is non null
      if (linear_velocity.norm() < tolerance) {
        updated_basis.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity();
        return updated_basis;
      }
      //return only the linear block
      updated_basis.topLeftCorner<3, 3>() =
          Dissipative<CartesianState>::orthonormalize_basis(this->basis_.topLeftCorner<3, 3>(), linear_velocity);
      break;
    }
    case ComputationalSpaceType::ANGULAR: {
      const Eigen::Vector3d& angular_velocity = desired_velocity.get_angular_velocity();
      // only update the damping if the commanded angular velocity is non null
      if (angular_velocity.norm() < tolerance) {
        updated_basis.bottomRightCorner<3, 3>() = Eigen::Matrix3d::Identity();
        return updated_basis;
      }
      // return only the angular block
      updated_basis.bottomRightCorner<3, 3>() =
          Dissipative<CartesianState>::orthonormalize_basis(this->basis_.bottomRightCorner<3, 3>(), angular_velocity);
      break;
    }
    case ComputationalSpaceType::DECOUPLED_TWIST: {
      // compute per block
      bool updated = false;
      const Eigen::Vector3d& linear_velocity = desired_velocity.get_linear_velocity();
      const Eigen::Vector3d& angular_velocity = desired_velocity.get_angular_velocity();
      if (linear_velocity.norm() > tolerance) {
        updated_basis.block<3, 3>(0, 0) =
            Dissipative<CartesianState>::orthonormalize_basis(this->basis_.topLeftCorner<3, 3>(), linear_velocity);
        updated = true;
      }
      if (angular_velocity.norm() > tolerance) {
        updated_basis.block<3, 3>(3, 3) =
            Dissipative<CartesianState>::orthonormalize_basis(this->basis_.bottomRightCorner<3, 3>(), angular_velocity);
        updated = true;
      }
      // at least the linear or angular parts have been updated
      if (!updated) {
        return Eigen::MatrixXd::Identity(this->nb_dimensions_, this->nb_dimensions_);
      }
      break;
    }
    case ComputationalSpaceType::FULL: {
      // only update the damping if the commanded velocity is non null
      if (desired_velocity.get_twist().norm() < tolerance) {
        return Eigen::MatrixXd::Identity(this->nb_dimensions_, this->nb_dimensions_);
      }
      // return the full damping matrix
      updated_basis = Dissipative<CartesianState>::orthonormalize_basis(this->basis_, desired_velocity.get_twist());
      break;
    }
  }
  return updated_basis;
}

template<>
Eigen::MatrixXd Dissipative<JointState>::compute_orthonormal_basis(const JointState& desired_velocity) {
  if (desired_velocity.get_size() != this->nb_dimensions_) {
    throw state_representation::exceptions::IncompatibleSizeException(
        "The input state is of incorrect dimensions, expected "
            + std::to_string(this->nb_dimensions_) + " got " + std::to_string(desired_velocity.get_size()));
  }
  double tolerance = 1e-4;
  // only update the damping if the commanded velocity is non null
  if (desired_velocity.get_velocities().norm() < tolerance) {
    return Eigen::MatrixXd::Identity(this->nb_dimensions_, this->nb_dimensions_);
  }
  // return the full damping matrix
  return Dissipative<JointState>::orthonormalize_basis(this->basis_, desired_velocity.get_velocities());
}
}// namespace controllers