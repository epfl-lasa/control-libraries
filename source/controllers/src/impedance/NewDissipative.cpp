#include "controllers/impedance/NewDissipative.hpp"

#include "controllers/exceptions/NotImplementedException.hpp"

using namespace state_representation;

namespace controllers::impedance {

template<class S>
Eigen::MatrixXd NewDissipative<S>::compute_orthonormal_basis(const S&) {
  throw exceptions::NotImplementedException(
      "compute_orthonormal_basis(desired_velocity) not implemented for this input class");
}

template<>
Eigen::MatrixXd NewDissipative<CartesianState>::compute_orthonormal_basis(const CartesianState& desired_velocity) {
  double tolerance = 1e-4;
  Eigen::MatrixXd updated_basis = Eigen::MatrixXd::Zero(this->dimensions_, this->dimensions_);
  switch (this->computational_space_) {
    case ComputationalSpaceType::LINEAR: {
      const Eigen::Vector3d& linear_velocity = desired_velocity.get_linear_velocity();
      // only update the damping if the commanded linear velocity is non-null
      if (linear_velocity.norm() < tolerance) {
        updated_basis.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity();
        return updated_basis;
      }
      //return only the linear block
      updated_basis.topLeftCorner<3, 3>() =
          NewDissipative<CartesianState>::orthonormalize_basis(this->basis_.topLeftCorner<3, 3>(), linear_velocity);
      break;
    }
    case ComputationalSpaceType::ANGULAR: {
      const Eigen::Vector3d& angular_velocity = desired_velocity.get_angular_velocity();
      // only update the damping if the commanded angular velocity is non-null
      if (angular_velocity.norm() < tolerance) {
        updated_basis.bottomRightCorner<3, 3>() = Eigen::Matrix3d::Identity();
        return updated_basis;
      }
      // return only the angular block
      updated_basis.bottomRightCorner<3, 3>() = NewDissipative<CartesianState>::orthonormalize_basis(
          this->basis_.bottomRightCorner<3, 3>(), angular_velocity);
      break;
    }
    case ComputationalSpaceType::DECOUPLED_TWIST: {
      // compute per block
      bool updated = false;
      const Eigen::Vector3d& linear_velocity = desired_velocity.get_linear_velocity();
      const Eigen::Vector3d& angular_velocity = desired_velocity.get_angular_velocity();
      if (linear_velocity.norm() > tolerance) {
        updated_basis.block<3, 3>(0, 0) =
            NewDissipative<CartesianState>::orthonormalize_basis(this->basis_.topLeftCorner<3, 3>(), linear_velocity);
        updated = true;
      }
      if (angular_velocity.norm() > tolerance) {
        updated_basis.block<3, 3>(3, 3) = NewDissipative<CartesianState>::orthonormalize_basis(
            this->basis_.bottomRightCorner<3, 3>(), angular_velocity);
        updated = true;
      }
      // at least the linear or angular parts have been updated
      if (!updated) {
        return Eigen::MatrixXd::Identity(this->dimensions_, this->dimensions_);
      }
      break;
    }
    case ComputationalSpaceType::FULL: {
      // only update the damping if the commanded velocity is non null
      if (desired_velocity.get_twist().norm() < tolerance) {
        return Eigen::MatrixXd::Identity(this->dimensions_, this->dimensions_);
      }
      // return the full damping matrix
      updated_basis = NewDissipative<CartesianState>::orthonormalize_basis(this->basis_, desired_velocity.get_twist());
      break;
    }
  }
  return updated_basis;
}

template<>
Eigen::MatrixXd NewDissipative<JointState>::compute_orthonormal_basis(const JointState& desired_velocity) {
  if (desired_velocity.get_size() != this->dimensions_) {
    throw state_representation::exceptions::IncompatibleSizeException(
        "The input state is of incorrect dimensions, expected " + std::to_string(this->dimensions_) + " got "
            + std::to_string(desired_velocity.get_size()));
  }
  double tolerance = 1e-4;
  // only update the damping if the commanded velocity is non-null
  if (desired_velocity.get_velocities().norm() < tolerance) {
    return Eigen::MatrixXd::Identity(this->dimensions_, this->dimensions_);
  }
  // return the full damping matrix
  return NewDissipative<JointState>::orthonormalize_basis(this->basis_, desired_velocity.get_velocities());
}
}// namespace controllers