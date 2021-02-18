#include "state_representation/Robot/Jacobian.hpp"
#include "state_representation/Exceptions/EmptyStateException.hpp"
#include "state_representation/Exceptions/IncompatibleStatesException.hpp"

namespace StateRepresentation {
Jacobian::Jacobian() : State(StateType::JACOBIANMATRIX), nb_rows_(6) {}

Jacobian::Jacobian(const std::string& robot_name, unsigned int nb_joints) :
    State(StateType::JACOBIANMATRIX, robot_name), nb_rows_(6), nb_cols_(nb_joints) {
  this->set_joint_names(nb_joints);
}

Jacobian::Jacobian(const std::string& robot_name, const std::vector<std::string>& joint_names) :
    State(StateType::JACOBIANMATRIX, robot_name), nb_rows_(6), nb_cols_(joint_names.size()) {
  this->set_joint_names(joint_names);
}

Jacobian::Jacobian(const std::string& robot_name, const Eigen::MatrixXd& data) :
    State(StateType::JACOBIANMATRIX, robot_name), nb_rows_(data.rows()), nb_cols_(data.cols()) {
  this->set_joint_names(this->get_nb_cols());
  this->set_data(data);
}

Jacobian::Jacobian(const std::string& robot_name, const std::vector<std::string>& joint_names,
                   const Eigen::MatrixXd& data) :
    State(StateType::JACOBIANMATRIX, robot_name), nb_rows_(data.rows()), nb_cols_(joint_names.size()) {
  this->set_joint_names(joint_names);
  this->set_data(data);
}

Jacobian::Jacobian(const Jacobian& jacobian) :
    State(jacobian),
    joint_names_(jacobian.joint_names_),
    nb_rows_(jacobian.nb_rows_),
    nb_cols_(jacobian.nb_cols_),
    data_matrix_(jacobian.data_matrix_) {}

void Jacobian::initialize() {
  this->State::initialize();
  this->data_matrix_.resize(this->nb_rows_, this->nb_cols_);
  this->data_matrix_.setZero();
}

Jacobian Jacobian::transpose() const {
  Jacobian result(*this);
  result.set_nb_cols(this->get_nb_rows());
  result.set_nb_rows(this->get_nb_cols());
  result.set_data(result.data().transpose());
  return result;
}

Jacobian Jacobian::inverse() const {
  if (this->get_nb_rows() != this->get_nb_cols()) {
    throw Exceptions::IncompatibleSizeException(
        "The Jacobian matrix is not invertible, use the pseudoinverse function instead");
  }
  Jacobian result(*this);
  result.set_nb_cols(this->get_nb_cols());
  result.set_nb_rows(this->get_nb_rows());
  result.set_data(result.data().inverse());
  return result;
}

Jacobian Jacobian::pseudoinverse() const {
  Jacobian result(*this);
  Eigen::MatrixXd pinv = this->data().completeOrthogonalDecomposition().pseudoInverse();
  result.set_nb_cols(pinv.cols());
  result.set_nb_rows(pinv.rows());
  result.set_data(pinv);
  return result;
}

Eigen::MatrixXd Jacobian::operator*(const Eigen::MatrixXd& matrix) const {
  if (this->is_empty()) { throw EmptyStateException(this->get_name() + " state is empty"); }
  if (matrix.rows() != this->get_nb_cols()) { throw IncompatibleSizeException("Input matrix is of incorrect size"); }
  return this->data() * matrix;
}

CartesianTwist Jacobian::operator*(const JointVelocities& dq) const {
  if (dq.is_empty()) { throw EmptyStateException(dq.get_name() + " state is empty"); }
  if (!this->is_compatible(dq)) {
    throw IncompatibleStatesException("The Jacobian and the input JointVelocities are incompatible");
  }
  Eigen::Matrix<double, 6, 1> twist = (*this) * dq.data();
  // crate a CartesianTwist with name "robot"_end_effector and reference frame "robot"_base
  CartesianTwist result(this->get_name() + "_end_effector", twist, this->get_name() + "_base");
  return result;
}

JointVelocities Jacobian::operator*(const CartesianTwist& twist) const {
  Eigen::VectorXd joint_velocities = (*this) * twist.data();
  JointVelocities result(this->get_name(), this->get_joint_names(), joint_velocities);
  return result;
}

JointTorques Jacobian::operator*(const CartesianWrench& wrench) const {
  Eigen::VectorXd joint_torques = (*this) * wrench.data();
  JointTorques result(this->get_name(), this->get_joint_names(), joint_torques);
  return result;
}

Eigen::MatrixXd Jacobian::solve(const Eigen::MatrixXd& matrix) const {
  if (this->is_empty()) { throw EmptyStateException(this->get_name() + " state is empty"); }
  if (this->get_nb_rows() != matrix.rows()) { throw IncompatibleSizeException("Input matrix is of incorrect size"); }
  return this->data().colPivHouseholderQr().solve(matrix);
}

JointVelocities Jacobian::solve(const CartesianTwist& twist) const {
  if (twist.is_empty()) { throw EmptyStateException(twist.get_name() + " state is empty"); }
  // this use the solve operation instead of using the inverse or pseudo-inverse of the jacobian
  Eigen::VectorXd joint_velocities = this->solve(twist.data());
  // return a JointVelocities state
  JointVelocities result(this->get_name(), this->get_joint_names(), joint_velocities);
  return result;
}

Jacobian Jacobian::copy() const {
  Jacobian result(*this);
  return result;
}

std::ostream& operator<<(std::ostream& os, const Jacobian& matrix) {
  if (matrix.is_empty()) {
    os << "Empty Jacobian";
  } else {
    os << matrix.get_name() << " Jacobian" << std::endl;
    os << "joint names: [";
    for (auto& n : matrix.get_joint_names()) { os << n << ", "; }
    os << "]" << std::endl;
    os << "number of rows: " << matrix.get_nb_rows() << std::endl;
    os << "number of columns: " << matrix.get_nb_cols() << std::endl;
    for (unsigned int i = 0; i < matrix.get_nb_rows(); ++i) {
      os << "| " << matrix(i, 0);
      for (unsigned int j = 1; j < matrix.get_nb_cols(); ++j) {
        os << ", " << matrix(i, j);
      }
      os << " |" << std::endl;
    }
  }
  return os;
}
}// namespace StateRepresentation
