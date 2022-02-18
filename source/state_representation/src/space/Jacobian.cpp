#include "state_representation/space/Jacobian.hpp"

#include "state_representation/exceptions/EmptyStateException.hpp"
#include "state_representation/exceptions/IncompatibleStatesException.hpp"

namespace state_representation {
Jacobian::Jacobian() : State(StateType::JACOBIANMATRIX) {
  this->State::initialize();
}

Jacobian::Jacobian(const std::string& robot_name,
                   unsigned int nb_joints,
                   const std::string& frame,
                   const std::string& reference_frame) :
    State(StateType::JACOBIANMATRIX, robot_name),
    joint_names_(nb_joints),
    frame_(frame),
    reference_frame_(reference_frame),
    rows_(6),
    cols_(nb_joints) {
  this->set_joint_names(nb_joints);
  this->initialize();
}

Jacobian::Jacobian(const std::string& robot_name,
                   const std::vector<std::string>& joint_names,
                   const std::string& frame,
                   const std::string& reference_frame) :
    State(StateType::JACOBIANMATRIX, robot_name),
    joint_names_(joint_names),
    frame_(frame),
    reference_frame_(reference_frame),
    rows_(6),
    cols_(joint_names.size()) {
  this->initialize();
}

Jacobian::Jacobian(const std::string& robot_name,
                   const std::string& frame,
                   const Eigen::MatrixXd& data,
                   const std::string& reference_frame) :
    Jacobian(robot_name, data.cols(), frame, reference_frame) {
  this->set_data(data);
}

Jacobian::Jacobian(const std::string& robot_name,
                   const std::vector<std::string>& joint_names,
                   const std::string& frame,
                   const Eigen::MatrixXd& data,
                   const std::string& reference_frame) :
    Jacobian(robot_name, joint_names, frame, reference_frame) {
  this->set_data(data);
}

Jacobian::Jacobian(const Jacobian& jacobian) :
    State(jacobian),
    joint_names_(jacobian.joint_names_),
    frame_(jacobian.frame_),
    reference_frame_(jacobian.reference_frame_),
    rows_(jacobian.rows_),
    cols_(jacobian.cols()),
    data_(jacobian.data_) {}

void Jacobian::initialize() {
  this->State::initialize();
  this->data_.resize(this->rows_, this->cols());
  this->data_.setZero();
}

Jacobian Jacobian::Random(const std::string& robot_name,
                          unsigned int nb_joints,
                          const std::string& frame,
                          const std::string& reference_frame) {
  Jacobian random(robot_name, nb_joints, frame, reference_frame);
  random.set_data(Eigen::MatrixXd::Random(random.rows_, random.cols_));
  return random;
}

Jacobian Jacobian::Random(const std::string& robot_name,
                          const std::vector<std::string>& joint_names,
                          const std::string& frame,
                          const std::string& reference_frame) {
  Jacobian random(robot_name, joint_names, frame, reference_frame);
  random.set_data(Eigen::MatrixXd::Random(random.rows_, random.cols_));
  return random;
}

bool Jacobian::is_compatible(const State& state) const {
  bool compatible = false;
  switch (state.get_type()) {
    case StateType::JACOBIANMATRIX:
      // compatibility is assured through the vector of joint names
      compatible = (this->get_name() == state.get_name())
          && (this->cols_ == dynamic_cast<const Jacobian&>(state).get_joint_names().size());
      if (compatible) {
        for (unsigned int i = 0; i < this->cols_; ++i) {
          compatible = (compatible && this->joint_names_[i] == dynamic_cast<const Jacobian&>(state).get_joint_names()[i]);
        }
        // compatibility is assured through the reference frame and the name of the frame
        compatible = (compatible && ((this->reference_frame_ == dynamic_cast<const Jacobian&>(state).get_reference_frame())
            && (this->frame_ == dynamic_cast<const Jacobian&>(state).get_frame())));
      }
      break;
    case StateType::JOINTSTATE:
      // compatibility is assured through the vector of joint names
      compatible = (this->get_name() == state.get_name())
          && (this->cols_ == dynamic_cast<const JointState&>(state).get_size());
      if (compatible) {
        for (unsigned int i = 0; i < this->cols_; ++i) {
          compatible = (compatible && this->joint_names_[i] == dynamic_cast<const JointState&>(state).get_names()[i]);
        }
      }
      break;
    case StateType::CARTESIANSTATE:
      // compatibility is assured through the reference frame and the name of the frame
      compatible = (this->reference_frame_ == dynamic_cast<const CartesianState&>(state).get_reference_frame())
          && (this->frame_ == dynamic_cast<const CartesianState&>(state).get_name());
      break;
    default:
      break;
  }
  return compatible;
}

void Jacobian::set_reference_frame(const CartesianPose& reference_frame) {
  *this = reference_frame * (*this);
}

Jacobian Jacobian::transpose() const {
  Jacobian result(*this);
  result.cols_ = this->rows_;
  result.rows_ = this->cols_;
  result.set_data(result.data().transpose());
  return result;
}

Jacobian Jacobian::inverse() const {
  if (this->rows_ != this->cols_) {
    throw exceptions::IncompatibleSizeException(
        "The Jacobian matrix is not invertible, use the pseudoinverse function instead");
  }
  Jacobian result(*this);
  result.cols_ = this->cols_;
  result.rows_ = this->rows_;
  result.set_data(result.data().inverse());
  return result;
}

Jacobian Jacobian::pseudoinverse() const {
  Jacobian result(*this);
  Eigen::MatrixXd pinv = this->data().completeOrthogonalDecomposition().pseudoInverse();
  result.cols_ = pinv.cols();
  result.rows_ = pinv.rows();
  result.set_data(pinv);
  return result;
}

Eigen::MatrixXd Jacobian::operator*(const Eigen::MatrixXd& matrix) const {
  if (this->is_empty()) {
    throw EmptyStateException(this->get_name() + " state is empty");
  }
  if (matrix.rows() != this->cols_) {
    throw IncompatibleSizeException("Input matrix is of incorrect size, expected "
                                        + std::to_string(this->cols_) + " rows, got " + std::to_string(matrix.rows()));
  }
  return this->data() * matrix;
}

Eigen::MatrixXd Jacobian::operator*(const Jacobian& jacobian) const {
  if (!this->is_compatible(jacobian)) {
    throw IncompatibleStatesException("The two Jacobian matrices are not compatible");
  }
  // multiply with the data of the second Jacobian
  return (*this) * jacobian.data();
}

CartesianTwist Jacobian::operator*(const JointVelocities& dq) const {
  if (this->is_empty()) {
    throw EmptyStateException(this->get_name() + " state is empty");
  }
  if (dq.is_empty()) {
    throw EmptyStateException(dq.get_name() + " state is empty");
  }
  if (!this->is_compatible(dq)) {
    throw IncompatibleStatesException("The Jacobian and the input JointVelocities are incompatible");
  }
  Eigen::Matrix<double, 6, 1> twist = (*this) * dq.data();
  CartesianTwist result(this->frame_, twist, this->reference_frame_);
  return result;
}

JointVelocities Jacobian::operator*(const CartesianTwist& twist) const {
  if (this->is_empty()) {
    throw EmptyStateException(this->get_name() + " state is empty");
  }
  if (twist.is_empty()) {
    throw EmptyStateException(twist.get_name() + " state is empty");
  }
  if (!this->is_compatible(twist)) {
    throw IncompatibleStatesException("The Jacobian and the input CartesianTwist are incompatible");
  }
  Eigen::VectorXd joint_velocities = (*this) * twist.data();
  JointVelocities result(this->get_name(), this->get_joint_names(), joint_velocities);
  return result;
}

JointTorques Jacobian::operator*(const CartesianWrench& wrench) const {
  if (this->is_empty()) {
    throw EmptyStateException(this->get_name() + " state is empty");
  }
  if (wrench.is_empty()) {
    throw EmptyStateException(wrench.get_name() + " state is empty");
  }
  if (!this->is_compatible(wrench)) {
    throw IncompatibleStatesException("The Jacobian and the input CartesianWrench are incompatible");
  }
  Eigen::VectorXd joint_torques = (*this) * wrench.data();
  JointTorques result(this->get_name(), this->get_joint_names(), joint_torques);
  return result;
}

Eigen::MatrixXd Jacobian::solve(const Eigen::MatrixXd& matrix) const {
  if (this->is_empty()) {
    throw EmptyStateException(this->get_name() + " state is empty");
  }
  if (this->rows_ != matrix.rows()) {
    throw IncompatibleSizeException("Input matrix is of incorrect size, expected "
                                        + std::to_string(this->rows_) + " rows, got " + std::to_string(matrix.rows()));
  }
  return this->data().colPivHouseholderQr().solve(matrix);
}

JointVelocities Jacobian::solve(const CartesianTwist& twist) const {
  if (this->is_empty()) {
    throw EmptyStateException(this->get_name() + " state is empty");
  }
  if (twist.is_empty()) {
    throw EmptyStateException(twist.get_name() + " state is empty");
  }
  if (!this->is_compatible(twist)) {
    throw IncompatibleStatesException("The Jacobian and the input CartesianTwist are incompatible");
  }
  // this uses the solve operation instead of using the inverse or pseudo-inverse of the Jacobian
  Eigen::VectorXd joint_velocities = this->solve(twist.data());
  // return a JointVelocities state
  JointVelocities result(this->get_name(), this->get_joint_names(), joint_velocities);
  return result;
}

Jacobian Jacobian::copy() const {
  Jacobian result(*this);
  return result;
}

std::ostream& operator<<(std::ostream& os, const Jacobian& jacobian) {
  if (jacobian.is_empty()) {
    os << "Empty Jacobian";
  } else {
    os << jacobian.get_name() << " Jacobian associated to " << jacobian.frame_;
    os << ", expressed in " << jacobian.reference_frame_ << std::endl;
    os << "joint names: [";
    for (auto& n : jacobian.get_joint_names()) { os << n << ", "; }
    os << "]" << std::endl;
    os << "number of rows: " << jacobian.rows_ << std::endl;
    os << "number of columns: " << jacobian.cols_ << std::endl;
    for (unsigned int i = 0; i < jacobian.rows_; ++i) {
      os << "| " << jacobian(i, 0);
      for (unsigned int j = 1; j < jacobian.cols_; ++j) {
        os << ", " << jacobian(i, j);
      }
      os << " |";
      if (i != jacobian.rows_ - 1) {
        os << std::endl;
      }
    }
  }
  return os;
}

Jacobian operator*(const CartesianPose& pose, const Jacobian& jacobian) {
  // check compatibility
  if (jacobian.is_empty()) {
    throw EmptyStateException(jacobian.get_name() + " state is empty");
  }
  if (pose.is_empty()) {
    throw EmptyStateException(pose.get_name() + " state is empty");
  }
  if (pose.get_name() != jacobian.get_reference_frame()) {
    throw IncompatibleStatesException("The Jacobian and the input CartesianPose are incompatible, expected pose of "
                                          + jacobian.get_reference_frame() + " got " + pose.get_name());
  }
  // number of rows of the jacobian should be 6 (incorrect if it has been transposed before)
  if (jacobian.rows_ != 6) {
    throw IncompatibleStatesException(
        "The Jacobian and the input CartesianPose are incompatible, the Jacobian has probably been transposed before");
  }
  Jacobian result(jacobian);
  // change the reference frame of all the columns
  for (unsigned int i = 0; i < jacobian.cols_; ++i) {
    // update position part
    result.data_.col(i).head(3) = pose.get_orientation() * jacobian.data_.col(i).head(3);
    // update orientation part
    result.data_.col(i).tail(3) = pose.get_orientation() * jacobian.data_.col(i).tail(3);
  }
  // change the reference frame
  result.reference_frame_ = pose.get_reference_frame();
  return result;
}

Eigen::MatrixXd operator*(const Eigen::MatrixXd& matrix, const Jacobian& jacobian) {
  // check compatibility
  if (jacobian.is_empty()) {
    throw EmptyStateException(jacobian.get_name() + " state is empty");
  }
  if (matrix.cols() != jacobian.rows()) {
    throw IncompatibleStatesException("The matrix and the Jacobian have incompatible sizes");
  }
  return matrix * jacobian.data();
}
}// namespace state_representation
