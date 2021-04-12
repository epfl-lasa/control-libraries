#include "state_representation/space/cartesian/CartesianPose.hpp"
#include "state_representation/exceptions/EmptyStateException.hpp"
#include "state_representation/exceptions/IncompatibleSizeException.hpp"

using namespace state_representation::exceptions;

namespace state_representation {
CartesianPose::CartesianPose() = default;

CartesianPose::CartesianPose(const std::string& name, const std::string& reference) : CartesianState(name, reference) {}

CartesianPose::CartesianPose(const std::string& name, const Eigen::Vector3d& position, const std::string& reference) :
    CartesianState(name, reference) {
  this->set_position(position);
}

CartesianPose::CartesianPose(const std::string& name,
                             const double& x,
                             const double& y,
                             const double& z,
                             const std::string& reference) : CartesianState(name, reference) {
  this->set_position(x, y, z);
}

CartesianPose::CartesianPose(const std::string& name,
                             const Eigen::Vector3d& position,
                             const Eigen::Quaterniond& orientation,
                             const std::string& reference) : CartesianState(name, reference) {
  this->set_position(position);
  this->set_orientation(orientation);
}

CartesianPose::CartesianPose(const CartesianState& state) : CartesianState(state) {
  // set all the state variables to 0 except position and orientation
  this->set_zero();
  this->set_pose(state.get_pose());
}

CartesianPose::CartesianPose(const CartesianPose& pose) : CartesianPose(static_cast<const CartesianState&>(pose)) {}

CartesianPose::CartesianPose(const CartesianTwist& twist) : CartesianPose(std::chrono::seconds(1) * twist) {}

CartesianPose CartesianPose::Identity(const std::string& name, const std::string& reference) {
  return CartesianState::Identity(name, reference);
}

CartesianPose CartesianPose::Random(const std::string& name, const std::string& reference) {
  return CartesianPose(name, Eigen::Vector3d::Random(), Eigen::Quaterniond::UnitRandom(), reference);
}

CartesianPose& CartesianPose::operator=(const CartesianState& state) {
  this->CartesianState::operator=(state);
  return (*this);
}

Eigen::Vector3d CartesianPose::operator*(const Eigen::Vector3d& vector) const {
  return this->get_orientation() * vector + this->get_position();
}

CartesianPose& CartesianPose::operator*=(const CartesianPose& pose) {
  this->CartesianState::operator*=(pose);
  return (*this);
}

CartesianPose CartesianPose::operator*(const CartesianPose& pose) const {
  return this->CartesianState::operator*(pose);
}

CartesianPose& CartesianPose::operator*=(double lambda) {
  this->CartesianState::operator*=(lambda);
  return (*this);
}

CartesianPose CartesianPose::operator*(double lambda) const {
  return this->CartesianState::operator*(lambda);
}

CartesianPose& CartesianPose::operator+=(const CartesianPose& pose) {
  this->CartesianState::operator+=(pose);
  return (*this);
}

CartesianPose CartesianPose::operator+(const CartesianPose& pose) const {
  return this->CartesianState::operator+(pose);
}

CartesianPose& CartesianPose::operator-=(const CartesianPose& pose) {
  this->CartesianState::operator-=(pose);
  return (*this);
}

CartesianPose CartesianPose::operator-(const CartesianPose& pose) const {
  return this->CartesianState::operator-(pose);
}

CartesianTwist CartesianPose::operator/(const std::chrono::nanoseconds& dt) const {
  // sanity check
  if (this->is_empty()) {
    throw EmptyStateException(this->get_name() + " state is empty");
  }
  // operations
  CartesianTwist twist(this->get_name(), this->get_reference_frame());
  // convert the period to a double with the second as reference
  double period = dt.count();
  period /= 1e9;
  // set linear velocity
  twist.set_linear_velocity(this->get_position() / period);
  // set angular velocity from the log of the quaternion error
  Eigen::Quaterniond log_q = math_tools::log(this->get_orientation());
  if (this->get_orientation().dot(log_q) < 0) {
    log_q = Eigen::Quaterniond(-log_q.coeffs());
  }
  twist.set_angular_velocity(2 * log_q.vec() / period);
  return twist;
}

CartesianPose CartesianPose::copy() const {
  CartesianPose result(*this);
  return result;
}

Eigen::VectorXd CartesianPose::data() const {
  return this->get_pose();
}

std::ostream& operator<<(std::ostream& os, const CartesianPose& pose) {
  if (pose.is_empty()) {
    os << "Empty CartesianPose";
  } else {
    os << pose.get_name() << " CartesianPose expressed in " << pose.get_reference_frame() << " frame" << std::endl;
    os << "position: (" << pose.get_position()(0) << ", ";
    os << pose.get_position()(1) << ", ";
    os << pose.get_position()(2) << ")" << std::endl;
    os << "orientation: (" << pose.get_orientation().w() << ", ";
    os << pose.get_orientation().x() << ", ";
    os << pose.get_orientation().y() << ", ";
    os << pose.get_orientation().z() << ")";
    Eigen::AngleAxisd axis_angle(pose.get_orientation());
    os << " <=> theta: " << axis_angle.angle() << ", ";
    os << "axis: (" << axis_angle.axis()(0) << ", ";
    os << axis_angle.axis()(1) << ", ";
    os << axis_angle.axis()(2) << ")";
  }
  return os;
}

CartesianPose operator*(double lambda, const CartesianPose& pose) {
  return pose * lambda;
}

void CartesianPose::from_std_vector(const std::vector<double>& value) {
  if (value.size() == 3) {
    this->set_position(value);
  } else if (value.size() == 4) {
    this->set_orientation(value);
  } else if (value.size() == 7) {
    this->set_pose(value);
  } else {
    throw IncompatibleSizeException("The input vector is of incorrect size");
  }
}
}// namespace state_representation
