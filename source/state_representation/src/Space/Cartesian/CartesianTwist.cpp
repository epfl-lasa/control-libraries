#include "state_representation/Space/Cartesian/CartesianTwist.hpp"
#include "state_representation/Exceptions/EmptyStateException.hpp"
#include "state_representation/Exceptions/IncompatibleReferenceFramesException.hpp"
#include "state_representation/Exceptions/IncompatibleStatesException.hpp"

using namespace StateRepresentation::Exceptions;

namespace StateRepresentation {
CartesianTwist::CartesianTwist() {}

CartesianTwist::CartesianTwist(const std::string& name, const std::string& reference) : CartesianState(name, reference) {}

CartesianTwist::CartesianTwist(const std::string& name, const Eigen::Vector3d& linear_velocity, const std::string& reference) : CartesianState(name, reference) {
  this->set_linear_velocity(linear_velocity);
}

CartesianTwist::CartesianTwist(const std::string& name, const Eigen::Vector3d& linear_velocity, const Eigen::Vector3d& angular_velocity, const std::string& reference) : CartesianState(name, reference) {
  this->set_linear_velocity(linear_velocity);
  this->set_angular_velocity(angular_velocity);
}

CartesianTwist::CartesianTwist(const std::string& name, const Eigen::Matrix<double, 6, 1>& twist, const std::string& reference) : CartesianState(name, reference) {
  this->set_twist(twist);
}

CartesianTwist::CartesianTwist(const CartesianTwist& twist) : CartesianState(twist) {}

CartesianTwist::CartesianTwist(const CartesianState& state) : CartesianState(state) {}

CartesianTwist::CartesianTwist(const CartesianPose& pose) : CartesianState(pose / std::chrono::seconds(1)) {}

CartesianTwist& CartesianTwist::operator=(const CartesianState& state) {
  this->CartesianState::operator=(state);
  return (*this);
}

CartesianTwist& CartesianTwist::operator=(const Eigen::Matrix<double, 6, 1>& twist) {
  this->set_twist(twist);
  return (*this);
}

CartesianTwist& CartesianTwist::operator*=(const CartesianTwist& twist) {
  this->CartesianState::operator*=(twist);
  return (*this);
}

const CartesianTwist CartesianTwist::operator*(const CartesianTwist& twist) const {
  return this->CartesianState::operator*(twist);
}

const CartesianState CartesianTwist::operator*(const CartesianState& state) const {
  return this->CartesianState::operator*(state);
}

CartesianTwist& CartesianTwist::operator*=(double lambda) {
  this->CartesianState::operator*=(lambda);
  return (*this);
}

const CartesianTwist CartesianTwist::operator*(double lambda) const {
  return this->CartesianState::operator*(lambda);
}

CartesianTwist& CartesianTwist::operator+=(const CartesianTwist& twist) {
  this->CartesianState::operator+=(twist);
  return (*this);
}

const CartesianTwist CartesianTwist::operator+(const CartesianTwist& twist) const {
  return this->CartesianState::operator+(twist);
}

const CartesianState CartesianTwist::operator+(const CartesianState& state) const {
  return this->CartesianState::operator+(state);
}

CartesianTwist& CartesianTwist::operator-=(const CartesianTwist& twist) {
  this->CartesianState::operator-=(twist);
  return (*this);
}

const CartesianTwist CartesianTwist::operator-(const CartesianTwist& twist) const {
  return this->CartesianState::operator-(twist);
}

const CartesianState CartesianTwist::operator-(const CartesianState& state) const {
  return this->CartesianState::operator-(state);
}

CartesianTwist& CartesianTwist::operator+=(const Eigen::Matrix<double, 6, 1>& vector) {
  if (this->is_empty()) throw EmptyStateException(this->get_name() + " state is empty");
  this->set_twist(this->get_twist() + vector);
  return (*this);
}

const CartesianTwist CartesianTwist::operator+(const Eigen::Matrix<double, 6, 1>& vector) const {
  CartesianTwist result(*this);
  result += vector;
  return result;
}

CartesianTwist& CartesianTwist::operator-=(const Eigen::Matrix<double, 6, 1>& vector) {
  if (this->is_empty()) throw EmptyStateException(this->get_name() + " state is empty");
  this->set_twist(this->get_twist() - vector);
  return (*this);
}

const CartesianTwist CartesianTwist::operator-(const Eigen::Matrix<double, 6, 1>& vector) const {
  CartesianTwist result(*this);
  result -= vector;
  return result;
}

CartesianTwist& CartesianTwist::operator*=(const Eigen::Matrix<double, 6, 6>& lambda) {
  // sanity check
  if (this->is_empty()) throw EmptyStateException(this->get_name() + " state is empty");
  // operation
  this->set_linear_velocity(lambda.block<3, 3>(0, 0) * this->get_linear_velocity());
  this->set_angular_velocity(lambda.block<3, 3>(3, 3) * this->get_angular_velocity());
  return (*this);
}

void CartesianTwist::clamp(double max_linear, double max_angular, double linear_noise_ratio, double angular_noise_ratio) {
  if (linear_noise_ratio != 0 || angular_noise_ratio != 0) {
    // substract the noise ratio to both velocities
    this->set_linear_velocity(this->get_linear_velocity() - linear_noise_ratio * this->get_linear_velocity().normalized());
    this->set_angular_velocity(this->get_angular_velocity() - angular_noise_ratio * this->get_angular_velocity().normalized());
    // apply a deadzone
    if (this->get_linear_velocity().norm() < linear_noise_ratio) this->set_linear_velocity(Eigen::Vector3d::Zero());
    if (this->get_angular_velocity().norm() < angular_noise_ratio) this->set_angular_velocity(Eigen::Vector3d::Zero());
  }
  // clamp the velocities to their maximum amplitude provided
  if (this->get_linear_velocity().norm() > max_linear) this->set_linear_velocity(max_linear * this->get_linear_velocity().normalized());
  if (this->get_angular_velocity().norm() > max_angular) this->set_angular_velocity(max_angular * this->get_angular_velocity().normalized());
}

const CartesianTwist CartesianTwist::clamped(double max_linear, double max_angular, double linear_noise_ratio, double angular_noise_ratio) const {
  CartesianTwist result(*this);
  result.clamp(max_linear, max_angular, linear_noise_ratio, angular_noise_ratio);
  return result;
}

const CartesianTwist CartesianTwist::copy() const {
  CartesianTwist result(*this);
  return result;
}

const Eigen::Array<double, 6, 1> CartesianTwist::array() const {
  return this->get_twist().array();
}

std::ostream& operator<<(std::ostream& os, const CartesianTwist& twist) {
  if (twist.is_empty()) {
    os << "Empty CartesianTwist";
  } else {
    os << twist.get_name() << " CartesianTwist expressed in " << twist.get_reference_frame() << " frame" << std::endl;
    os << "linear_velocity: (" << twist.get_linear_velocity()(0) << ", ";
    os << twist.get_linear_velocity()(1) << ", ";
    os << twist.get_linear_velocity()(2) << ")" << std::endl;
    os << "angular_velocity: (" << twist.get_angular_velocity()(0) << ", ";
    os << twist.get_angular_velocity()(1) << ", ";
    os << twist.get_angular_velocity()(2) << ")";
  }
  return os;
}

const CartesianTwist operator+(const Eigen::Matrix<double, 6, 1>& vector, const CartesianTwist& twist) {
  return twist + vector;
}

const CartesianTwist operator-(const Eigen::Matrix<double, 6, 1>& vector, const CartesianTwist& twist) {
  return vector + (-1) * twist;
}

const CartesianTwist operator*(double lambda, const CartesianTwist& twist) {
  return twist * lambda;
}

const CartesianTwist operator*(const Eigen::Matrix<double, 6, 6>& lambda, const CartesianTwist& twist) {
  CartesianTwist result(twist);
  result *= lambda;
  return result;
}

const CartesianPose operator*(const std::chrono::nanoseconds& dt, const CartesianTwist& twist) {
  // sanity check
  if (twist.is_empty()) throw EmptyStateException(twist.get_name() + " state is empty");
  // operations
  CartesianPose displacement(twist.get_name(), twist.get_reference_frame());
  // convert the period to a double with the second as reference
  double period = dt.count();
  period /= 1e9;
  // convert the velocities into a displacement
  displacement.set_position(period * twist.get_linear_velocity());
  Eigen::Quaterniond angular_displacement = Eigen::Quaterniond::Identity();
  double angular_norm = twist.get_angular_velocity().norm();
  if (angular_norm > 1e-4) {
    double theta = angular_norm * period * 0.5;
    angular_displacement.w() = cos(theta);
    angular_displacement.vec() = twist.get_angular_velocity() / angular_norm * sin(theta);
  }
  displacement.set_orientation(angular_displacement);
  return displacement;
}

const CartesianPose operator*(const CartesianTwist& twist, const std::chrono::nanoseconds& dt) {
  return dt * twist;
}
}// namespace StateRepresentation
