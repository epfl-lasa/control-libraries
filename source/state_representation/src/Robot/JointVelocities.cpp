#include "state_representation/Robot/JointVelocities.hpp"
#include "state_representation/Exceptions/EmptyStateException.hpp"
#include "state_representation/Exceptions/IncompatibleStatesException.hpp"

using namespace StateRepresentation::Exceptions;

namespace StateRepresentation {
JointVelocities::JointVelocities() {}

JointVelocities::JointVelocities(const std::string& robot_name, unsigned int nb_joints) : JointState(robot_name, nb_joints) {}

JointVelocities::JointVelocities(const std::string& robot_name, const Eigen::VectorXd& velocities) : JointState(robot_name, velocities.size()) {
  this->set_velocities(velocities);
}

JointVelocities::JointVelocities(const std::string& robot_name, const std::vector<std::string>& joint_names) : JointState(robot_name, joint_names) {}

JointVelocities::JointVelocities(const std::string& robot_name, const std::vector<std::string>& joint_names, const Eigen::VectorXd& velocities) : JointState(robot_name, joint_names) {
  this->set_velocities(velocities);
}

JointVelocities::JointVelocities(const JointVelocities& velocities) : JointState(velocities) {}

JointVelocities::JointVelocities(const JointState& state) : JointState(state) {}

JointVelocities::JointVelocities(const JointPositions& positions) : JointState(positions / std::chrono::seconds(1)) {}

JointVelocities& JointVelocities::operator=(const Eigen::VectorXd& velocities) {
  this->set_velocities(velocities);
  return (*this);
}

JointVelocities& JointVelocities::operator+=(const Eigen::VectorXd& vector) {
  if (this->is_empty()) throw EmptyStateException(this->get_name() + " state is empty");
  if (this->get_size() != vector.size()) throw IncompatibleSizeException("Input vector is of incorrect size: expected " + std::to_string(this->get_size()) + ", given " + std::to_string(vector.size()));
  this->set_velocities(this->get_velocities() + vector);
  return (*this);
}

JointVelocities& JointVelocities::operator+=(const JointVelocities& velocities) {
  // sanity check
  if (this->is_empty()) throw EmptyStateException(this->get_name() + " state is empty");
  if (velocities.is_empty()) throw EmptyStateException(velocities.get_name() + " state is empty");
  if (!this->is_compatible(velocities)) throw IncompatibleStatesException("The two joint states are incompatible");
  // operation
  this->set_velocities(this->get_velocities() + velocities.get_velocities());
  return (*this);
}

const JointVelocities JointVelocities::operator+(const Eigen::VectorXd& vector) const {
  JointVelocities result(*this);
  result += vector;
  return result;
}

const JointVelocities JointVelocities::operator+(const JointVelocities& velocities) const {
  JointVelocities result(*this);
  result += velocities;
  return result;
}

JointVelocities& JointVelocities::operator-=(const Eigen::VectorXd& vector) {
  if (this->is_empty()) throw EmptyStateException(this->get_name() + " state is empty");
  if (this->get_size() != vector.size()) throw IncompatibleSizeException("Input vector is of incorrect size: expected " + std::to_string(this->get_size()) + ", given " + std::to_string(vector.size()));
  this->set_velocities(this->get_velocities() - vector);
  return (*this);
}

JointVelocities& JointVelocities::operator-=(const JointVelocities& velocities) {
  // sanity check
  if (this->is_empty()) throw EmptyStateException(this->get_name() + " state is empty");
  if (velocities.is_empty()) throw EmptyStateException(velocities.get_name() + " state is empty");
  if (!this->is_compatible(velocities)) throw IncompatibleStatesException("The two joint states are incompatible");
  // operation
  this->set_velocities(this->get_velocities() - velocities.get_velocities());
  return (*this);
}

const JointVelocities JointVelocities::operator-(const Eigen::VectorXd& vector) const {
  JointVelocities result(*this);
  result -= vector;
  return result;
}

const JointVelocities JointVelocities::operator-(const JointVelocities& velocities) const {
  JointVelocities result(*this);
  result -= velocities;
  return result;
}

const JointVelocities JointVelocities::copy() const {
  JointVelocities result(*this);
  return result;
}

const Eigen::ArrayXd JointVelocities::array() const {
  return this->get_velocities().array();
}

void JointVelocities::clamp(const std::vector<double>& max_values, const std::vector<double>& noise_ratio) {
  Eigen::VectorXd velocities(this->get_size());
  for (unsigned int i; i < this->get_size(); ++i) {
    // apply a deadzone
    if (!noise_ratio.empty() && abs(this->get_velocities()(i)) < noise_ratio[i]) velocities(i) = 0.0;
    // clamp the velocities to their maximum amplitude provided
    if (abs(this->get_velocities()(i)) > max_values[i]) velocities(i) = copysign(1.0, this->get_velocities()(i)) * max_values[i];
  }
  this->set_velocities(velocities);
}

void JointVelocities::clamp(double max_value, double noise_ratio) {
  this->clamp(std::vector<double>(this->get_size(), max_value), std::vector<double>(this->get_size(), noise_ratio));
}

const JointVelocities JointVelocities::clamped(double max_value, double noise_ratio) const {
  JointVelocities result(*this);
  result.clamp(max_value, noise_ratio);
  return result;
}

const JointVelocities JointVelocities::clamped(const std::vector<double>& max_values, const std::vector<double>& noise_ratio) const {
  JointVelocities result(*this);
  result.clamp(max_values, noise_ratio);
  return result;
}

std::ostream& operator<<(std::ostream& os, const JointVelocities& velocities) {
  if (velocities.is_empty()) {
    os << "Empty JointVelocities";
  } else {
    os << velocities.get_name() << " JointVelocities" << std::endl;
    os << "names: [";
    for (auto& n : velocities.get_names()) os << n << ", ";
    os << "]" << std::endl;
    os << "velocities: [";
    for (unsigned int i = 0; i < velocities.get_size(); ++i) os << velocities.get_velocities()(i) << ", ";
    os << "]";
  }
  return os;
}

const JointVelocities operator+(const Eigen::VectorXd& vector, const JointVelocities& velocities) {
  return velocities + vector;
}

const JointVelocities operator-(const Eigen::VectorXd& vector, const JointVelocities& velocities) {
  return vector + (-1) * velocities;
}

const JointVelocities operator*(double lambda, const JointVelocities& velocities) {
  if (velocities.is_empty()) throw EmptyStateException(velocities.get_name() + " state is empty");
  JointVelocities result(velocities);
  result.set_velocities(lambda * velocities.get_velocities());
  return result;
}

const JointVelocities operator*(const Eigen::ArrayXd& lambda, const JointVelocities& velocities) {
  if (velocities.is_empty()) throw EmptyStateException(velocities.get_name() + " state is empty");
  if (lambda.size() != velocities.get_size()) throw IncompatibleSizeException("Gain vector is of incorrect size");
  JointVelocities result(velocities);
  result.set_velocities(lambda * velocities.get_velocities().array());
  return result;
}

const JointPositions operator*(const std::chrono::nanoseconds& dt, const JointVelocities& velocities) {
  if (velocities.is_empty()) throw EmptyStateException(velocities.get_name() + " state is empty");
  // operations
  JointPositions displacement(velocities.get_name(), velocities.get_names());
  // convert the period to a double with the second as reference
  double period = dt.count();
  period /= 1e9;
  // multiply the velocities by this period value
  displacement.set_positions(period * velocities.get_velocities());
  return displacement;
}

const JointPositions operator*(const JointVelocities& velocities, const std::chrono::nanoseconds& dt) {
  return dt * velocities;
}

const JointVelocities operator/(const JointVelocities& velocities, double lambda) {
  if (velocities.is_empty()) throw EmptyStateException(velocities.get_name() + " state is empty");
  JointVelocities result(velocities);
  result.set_velocities(velocities.get_velocities() / lambda);
  return result;
}

const JointVelocities operator/(const JointVelocities& velocities, const Eigen::ArrayXd& lambda) {
  if (velocities.is_empty()) throw EmptyStateException(velocities.get_name() + " state is empty");
  if (lambda.size() != velocities.get_size()) throw IncompatibleSizeException("Gain vector is of incorrect size");
  JointVelocities result(velocities);
  result.set_velocities(velocities.get_velocities().array() / lambda);
  return result;
}
}// namespace StateRepresentation