#include "state_representation/Robot/JointVelocities.hpp"
#include "state_representation/Exceptions/EmptyStateException.hpp"
#include "state_representation/Exceptions/IncompatibleStatesException.hpp"

using namespace StateRepresentation::Exceptions;

namespace StateRepresentation {
JointVelocities::JointVelocities() {}

JointVelocities::JointVelocities(const std::string& robot_name, unsigned int nb_joints) :
    JointState(robot_name, nb_joints) {}

JointVelocities::JointVelocities(const std::string& robot_name, const Eigen::VectorXd& velocities) :
    JointState(robot_name, velocities.size()) {
  this->set_velocities(velocities);
}

JointVelocities::JointVelocities(const std::string& robot_name, const std::vector<std::string>& joint_names) :
    JointState(robot_name, joint_names) {}

JointVelocities::JointVelocities(const std::string& robot_name, const std::vector<std::string>& joint_names,
                                 const Eigen::VectorXd& velocities) : JointState(robot_name, joint_names) {
  this->set_velocities(velocities);
}

JointVelocities::JointVelocities(const JointVelocities& velocities) : JointState(velocities) {}

JointVelocities::JointVelocities(const JointState& state) : JointState(state) {}

JointVelocities::JointVelocities(const JointPositions& positions) : JointState(positions / std::chrono::seconds(1)) {}

JointVelocities& JointVelocities::operator+=(const JointVelocities& velocities) {
  this->JointState::operator+=(velocities);
  return (*this);
}

JointVelocities JointVelocities::operator+(const JointVelocities& velocities) const {
  return this->JointState::operator+(velocities);
}

JointVelocities& JointVelocities::operator-=(const JointVelocities& velocities) {
  this->JointState::operator-=(velocities);
  return (*this);
}

JointVelocities JointVelocities::operator-(const JointVelocities& velocities) const {
  return this->JointState::operator-(velocities);
}

JointVelocities& JointVelocities::operator*=(double lambda) {
  this->JointState::operator*=(lambda);
  return (*this);
}

JointVelocities JointVelocities::operator*(double lambda) const {
  return this->JointState::operator*(lambda);
}

JointVelocities& JointVelocities::operator*=(const Eigen::ArrayXd& lambda) {
  this->multiply_state_variable(lambda, JointStateVariable::TORQUES);
  return (*this);
}

JointVelocities JointVelocities::operator*(const Eigen::ArrayXd& lambda) const {
  JointVelocities result(*this);
  result *= lambda;
  return result;
}

JointVelocities& JointVelocities::operator*=(const Eigen::MatrixXd& lambda) {
  this->multiply_state_variable(lambda, JointStateVariable::TORQUES);
  return (*this);
}

JointVelocities JointVelocities::operator*(const Eigen::MatrixXd& lambda) const {
  JointVelocities result(*this);
  result *= lambda;
  return result;
}

JointVelocities& JointVelocities::operator/=(double lambda) {
  this->JointState::operator/=(lambda);
  return (*this);
}

JointVelocities JointVelocities::operator/(double lambda) const {
  return this->JointState::operator/(lambda);
}

JointPositions JointVelocities::operator*(const std::chrono::nanoseconds& dt) const {
  if (this->is_empty()) { throw EmptyStateException(this->get_name() + " state is empty"); }
  // operations
  JointPositions displacement(this->get_name(), this->get_names());
  // convert the period to a double with the second as reference
  double period = dt.count();
  period /= 1e9;
  // multiply the positions by this period value
  displacement *= period;
  return displacement;
}

JointVelocities JointVelocities::copy() const {
  JointVelocities result(*this);
  return result;
}

Eigen::VectorXd JointVelocities::data() const {
  return this->get_velocities();
}

void JointVelocities::clamp(double max_absolute_value, double noise_ratio) {
  this->clamp_state_variable(max_absolute_value, JointStateVariable::VELOCITIES, noise_ratio);
}

JointVelocities JointVelocities::clamped(double max_absolute_value, double noise_ratio) const {
  JointVelocities result(*this);
  result.clamp(max_absolute_value, noise_ratio);
  return result;
}

void JointVelocities::clamp(const Eigen::ArrayXd& max_absolute_value_array, const Eigen::ArrayXd& noise_ratio_array) {
  this->clamp_state_variable(max_absolute_value_array, JointStateVariable::VELOCITIES, noise_ratio_array);
}

JointVelocities JointVelocities::clamped(const Eigen::ArrayXd& max_absolute_value_array,
                                         const Eigen::ArrayXd& noise_ratio_array) const {
  JointVelocities result(*this);
  result.clamp(max_absolute_value_array, noise_ratio_array);
  return result;
}

std::ostream& operator<<(std::ostream& os, const JointVelocities& velocities) {
  if (velocities.is_empty()) {
    os << "Empty JointVelocities";
  } else {
    os << velocities.get_name() << " JointVelocities" << std::endl;
    os << "names: [";
    for (auto& n : velocities.get_names()) { os << n << ", "; }
    os << "]" << std::endl;
    os << "velocities: [";
    for (unsigned int i = 0; i < velocities.get_size(); ++i) { os << velocities.get_velocities()(i) << ", "; }
    os << "]";
  }
  return os;
}

JointVelocities operator*(double lambda, const JointVelocities& velocities) {
  JointVelocities result(velocities);
  result *= lambda;
  return result;
}

JointVelocities operator*(const Eigen::ArrayXd& lambda, const JointVelocities& velocities) {
  JointVelocities result(velocities);
  result *= lambda;
  return result;
}

JointVelocities operator*(const Eigen::MatrixXd& lambda, const JointVelocities& velocities) {
  JointVelocities result(velocities);
  result *= lambda;
  return result;
}

JointPositions operator*(const std::chrono::nanoseconds& dt, const JointVelocities& velocities) {
  return velocities * dt;
}
}// namespace StateRepresentation