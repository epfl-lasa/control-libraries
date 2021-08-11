#include "state_representation/robot/JointAccelerations.hpp"
#include "state_representation/exceptions/EmptyStateException.hpp"

using namespace state_representation::exceptions;

namespace state_representation {
JointAccelerations::JointAccelerations(const std::string& robot_name, unsigned int nb_joints) :
    JointState(robot_name, nb_joints) {}

JointAccelerations::JointAccelerations(const std::string& robot_name, const Eigen::VectorXd& accelerations) :
    JointState(robot_name, accelerations.size()) {
  this->set_accelerations(accelerations);
}

JointAccelerations::JointAccelerations(const std::string& robot_name, const std::vector<std::string>& joint_names) :
    JointState(robot_name, joint_names) {}

JointAccelerations::JointAccelerations(const std::string& robot_name,
                                       const std::vector<std::string>& joint_names,
                                       const Eigen::VectorXd& accelerations) : JointState(robot_name, joint_names) {
  this->set_accelerations(accelerations);
}

JointAccelerations::JointAccelerations(const JointState& state) : JointState(state) {
  // set all the state variables to 0 except accelerations
  this->set_zero();
  this->set_accelerations(state.get_accelerations());
  this->set_empty(state.is_empty());
}

JointAccelerations::JointAccelerations(const JointAccelerations& accelerations) :
    JointAccelerations(static_cast<const JointState&>(accelerations)) {}

JointAccelerations::JointAccelerations(const JointVelocities& velocities) :
    JointAccelerations(velocities / std::chrono::seconds(1)) {}

JointAccelerations JointAccelerations::Zero(const std::string& robot_name, unsigned int nb_joints) {
  return JointState::Zero(robot_name, nb_joints);
}

JointAccelerations
JointAccelerations::Zero(const std::string& robot_name, const std::vector<std::string>& joint_names) {
  return JointState::Zero(robot_name, joint_names);
}

JointAccelerations JointAccelerations::Random(const std::string& robot_name, unsigned int nb_joints) {
  return JointAccelerations(robot_name, Eigen::VectorXd::Random(nb_joints));
}

JointAccelerations
JointAccelerations::Random(const std::string& robot_name, const std::vector<std::string>& joint_names) {
  return JointAccelerations(robot_name, joint_names, Eigen::VectorXd::Random(joint_names.size()));
}

JointAccelerations& JointAccelerations::operator+=(const JointAccelerations& accelerations) {
  this->JointState::operator+=(accelerations);
  return (*this);
}

JointAccelerations JointAccelerations::operator+(const JointAccelerations& accelerations) const {
  return this->JointState::operator+(accelerations);
}

JointAccelerations& JointAccelerations::operator-=(const JointAccelerations& accelerations) {
  this->JointState::operator-=(accelerations);
  return (*this);
}

JointAccelerations JointAccelerations::operator-(const JointAccelerations& accelerations) const {
  return this->JointState::operator-(accelerations);
}

JointAccelerations& JointAccelerations::operator*=(double lambda) {
  this->JointState::operator*=(lambda);
  return (*this);
}

JointAccelerations JointAccelerations::operator*(double lambda) const {
  return this->JointState::operator*(lambda);
}

JointAccelerations& JointAccelerations::operator*=(const Eigen::ArrayXd& lambda) {
  this->multiply_state_variable(lambda, JointStateVariable::ACCELERATIONS);
  return (*this);
}

JointAccelerations JointAccelerations::operator*(const Eigen::ArrayXd& lambda) const {
  JointAccelerations result(*this);
  result *= lambda;
  return result;
}

JointAccelerations& JointAccelerations::operator*=(const Eigen::MatrixXd& lambda) {
  this->multiply_state_variable(lambda, JointStateVariable::ACCELERATIONS);
  return (*this);
}

JointAccelerations JointAccelerations::operator*(const Eigen::MatrixXd& lambda) const {
  JointAccelerations result(*this);
  result *= lambda;
  return result;
}

JointAccelerations& JointAccelerations::operator/=(double lambda) {
  this->JointState::operator/=(lambda);
  return (*this);
}

JointAccelerations JointAccelerations::operator/(double lambda) const {
  return this->JointState::operator/(lambda);
}

JointVelocities JointAccelerations::operator*(const std::chrono::nanoseconds& dt) const {
  if (this->is_empty()) { throw EmptyStateException(this->get_name() + " state is empty"); }
  // operations
  JointVelocities velocities(this->get_name(), this->get_names());
  // convert the period to a double with the second as reference
  double period = dt.count();
  period /= 1e9;
  // multiply the velocities by this period value and assign it as position
  velocities.set_velocities(period * this->get_accelerations());
  return velocities;
}

JointAccelerations JointAccelerations::copy() const {
  JointAccelerations result(*this);
  return result;
}

Eigen::VectorXd JointAccelerations::data() const {
  return this->get_accelerations();
}

void JointAccelerations::set_data(const Eigen::VectorXd& data) {
  this->set_accelerations(data);
}

void JointAccelerations::set_data(const std::vector<double>& data) {
  this->set_accelerations(Eigen::VectorXd::Map(data.data(), data.size()));
}

void JointAccelerations::clamp(double max_absolute_value, double noise_ratio) {
  this->clamp_state_variable(max_absolute_value, JointStateVariable::ACCELERATIONS, noise_ratio);
}

JointAccelerations JointAccelerations::clamped(double max_absolute_value, double noise_ratio) const {
  JointAccelerations result(*this);
  result.clamp(max_absolute_value, noise_ratio);
  return result;
}

void
JointAccelerations::clamp(const Eigen::ArrayXd& max_absolute_value_array, const Eigen::ArrayXd& noise_ratio_array) {
  this->clamp_state_variable(max_absolute_value_array, JointStateVariable::ACCELERATIONS, noise_ratio_array);
}

JointAccelerations JointAccelerations::clamped(const Eigen::ArrayXd& max_absolute_value_array,
                                               const Eigen::ArrayXd& noise_ratio_array) const {
  JointAccelerations result(*this);
  result.clamp(max_absolute_value_array, noise_ratio_array);
  return result;
}

std::ostream& operator<<(std::ostream& os, const JointAccelerations& accelerations) {
  if (accelerations.is_empty()) {
    os << "Empty JointAccelerations";
  } else {
    os << accelerations.get_name() << " JointAccelerations" << std::endl;
    os << "names: [";
    for (auto& n : accelerations.get_names()) { os << n << ", "; }
    os << "]" << std::endl;
    os << "accelerations: [";
    for (unsigned int i = 0; i < accelerations.get_size(); ++i) { os << accelerations.get_accelerations()(i) << ", "; }
    os << "]";
  }
  return os;
}

JointAccelerations operator*(double lambda, const JointAccelerations& accelerations) {
  JointAccelerations result(accelerations);
  result *= lambda;
  return result;
}

JointAccelerations operator*(const Eigen::ArrayXd& lambda, const JointAccelerations& accelerations) {
  JointAccelerations result(accelerations);
  result *= lambda;
  return result;
}

JointAccelerations operator*(const Eigen::MatrixXd& lambda, const JointAccelerations& accelerations) {
  JointAccelerations result(accelerations);
  result *= lambda;
  return result;
}

JointVelocities operator*(const std::chrono::nanoseconds& dt, const JointAccelerations& accelerations) {
  return accelerations * dt;
}
}// namespace state_representation