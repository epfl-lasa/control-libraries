#include "state_representation/Robot/JointTorques.hpp"
#include "state_representation/Exceptions/EmptyStateException.hpp"
#include "state_representation/Exceptions/IncompatibleStatesException.hpp"

using namespace StateRepresentation::Exceptions;

namespace StateRepresentation {
JointTorques::JointTorques() {}

JointTorques::JointTorques(const std::string& robot_name, unsigned int nb_joints) : JointState(robot_name, nb_joints) {}

JointTorques::JointTorques(const std::string& robot_name, const Eigen::VectorXd& torques) : JointState(robot_name, torques.size()) {
  this->set_torques(torques);
}

JointTorques::JointTorques(const std::string& robot_name, const std::vector<std::string>& joint_names) : JointState(robot_name, joint_names) {}

JointTorques::JointTorques(const std::string& robot_name, const std::vector<std::string>& joint_names, const Eigen::VectorXd& torques) : JointState(robot_name, joint_names) {
  this->set_torques(torques);
}

JointTorques::JointTorques(const JointTorques& torques) : JointState(torques) {}

JointTorques::JointTorques(const JointState& state) : JointState(state) {}

JointTorques& JointTorques::operator+=(const JointTorques& torques) {
  this->JointState::operator+=(torques);
  return (*this);
}

JointTorques JointTorques::operator+(const JointTorques& torques) const {
  return this->JointState::operator+(torques);
}

JointTorques& JointTorques::operator-=(const JointTorques& torques) {
  this->JointState::operator-=(torques);
  return (*this);
}

JointTorques JointTorques::operator-(const JointTorques& torques) const {
  return this->JointState::operator-(torques);
}

JointTorques& JointTorques::operator*=(double lambda) {
  this->JointState::operator*=(lambda);
  return (*this);
}

JointTorques JointTorques::operator*(double lambda) const {
  return this->JointState::operator*(lambda);
}

JointTorques& JointTorques::operator*=(const Eigen::ArrayXd& lambda) {
  this->multiply_state_variable(lambda, JointStateVariable::TORQUES);
  return (*this);
}

JointTorques JointTorques::operator*(const Eigen::ArrayXd& lambda) const {
  JointTorques result(*this);
  result *= lambda;
  return result;
}

JointTorques& JointTorques::operator*=(const Eigen::MatrixXd& lambda) {
  this->multiply_state_variable(lambda, JointStateVariable::TORQUES);
  return (*this);
}

JointTorques JointTorques::operator*(const Eigen::MatrixXd& lambda) const {
  JointTorques result(*this);
  result *= lambda;
  return result;
}

JointTorques& JointTorques::operator/=(double lambda) {
  this->JointState::operator/=(lambda);
  return (*this);
}

JointTorques JointTorques::operator/(double lambda) const {
  return this->JointState::operator/(lambda);
}

JointTorques JointTorques::copy() const {
  JointTorques result(*this);
  return result;
}

Eigen::ArrayXd JointTorques::array() const {
  return this->get_torques().array();
}

void JointTorques::clamp(double max_absolute_value, double noise_ratio) {
  this->clamp_state_variable(max_absolute_value, JointStateVariable::TORQUES, noise_ratio);
}

JointTorques JointTorques::clamped(double max_absolute_value, double noise_ratio) const {
  JointTorques result(*this);
  result.clamp(max_absolute_value, noise_ratio);
  return result;
}

void JointTorques::clamp(const Eigen::ArrayXd& max_absolute_value_array, const Eigen::ArrayXd& noise_ratio_array) {
  this->clamp_state_variable(max_absolute_value_array, JointStateVariable::TORQUES, noise_ratio_array);
}

JointTorques JointTorques::clamped(const Eigen::ArrayXd& max_absolute_value_array, const Eigen::ArrayXd& noise_ratio_array) const {
  JointTorques result(*this);
  result.clamp(max_absolute_value_array, noise_ratio_array);
  return result;
}

std::ostream& operator<<(std::ostream& os, const JointTorques& torques) {
  if (torques.is_empty()) {
    os << "Empty JointTorques";
  } else {
    os << torques.get_name() << " JointTorques" << std::endl;
    os << "names: [";
    for (auto& n : torques.get_names()) os << n << ", ";
    os << "]" << std::endl;
    os << "torques: [";
    for (unsigned int i = 0; i < torques.get_size(); ++i) os << torques.get_torques()(i) << ", ";
    os << "]";
  }
  return os;
}

JointTorques operator*(double lambda, const JointTorques& torques) {
  JointTorques result(torques);
  result *= lambda;
  return result;
}

JointTorques operator*(const Eigen::ArrayXd& lambda, const JointTorques& torques) {
  JointTorques result(torques);
  result *= lambda;
  return result;
}

JointTorques operator*(const Eigen::MatrixXd& lambda, const JointTorques& torques) {
  JointTorques result(torques);
  result *= lambda;
  return result;
}
}// namespace StateRepresentation