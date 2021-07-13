#include "state_representation/robot/JointPositions.hpp"

#include "state_representation/exceptions/EmptyStateException.hpp"

using namespace state_representation::exceptions;

namespace state_representation {
JointPositions::JointPositions(const std::string& robot_name, unsigned int nb_joints) :
    JointState(robot_name, nb_joints) {}

JointPositions::JointPositions(const std::string& robot_name, const Eigen::VectorXd& positions) :
    JointState(robot_name, positions.size()) {
  this->set_positions(positions);
}

JointPositions::JointPositions(const std::string& robot_name, const std::vector<std::string>& joint_names) :
    JointState(robot_name, joint_names) {}

JointPositions::JointPositions(const std::string& robot_name, const std::vector<std::string>& joint_names,
                               const Eigen::VectorXd& positions) : JointState(robot_name, joint_names) {
  this->set_positions(positions);
}

JointPositions::JointPositions(const JointState& state) : JointState(state) {
  // set all the state variables to 0 except positions
  this->set_zero();
  this->set_positions(state.get_positions());
  this->set_empty(state.is_empty());
}

JointPositions::JointPositions(const JointPositions& positions) : JointPositions(static_cast<const JointState&>(positions)) {}

JointPositions::JointPositions(const JointVelocities& velocities) : JointPositions(std::chrono::seconds(1) * velocities) {}

JointPositions JointPositions::Zero(const std::string& robot_name, unsigned int nb_joints) {
  return JointState::Zero(robot_name, nb_joints);
}

JointPositions JointPositions::Zero(const std::string& robot_name, const std::vector<std::string>& joint_names) {
  return JointState::Zero(robot_name, joint_names);
}

JointPositions JointPositions::Random(const std::string& robot_name, unsigned int nb_joints) {
  return JointPositions(robot_name, Eigen::VectorXd::Random(nb_joints));
}

JointPositions JointPositions::Random(const std::string& robot_name, const std::vector<std::string>& joint_names) {
  return JointPositions(robot_name, joint_names, Eigen::VectorXd::Random(joint_names.size()));
}

JointPositions& JointPositions::operator+=(const JointPositions& positions) {
  this->JointState::operator+=(positions);
  return (*this);
}

JointPositions JointPositions::operator+(const JointPositions& positions) const {
  return this->JointState::operator+(positions);
}

JointPositions& JointPositions::operator-=(const JointPositions& positions) {
  this->JointState::operator-=(positions);
  return (*this);
}

JointPositions JointPositions::operator-(const JointPositions& positions) const {
  return this->JointState::operator-(positions);
}

JointPositions& JointPositions::operator*=(double lambda) {
  this->JointState::operator*=(lambda);
  return (*this);
}

JointPositions JointPositions::operator*(double lambda) const {
  return this->JointState::operator*(lambda);
}

JointPositions& JointPositions::operator*=(const Eigen::ArrayXd& lambda) {
  this->multiply_state_variable(lambda, JointStateVariable::POSITIONS);
  return (*this);
}

JointPositions JointPositions::operator*(const Eigen::ArrayXd& lambda) const {
  JointPositions result(*this);
  result *= lambda;
  return result;
}

JointPositions& JointPositions::operator*=(const Eigen::MatrixXd& lambda) {
  this->multiply_state_variable(lambda, JointStateVariable::POSITIONS);
  return (*this);
}

JointPositions JointPositions::operator*(const Eigen::MatrixXd& lambda) const {
  JointPositions result(*this);
  result *= lambda;
  return result;
}

JointPositions& JointPositions::operator/=(double lambda) {
  this->JointState::operator/=(lambda);
  return (*this);
}

JointPositions JointPositions::operator/(double lambda) const {
  return this->JointState::operator/(lambda);
}

JointVelocities JointPositions::operator/(const std::chrono::nanoseconds& dt) const {
  if (this->is_empty()) { throw EmptyStateException(this->get_name() + " state is empty"); }
  // operations
  JointVelocities velocities(this->get_name(), this->get_names());
  // convert the period to a double with the second as reference
  double period = dt.count();
  period /= 1e9;
  // divide the positions by this period value and assign it as velocities
  velocities.set_velocities(this->get_positions() / period);
  return velocities;
}

JointPositions JointPositions::copy() const {
  JointPositions result(*this);
  return result;
}

Eigen::VectorXd JointPositions::data() const {
  return this->get_positions();
}

void JointPositions::set_data(const Eigen::VectorXd& data) {
  this->set_positions(data);
}

void JointPositions::set_data(const std::vector<double>& data) {
  this->set_positions(Eigen::VectorXd::Map(data.data(), data.size()));
}

std::ostream& operator<<(std::ostream& os, const JointPositions& positions) {
  if (positions.is_empty()) {
    os << "Empty JointPositions";
  } else {
    os << positions.get_name() << " JointPositions" << std::endl;
    os << "names: [";
    for (auto& n : positions.get_names()) { os << n << ", "; }
    os << "]" << std::endl;
    os << "positions: [";
    for (unsigned int i = 0; i < positions.get_size(); ++i) { os << positions.get_positions()(i) << ", "; }
    os << "]";
  }
  return os;
}

JointPositions operator*(double lambda, const JointPositions& positions) {
  JointPositions result(positions);
  result *= lambda;
  return result;
}

JointPositions operator*(const Eigen::ArrayXd& lambda, const JointPositions& positions) {
  JointPositions result(positions);
  result *= lambda;
  return result;
}

JointPositions operator*(const Eigen::MatrixXd& lambda, const JointPositions& positions) {
  JointPositions result(positions);
  result *= lambda;
  return result;
}

void JointPositions::from_std_vector(const std::vector<double>& value) {
  this->set_positions(value);
}
}// namespace state_representation