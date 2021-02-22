#include "state_representation/Robot/JointPositions.hpp"
#include "state_representation/Exceptions/EmptyStateException.hpp"
#include "state_representation/Exceptions/IncompatibleStatesException.hpp"

using namespace StateRepresentation::Exceptions;

namespace StateRepresentation {
JointPositions::JointPositions() {}

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

JointPositions::JointPositions(const JointPositions& positions) : JointState(positions) {}

JointPositions::JointPositions(const JointState& state) : JointState(state) {}

JointPositions::JointPositions(const JointVelocities& velocities) : JointState(std::chrono::seconds(1) * velocities) {}

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
  // multiply the positions by this period value and assign it as velocity
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

std::vector<double> JointPositions::to_std_vector() const {
  std::vector<double> temp(this->get_positions().data(), this->get_positions().data() + this->get_size());
  return temp;
}

void JointPositions::from_std_vector(const std::vector<double>& value) {
  this->set_positions(value);
}
}// namespace StateRepresentation