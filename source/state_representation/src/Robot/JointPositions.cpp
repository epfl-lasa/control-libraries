#include "state_representation/Robot/JointPositions.hpp"
#include "state_representation/Exceptions/EmptyStateException.hpp"
#include "state_representation/Exceptions/IncompatibleStatesException.hpp"

using namespace StateRepresentation::Exceptions;

namespace StateRepresentation {
JointPositions::JointPositions() {}

JointPositions::JointPositions(const std::string& robot_name, unsigned int nb_joints) : JointState(robot_name, nb_joints) {}

JointPositions::JointPositions(const std::string& robot_name, const Eigen::VectorXd& positions) : JointState(robot_name, positions.size()) {
  this->set_positions(positions);
}

JointPositions::JointPositions(const std::string& robot_name, const std::vector<std::string>& joint_names) : JointState(robot_name, joint_names) {}

JointPositions::JointPositions(const std::string& robot_name, const std::vector<std::string>& joint_names, const Eigen::VectorXd& positions) : JointState(robot_name, joint_names) {
  this->set_positions(positions);
}

JointPositions::JointPositions(const JointPositions& positions) : JointState(positions) {}

JointPositions::JointPositions(const JointState& state) : JointState(state) {}

JointPositions::JointPositions(const JointVelocities& velocities) : JointState(std::chrono::seconds(1) * velocities) {}

JointPositions& JointPositions::operator=(const Eigen::VectorXd& positions) {
  this->set_positions(positions);
  return (*this);
}

JointPositions& JointPositions::operator+=(const Eigen::VectorXd& vector) {
  if (this->is_empty()) throw EmptyStateException(this->get_name() + " state is empty");
  if (this->get_size() != vector.size()) throw IncompatibleSizeException("Input vector is of incorrect size: expected " + std::to_string(this->get_size()) + ", given " + std::to_string(vector.size()));
  this->set_positions(this->get_positions() + vector);
  return (*this);
}

JointPositions& JointPositions::operator+=(const JointPositions& positions) {
  // sanity check
  if (this->is_empty()) throw EmptyStateException(this->get_name() + " state is empty");
  if (positions.is_empty()) throw EmptyStateException(positions.get_name() + " state is empty");
  if (!this->is_compatible(positions)) throw IncompatibleStatesException("The two joint states are incompatible, check name, joint names and order or size");
  // operation
  this->set_positions(this->get_positions() + positions.get_positions());
  return (*this);
}

const JointPositions JointPositions::operator+(const Eigen::VectorXd& vector) const {
  JointPositions result(*this);
  result += vector;
  return result;
}

const JointPositions JointPositions::operator+(const JointPositions& positions) const {
  JointPositions result(*this);
  result += positions;
  return result;
}

JointPositions& JointPositions::operator-=(const Eigen::VectorXd& vector) {
  if (this->is_empty()) throw EmptyStateException(this->get_name() + " state is empty");
  if (this->get_size() != vector.size()) throw IncompatibleSizeException("Input vector is of incorrect size: expected " + std::to_string(this->get_size()) + ", given " + std::to_string(vector.size()));
  this->set_positions(this->get_positions() - vector);
  return (*this);
}

JointPositions& JointPositions::operator-=(const JointPositions& positions) {
  // sanity check
  if (this->is_empty()) throw EmptyStateException(this->get_name() + " state is empty");
  if (positions.is_empty()) throw EmptyStateException(positions.get_name() + " state is empty");
  if (!this->is_compatible(positions)) throw IncompatibleStatesException("The two joint states are incompatible, check name, joint names and order or size");
  // operation
  this->set_positions(this->get_positions() - positions.get_positions());
  return (*this);
}

const JointPositions JointPositions::operator-(const Eigen::VectorXd& vector) const {
  JointPositions result(*this);
  result -= vector;
  return result;
}

const JointPositions JointPositions::operator-(const JointPositions& positions) const {
  JointPositions result(*this);
  result -= positions;
  return result;
}

const JointPositions JointPositions::copy() const {
  JointPositions result(*this);
  return result;
}

const Eigen::ArrayXd JointPositions::array() const {
  return this->get_positions().array();
}

std::ostream& operator<<(std::ostream& os, const JointPositions& positions) {
  if (positions.is_empty()) {
    os << "Empty JointPositions";
  } else {
    os << positions.get_name() << " JointPositions" << std::endl;
    os << "names: [";
    for (auto& n : positions.get_names()) os << n << ", ";
    os << "]" << std::endl;
    os << "positions: [";
    for (unsigned int i = 0; i < positions.get_size(); ++i) os << positions.get_positions()(i) << ", ";
    os << "]";
  }
  return os;
}

const JointPositions operator+(const Eigen::VectorXd& vector, const JointPositions& positions) {
  return positions + vector;
}

const JointPositions operator-(const Eigen::VectorXd& vector, const JointPositions& positions) {
  return vector + (-1) * positions;
}

const JointPositions operator*(double lambda, const JointPositions& positions) {
  if (positions.is_empty()) throw EmptyStateException(positions.get_name() + " state is empty");
  JointPositions result(positions);
  result.set_positions(lambda * positions.get_positions());
  return result;
}

const JointPositions operator*(const Eigen::ArrayXd& lambda, const JointPositions& positions) {
  if (positions.is_empty()) throw EmptyStateException(positions.get_name() + " state is empty");
  if (lambda.size() != positions.get_size()) throw IncompatibleSizeException("Gain vector is of incorrect size: expected " + std::to_string(positions.get_size()) + ", given " + std::to_string(lambda.size()));
  JointPositions result(positions);
  result.set_positions(lambda * positions.get_positions().array());
  return result;
}

const JointPositions operator/(const JointPositions& positions, double lambda) {
  if (positions.is_empty()) throw EmptyStateException(positions.get_name() + " state is empty");
  JointPositions result(positions);
  result.set_positions(positions.get_positions() / lambda);
  return result;
}

const JointPositions operator/(const JointPositions& positions, const Eigen::ArrayXd& lambda) {
  if (positions.is_empty()) throw EmptyStateException(positions.get_name() + " state is empty");
  if (lambda.size() != positions.get_size()) throw IncompatibleSizeException("Gain vector is of incorrect size: expected " + std::to_string(positions.get_size()) + +", given " + std::to_string(lambda.size()));
  JointPositions result(positions);
  result.set_positions(positions.get_positions().array() / lambda);
  return result;
}

const JointVelocities operator/(const JointPositions& positions, const std::chrono::nanoseconds& dt) {
  if (positions.is_empty()) throw EmptyStateException(positions.get_name() + " state is empty");
  // operations
  JointVelocities velocities(positions.get_name(), positions.get_names());
  // convert the period to a double with the second as reference
  double period = dt.count();
  period /= 1e9;
  // multiply the positions by this period value
  velocities.set_velocities(positions.get_positions() / period);
  return velocities;
}

const std::vector<double> JointPositions::to_std_vector() const {
  std::vector<double> temp(this->get_positions().data(), this->get_positions().data() + this->get_size());
  return temp;
}

void JointPositions::from_std_vector(const std::vector<double>& value) {
  this->set_positions(value);
}
}// namespace StateRepresentation