#include "state_representation/Robot/JointState.hpp"
#include "state_representation/Exceptions/EmptyStateException.hpp"
#include "state_representation/Exceptions/IncompatibleStatesException.hpp"
#include "state_representation/Exceptions/NotImplementedException.hpp"

namespace StateRepresentation {
JointState::JointState() : State(StateType::JOINTSTATE) {
  this->initialize();
}

JointState::JointState(const std::string& robot_name, unsigned int nb_joints) : State(StateType::JOINTSTATE, robot_name), names(nb_joints) {
  this->set_names(nb_joints);
}

JointState::JointState(const std::string& robot_name, const std::vector<std::string>& joint_names) : State(StateType::JOINTSTATE, robot_name) {
  this->set_names(joint_names);
}

JointState::JointState(const JointState& state) : State(state), names(state.names), positions(state.positions), velocities(state.velocities),
                                                  accelerations(state.accelerations), torques(state.torques) {}

void JointState::initialize() {
  this->State::initialize();
  // resize
  unsigned int size = this->names.size();
  this->positions.resize(size);
  this->velocities.resize(size);
  this->accelerations.resize(size);
  this->torques.resize(size);
  // set to zeros
  this->set_zero();
}

void JointState::set_zero() {
  this->positions.setZero();
  this->velocities.setZero();
  this->accelerations.setZero();
  this->torques.setZero();
}

JointState& JointState::operator+=(const JointState& state) {
  if (!this->is_compatible(state)) throw IncompatibleStatesException("The two joint states are incompatible, check name, joint names and order or size");
  this->set_all_state_variables(this->get_all_state_variables() + state.get_all_state_variables());
  return (*this);
}

JointState JointState::operator+(const JointState& state) const {
  JointState result(*this);
  result += state;
  return result;
}

JointState& JointState::operator-=(const JointState& state) {
  if (!this->is_compatible(state)) throw IncompatibleStatesException("The two joint states are incompatible, check name, joint names and order or size");
  this->set_all_state_variables(this->get_all_state_variables() - state.get_all_state_variables());
  return (*this);
}

JointState JointState::operator-(const JointState& state) const {
  JointState result(*this);
  result -= state;
  return result;
}

JointState& JointState::operator*=(double lambda) {
  if (this->is_empty()) throw EmptyStateException(this->get_name() + " state is empty");
  this->set_all_state_variables(lambda * this->get_all_state_variables());
  return (*this);
}

JointState& JointState::operator*=(const Eigen::MatrixXd& lambda) {
  if (this->is_empty()) throw EmptyStateException(this->get_name() + " state is empty");
  // the size of the matrix should correspond to the number of joints times each features
  // (positions, velocities, accelerations and torques, i.e 4)
  int expected_size = this->get_size() * 4;
  if (lambda.rows() != expected_size || lambda.cols() != expected_size) throw IncompatibleSizeException("Gain matrix is of incorrect size: expected "
                                                                                                        + std::to_string(expected_size) + "x" + std::to_string(expected_size)
                                                                                                        + ", given " + std::to_string(lambda.rows()) + "x" + std::to_string(lambda.cols()));
  this->set_all_state_variables(lambda * this->get_all_state_variables());
  return (*this);
}

JointState& JointState::operator*=(const Eigen::ArrayXd& lambda) {
  int expected_size = this->get_size() * 4;
  if (lambda.size() != expected_size) throw IncompatibleSizeException("Gain matrix is of incorrect size: expected "
                                                                      + std::to_string(expected_size)
                                                                      + ", given " + std::to_string(lambda.size()));
  // transform the array of gain to a diagonal matrix and use the appropriate operator
  this->set_all_state_variables((lambda * this->get_all_state_variables().array()).matrix());
  return (*this);
}

JointState JointState::operator*(double lambda) const {
  JointState result(*this);
  result *= lambda;
  return result;
}

JointState JointState::operator*(const Eigen::MatrixXd& lambda) const {
  JointState result(*this);
  result *= lambda;
  return result;
}

JointState JointState::operator*(const Eigen::ArrayXd& lambda) const {
  JointState result(*this);
  result *= lambda;
  return result;
}

JointState& JointState::operator/=(double lambda) {
  return JointState::operator*=(1 / lambda);
}

JointState JointState::operator/(double lambda) const {
  JointState result(*this);
  result /= lambda;
  return result;
}

JointState JointState::copy() const {
  JointState result(*this);
  return result;
}

void JointState::clamp_state_variable(double max_value, const JointStateVariable& state_variable_type, double noise_ratio) {
  Eigen::VectorXd state_variable_value = this->get_state_variable(state_variable_type);
  if (noise_ratio != 0) {
    state_variable_value -= noise_ratio * state_variable_value.normalized();
    // apply a deadzone
    if (state_variable_value.norm() < noise_ratio) state_variable_value.setZero();
  }
  // clamp the values to their maximum amplitude provided
  if (state_variable_value.norm() > max_value) state_variable_value = max_value * state_variable_value.normalized();
  this->set_state_variable(state_variable_value, state_variable_type);
}

double JointState::dist(const JointState& state, const JointStateVariable& state_variable_type) const {
  // sanity check
  if (this->is_empty()) throw EmptyStateException(this->get_name() + " state is empty");
  if (state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty");
  if (!this->is_compatible(state)) throw IncompatibleStatesException("The two joint states are incompatible, check name, joint names and order or size");
  // calculation
  double result = 0;
  if (state_variable_type == JointStateVariable::POSITIONS || state_variable_type == JointStateVariable::ALL) {
    result += (this->get_positions() - state.get_positions()).norm();
  }
  if (state_variable_type == JointStateVariable::VELOCITIES || state_variable_type == JointStateVariable::ALL) {
    result += (this->get_velocities() - state.get_velocities()).norm();
  }
  if (state_variable_type == JointStateVariable::ACCELERATIONS || state_variable_type == JointStateVariable::ALL) {
    result += (this->get_accelerations() - state.get_accelerations()).norm();
  }
  if (state_variable_type == JointStateVariable::TORQUES || state_variable_type == JointStateVariable::ALL) {
    result += (this->get_torques() - state.get_torques()).norm();
  }
  return result;
}

std::ostream& operator<<(std::ostream& os, const JointState& state) {
  if (state.is_empty()) {
    os << "Empty " << state.get_name() << " JointState";
  } else {
    os << state.get_name() << " JointState" << std::endl;
    os << "names: [";
    for (auto& n : state.names) os << n << ", ";
    os << "]" << std::endl;
    os << "positions: [";
    for (unsigned int i = 0; i < state.positions.size(); ++i) os << state.positions(i) << ", ";
    os << "]" << std::endl;
    os << "velocities: [";
    for (unsigned int i = 0; i < state.velocities.size(); ++i) os << state.velocities(i) << ", ";
    os << "]" << std::endl;
    os << "accelerations: [";
    for (unsigned int i = 0; i < state.accelerations.size(); ++i) os << state.accelerations(i) << ", ";
    os << "]" << std::endl;
    os << "torques: [";
    for (unsigned int i = 0; i < state.torques.size(); ++i) os << state.torques(i) << ", ";
    os << "]";
  }
  return os;
}

double dist(const JointState& s1, const JointState& s2, const JointStateVariable& state_variable_type) {
  return s1.dist(s2, state_variable_type);
}

JointState operator*(double lambda, const JointState& state) {
  if (state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty");
  JointState result(state);
  result *= lambda;
  return result;
}

JointState operator*(const Eigen::MatrixXd& lambda, const JointState& state) {
  JointState result(state);
  result *= lambda;
  return result;
}

JointState operator*(const Eigen::ArrayXd& lambda, const JointState& state) {
  JointState result(state);
  result *= lambda;
  return result;
}

std::vector<double> JointState::to_std_vector() const {
  throw(NotImplementedException("to_std_vector() is not implemented for the base JointState class"));
  return std::vector<double>();
}

void JointState::from_std_vector(const std::vector<double>&) {
  throw(NotImplementedException("from_std_vector() is not implemented for the base JointState class"));
}
}// namespace StateRepresentation