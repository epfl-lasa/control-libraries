#include "state_representation/Space/Cartesian/CartesianWrench.hpp"
#include "state_representation/Exceptions/EmptyStateException.hpp"
#include "state_representation/Exceptions/IncompatibleReferenceFramesException.hpp"
#include "state_representation/Exceptions/IncompatibleStatesException.hpp"

using namespace StateRepresentation::Exceptions;

namespace StateRepresentation {
CartesianWrench::CartesianWrench() {}

CartesianWrench::CartesianWrench(const std::string& name, const std::string& reference) : CartesianState(name, reference) {}

CartesianWrench::CartesianWrench(const std::string& name, const Eigen::Vector3d& force, const std::string& reference) : CartesianState(name, reference) {
  this->set_force(force);
}

CartesianWrench::CartesianWrench(const std::string& name, const Eigen::Vector3d& force, const Eigen::Vector3d& torque, const std::string& reference) : CartesianState(name, reference) {
  this->set_force(force);
  this->set_torque(torque);
}

CartesianWrench::CartesianWrench(const std::string& name, const Eigen::Matrix<double, 6, 1>& wrench, const std::string& reference) : CartesianState(name, reference) {
  this->set_wrench(wrench);
}

CartesianWrench::CartesianWrench(const CartesianWrench& wrench) : CartesianState(wrench) {}

CartesianWrench::CartesianWrench(const CartesianState& state) : CartesianState(state) {}

CartesianWrench CartesianWrench::Zero(const std::string& name, const std::string& reference) {
  return CartesianState::Identity(name, reference);
}

CartesianWrench CartesianWrench::Random(const std::string& name, const std::string& reference) {
  // separating in the two lines in needed to avoid compilation error due to ambiguous constructor call
  Eigen::Matrix<double, 6, 1> random = Eigen::Matrix<double, 6, 1>::Random();
  return CartesianWrench(name, random, reference);
}

CartesianWrench& CartesianWrench::operator=(const CartesianState& state) {
  this->CartesianState::operator=(state);
  return (*this);
}

CartesianWrench& CartesianWrench::operator*=(const CartesianWrench& wrench) {
  this->CartesianState::operator*=(wrench);
  return (*this);
}

CartesianWrench CartesianWrench::operator*(const CartesianWrench& wrench) const {
  return this->CartesianState::operator*(wrench);
}

CartesianWrench& CartesianWrench::operator+=(const CartesianWrench& wrench) {
  this->CartesianState::operator+=(wrench);
  return (*this);
}

CartesianWrench CartesianWrench::operator+(const CartesianWrench& wrench) const {
  return this->CartesianState::operator+(wrench);
}

CartesianWrench& CartesianWrench::operator-=(const CartesianWrench& wrench) {
  this->CartesianState::operator-=(wrench);
  return (*this);
}

CartesianWrench CartesianWrench::operator-(const CartesianWrench& wrench) const {
  return this->CartesianState::operator-(wrench);
}

CartesianWrench& CartesianWrench::operator*=(double lambda) {
  this->CartesianState::operator*=(lambda);
  return (*this);
}

CartesianWrench CartesianWrench::operator*(double lambda) const {
  return this->CartesianState::operator*(lambda);
}

void CartesianWrench::clamp(double max_force, double max_torque, double force_noise_ratio, double torque_noise_ratio) {
  // clamp force
  this->clamp_state_variable(max_force, CartesianStateVariable::FORCE, force_noise_ratio);
  // clamp torque
  this->clamp_state_variable(max_torque, CartesianStateVariable::TORQUE, torque_noise_ratio);
}

CartesianWrench CartesianWrench::clamped(double max_force, double max_torque, double force_noise_ratio, double torque_noise_ratio) const {
  CartesianWrench result(*this);
  result.clamp(max_force, max_torque, force_noise_ratio, torque_noise_ratio);
  return result;
}

CartesianWrench CartesianWrench::copy() const {
  CartesianWrench result(*this);
  return result;
}

Eigen::VectorXd CartesianWrench::data() const {
  return this->get_wrench();
}

Eigen::Array<double, 6, 1> CartesianWrench::array() const {
  return this->get_wrench().array();
}

std::ostream& operator<<(std::ostream& os, const CartesianWrench& wrench) {
  if (wrench.is_empty()) {
    os << "Empty CartesianWrench";
  } else {
    os << wrench.get_name() << " CartesianWrench expressed in " << wrench.get_reference_frame() << " frame" << std::endl;
    os << "force: (" << wrench.get_force()(0) << ", ";
    os << wrench.get_force()(1) << ", ";
    os << wrench.get_force()(2) << ")" << std::endl;
    os << "torque: (" << wrench.get_torque()(0) << ", ";
    os << wrench.get_torque()(1) << ", ";
    os << wrench.get_torque()(2) << ")";
  }
  return os;
}

CartesianWrench operator*(double lambda, const CartesianWrench& wrench) {
  return wrench * lambda;
}
}// namespace StateRepresentation
