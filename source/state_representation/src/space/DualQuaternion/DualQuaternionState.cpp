#include "state_representation/space/DualQuaternion/DualQuaternionState.hpp"

namespace state_representation {
DualQuaternionState::DualQuaternionState() :
    SpatialState(StateType::DUALQUATERNIONSTATE) {}

DualQuaternionState::DualQuaternionState(const std::string& name, const std::string& reference) :
    SpatialState(StateType::DUALQUATERNIONSTATE, name, reference) {
  this->initialize();
}

DualQuaternionState::DualQuaternionState(const DualQuaternionState& state) :
    SpatialState(state),
    primary(state.primary),
    dual(state.dual) {}

DualQuaternionState::DualQuaternionState(const std::string& name,
                                         const Eigen::Quaterniond& primary,
                                         const Eigen::Quaterniond& dual,
                                         const std::string& reference) :
    SpatialState(StateType::DUALQUATERNIONSTATE, name, reference) {
  this->set_primary(primary);
  this->set_dual(dual);
}

DualQuaternionState& DualQuaternionState::operator*=(const DualQuaternionState& q) {
  Eigen::Quaterniond primary = this->get_primary() * q.get_primary();
  Eigen::Quaterniond dual =
      Eigen::Quaterniond((this->get_primary() * q.get_dual()).coeffs() + (this->get_dual() * q.get_primary()).coeffs());
  this->set_primary(primary);
  this->set_dual(dual);
  this->set_name(q.get_name());
  return (*this);
}

const DualQuaternionState DualQuaternionState::operator*(const DualQuaternionState& p) const {
  DualQuaternionState result(*this);
  result *= p;
  return result;
}

const DualQuaternionState DualQuaternionState::conjugate() const {
  DualQuaternionState result(*this);
  result.set_primary(result.get_primary().conjugate());
  result.set_dual(result.get_dual().conjugate());
  return result;
}

void DualQuaternionState::initialize() {
  this->State::initialize();
  this->primary = Eigen::Quaterniond::Identity();
  this->dual = Eigen::Quaterniond(0, 0, 0, 0);
}

const DualQuaternionState operator*(const float& lambda, const DualQuaternionState& state) {
  DualQuaternionState result(state.get_name(), state.get_reference_frame());
  result.set_primary(Eigen::Quaterniond(lambda * state.get_primary().coeffs()));
  result.set_dual(Eigen::Quaterniond(lambda * state.get_dual().coeffs()));
  return result;
}

const DualQuaternionState exp(const DualQuaternionState& state) {
  DualQuaternionState result(state.get_name(), state.get_reference_frame());
  Eigen::Quaterniond pexps;
  if (state.get_primary().norm() > 1e-5) {
    Eigen::Array4d
        coeffs = (sin(state.get_primary().norm()) / state.get_primary().norm()) * state.get_primary().coeffs();
    // to be extra carefulm Quaternion coeffs are returned as (x,y,z,w)
    pexps = Eigen::Quaterniond(coeffs(3) + cos(state.get_primary().norm()), coeffs(0), coeffs(1), coeffs(2));
  } else {
    pexps = Eigen::Quaterniond::Identity();
  }
  result.set_primary(pexps);
  result.set_dual(state.get_dual() * pexps);
  return result;
}

const DualQuaternionState DualQuaternionState::copy() const {
  DualQuaternionState result(*this);
  return result;
}

std::ostream& operator<<(std::ostream& os, const DualQuaternionState& state) {
  if (state.is_empty()) {
    os << "Empty DualQuaternionState";
  } else {
    os << state.get_name() << " DualQuaternionState expressed in " << state.get_reference_frame() << " frame"
       << std::endl;
    os << "primary: (" << state.primary.w() << ", ";
    os << state.primary.x() << ", ";
    os << state.primary.y() << ", ";
    os << state.primary.z() << ")" << std::endl;
    os << "dual: (" << state.dual.w() << ", ";
    os << state.dual.x() << ", ";
    os << state.dual.y() << ", ";
    os << state.dual.z() << ")";
  }
  return os;
}
}