#include "state_representation/space/cartesian/CartesianState.hpp"
#include "state_representation/exceptions/EmptyStateException.hpp"
#include "state_representation/exceptions/IncompatibleReferenceFramesException.hpp"
#include "state_representation/exceptions/NotImplementedException.hpp"

using namespace state_representation::exceptions;

namespace state_representation {
CartesianState::CartesianState() : SpatialState(StateType::CARTESIANSTATE) {
  this->initialize();
}

CartesianState::CartesianState(const std::string& robot_name, const std::string& reference) :
    SpatialState(StateType::CARTESIANSTATE, robot_name, reference) {
  this->initialize();
}

void CartesianState::initialize() {
  this->State::initialize();
  this->set_zero();
}

void CartesianState::set_zero() {
  this->position_.setZero();
  this->orientation_.setIdentity();
  this->linear_velocity_.setZero();
  this->angular_velocity_.setZero();
  this->linear_acceleration_.setZero();
  this->angular_acceleration_.setZero();
  this->force_.setZero();
  this->torque_.setZero();
}

CartesianState CartesianState::Identity(const std::string& name, const std::string& reference) {
  CartesianState identity = CartesianState(name, reference);
  // as opposed to the constructor specify this state to be filled
  identity.set_filled();
  return identity;
}

CartesianState CartesianState::Random(const std::string& name, const std::string& reference) {
  CartesianState random = CartesianState(name, reference);
  // set all the state variables to random
  random.set_state_variable(Eigen::VectorXd::Random(25),
                            CartesianStateVariable::ALL);
  return random;
}

CartesianState& CartesianState::operator*=(double lambda) {
  // sanity check
  if (this->is_empty()) {
    throw EmptyStateException(this->get_name() + " state is empty");
  }
  // operation
  this->set_position(lambda * this->get_position());
  // calculate the scaled rotation as a displacement from identity
  Eigen::Quaterniond w = math_tools::log(this->get_orientation());
  // calculate the orientation corresponding to the scaled velocity
  this->set_orientation(math_tools::exp(w, lambda / 2.));
  // calculate the other vectors normally
  this->set_twist(lambda * this->get_twist());
  this->set_accelerations(lambda * this->get_accelerations());
  this->set_wrench(lambda * this->get_wrench());
  return (*this);
}

CartesianState CartesianState::operator*(double lambda) const {
  CartesianState result(*this);
  result *= lambda;
  return result;
}

CartesianState CartesianState::copy() const {
  CartesianState result(*this);
  return result;
}

Eigen::VectorXd CartesianState::data() const {
  return this->get_state_variable(CartesianStateVariable::ALL);
}

Eigen::ArrayXd CartesianState::array() const {
  return this->data().array();
}

CartesianState& CartesianState::operator*=(const CartesianState& state) {
  // sanity check
  if (this->is_empty()) {
    throw EmptyStateException(this->get_name() + " state is empty");
  }
  if (state.is_empty()) {
    throw EmptyStateException(state.get_name() + " state is empty");
  }
  if (this->get_name() != state.get_reference_frame()) {
    throw IncompatibleReferenceFramesException("Expected " + this->get_name() + ", got " + state.get_reference_frame());
  }
  this->set_name(state.get_name());
  // intermediate variables for f_S_b
  Eigen::Vector3d f_P_b = this->get_position();
  Eigen::Quaterniond f_R_b = this->get_orientation();
  Eigen::Vector3d f_v_b = this->get_linear_velocity();
  Eigen::Vector3d f_omega_b = this->get_angular_velocity();
  Eigen::Vector3d f_a_b = this->get_linear_acceleration();
  Eigen::Vector3d f_alpha_b = this->get_angular_acceleration();
  // intermediate variables for b_S_c
  Eigen::Vector3d b_P_c = state.get_position();
  Eigen::Quaterniond b_R_c = (this->get_orientation().dot(state.get_orientation()) > 0) ? state.get_orientation()
                                                                                        : Eigen::Quaterniond(-state.get_orientation().coeffs());
  Eigen::Vector3d b_v_c = state.get_linear_velocity();
  Eigen::Vector3d b_omega_c = state.get_angular_velocity();
  Eigen::Vector3d b_a_c = state.get_linear_acceleration();
  Eigen::Vector3d b_alpha_c = state.get_angular_acceleration();
  // pose
  this->set_position(f_P_b + f_R_b * b_P_c);
  this->set_orientation(f_R_b * b_R_c);
  // twist
  this->set_linear_velocity(f_v_b + f_R_b * b_v_c + f_omega_b.cross(f_R_b * b_P_c));
  this->set_angular_velocity(f_omega_b + f_R_b * b_omega_c);
  // acceleration
  this->set_linear_acceleration(f_a_b + f_R_b * b_a_c
                                + f_alpha_b.cross(f_R_b * b_P_c)
                                + 2 * f_omega_b.cross(f_R_b * b_v_c)
                                + f_omega_b.cross(f_omega_b.cross(f_R_b * b_P_c)));
  this->set_angular_acceleration(f_alpha_b + f_R_b * b_alpha_c + f_omega_b.cross(f_R_b * b_omega_c));
  // wrench
  //TODO
  return (*this);
}

CartesianState CartesianState::operator*(const CartesianState& state) const {
  CartesianState result(*this);
  result *= state;
  return result;
}

CartesianState& CartesianState::operator+=(const CartesianState& state) {
  // sanity check
  if (this->is_empty()) {
    throw EmptyStateException(this->get_name() + " state is empty");
  }
  if (state.is_empty()) {
    throw EmptyStateException(state.get_name() + " state is empty");
  }
  if (!(this->get_reference_frame() == state.get_reference_frame())) {
    throw IncompatibleReferenceFramesException("The two states do not have the same reference frame");
  }
  // operation on pose
  this->set_position(this->get_position() + state.get_position());
  // specific operation on quaternion using Hamilton product
  Eigen::Quaterniond orientation = (this->get_orientation().dot(state.get_orientation()) > 0) ? state.get_orientation()
                                                                                              : Eigen::Quaterniond(-state.get_orientation().coeffs());
  this->set_orientation(this->get_orientation() * orientation);
  // operation on twist
  this->set_twist(this->get_twist() + state.get_twist());
  // operation on accelerations
  this->set_accelerations(this->get_accelerations() + state.get_accelerations());
  // operation on wrench
  this->set_wrench(this->get_wrench() + state.get_wrench());
  return (*this);
}

CartesianState CartesianState::operator+(const CartesianState& state) const {
  CartesianState result(*this);
  result += state;
  return result;
}

CartesianState& CartesianState::operator-=(const CartesianState& state) {
  // sanity check
  if (this->is_empty()) {
    throw EmptyStateException(this->get_name() + " state is empty");
  }
  if (state.is_empty()) {
    throw EmptyStateException(state.get_name() + " state is empty");
  }
  if (!(this->get_reference_frame() == state.get_reference_frame())) {
    throw IncompatibleReferenceFramesException("The two states do not have the same reference frame");
  }
  // operation on pose
  this->set_position(this->get_position() - state.get_position());
  // specific operation on quaternion using Hamilton product
  Eigen::Quaterniond orientation = (this->get_orientation().dot(state.get_orientation()) > 0) ? state.get_orientation()
                                                                                              : Eigen::Quaterniond(-state.get_orientation().coeffs());
  this->set_orientation(this->get_orientation() * orientation.conjugate());
  // operation on twist
  this->set_twist(this->get_twist() - state.get_twist());
  // operation on accelerations
  this->set_accelerations(this->get_accelerations() - state.get_accelerations());
  // operation on wrench
  this->set_wrench(this->get_wrench() - state.get_wrench());
  return (*this);
}

CartesianState CartesianState::operator-(const CartesianState& state) const {
  CartesianState result(*this);
  result -= state;
  return result;
}

CartesianState CartesianState::inverse() const {
  CartesianState result(*this);
  // inverse name and reference frame
  std::string ref = result.get_reference_frame();
  result.set_reference_frame(result.get_name());
  result.set_name(ref);
  // intermediate variables for f_S_b
  Eigen::Vector3d f_P_b = this->get_position();
  Eigen::Quaterniond f_R_b = this->get_orientation();
  Eigen::Vector3d f_v_b = this->get_linear_velocity();
  Eigen::Vector3d f_omega_b = this->get_angular_velocity();
  Eigen::Vector3d f_a_b = this->get_linear_acceleration();
  Eigen::Vector3d f_alpha_b = this->get_angular_acceleration();
  // computation for b_S_f
  Eigen::Quaterniond b_R_f = f_R_b.conjugate();
  Eigen::Vector3d b_P_f = b_R_f * (-f_P_b);
  Eigen::Vector3d b_v_f = b_R_f * (-f_v_b);
  Eigen::Vector3d b_omega_f = b_R_f * (-f_omega_b);
  Eigen::Vector3d b_a_f = b_R_f * f_a_b;        // not sure if minus is needed
  Eigen::Vector3d b_alpha_f = b_R_f * f_alpha_b;// no minus for sure
  // wrench
  //TODO
  // collect the results
  result.set_position(b_P_f);
  result.set_orientation(b_R_f);
  result.set_linear_velocity(b_v_f);
  result.set_angular_velocity(b_omega_f);
  result.set_linear_acceleration(b_a_f);
  result.set_angular_acceleration(b_alpha_f);
  return result;
}

void CartesianState::clamp_state_variable(double max_value,
                                          const CartesianStateVariable& state_variable_type,
                                          double noise_ratio) {
  Eigen::VectorXd state_variable_value = this->get_state_variable(state_variable_type);
  if (noise_ratio != 0) {
    state_variable_value -= noise_ratio * state_variable_value.normalized();
    // apply a deadzone
    if (state_variable_value.norm() < noise_ratio) { state_variable_value.setZero(); }
  }
  // clamp the values to their maximum amplitude provided
  if (state_variable_value.norm() > max_value) { state_variable_value = max_value * state_variable_value.normalized(); }
  this->set_state_variable(state_variable_value, state_variable_type);
}

double CartesianState::dist(const CartesianState& state, const CartesianStateVariable& state_variable_type) const {
  // sanity check
  if (this->is_empty()) {
    throw EmptyStateException(this->get_name() + " state is empty");
  }
  if (state.is_empty()) {
    throw EmptyStateException(state.get_name() + " state is empty");
  }
  if (!(this->get_reference_frame() == state.get_reference_frame())) {
    throw IncompatibleReferenceFramesException("The two states do not have the same reference frame");
  }
  // calculation
  double result = 0;
  if (state_variable_type == CartesianStateVariable::POSITION || state_variable_type == CartesianStateVariable::POSE
      || state_variable_type == CartesianStateVariable::ALL) {
    result += (this->get_position() - state.get_position()).norm();
  }
  if (state_variable_type == CartesianStateVariable::ORIENTATION || state_variable_type == CartesianStateVariable::POSE
      || state_variable_type == CartesianStateVariable::ALL) {
    // https://math.stackexchange.com/questions/90081/quaternion-distance for orientation
    double inner_product = this->get_orientation().dot(state.get_orientation());
    double argument = 2 * inner_product * inner_product - 1;
    result += acos(std::min(1.0, std::max(-1.0, argument)));
  }
  if (state_variable_type == CartesianStateVariable::LINEAR_VELOCITY
      || state_variable_type == CartesianStateVariable::TWIST || state_variable_type == CartesianStateVariable::ALL) {
    result += (this->get_linear_velocity() - state.get_linear_velocity()).norm();
  }
  if (state_variable_type == CartesianStateVariable::ANGULAR_VELOCITY
      || state_variable_type == CartesianStateVariable::TWIST || state_variable_type == CartesianStateVariable::ALL) {
    result += (this->get_angular_velocity() - state.get_angular_velocity()).norm();
  }
  if (state_variable_type == CartesianStateVariable::LINEAR_ACCELERATION
      || state_variable_type == CartesianStateVariable::ACCELERATIONS
      || state_variable_type == CartesianStateVariable::ALL) {
    result += (this->get_linear_acceleration() - state.get_linear_acceleration()).norm();
  }
  if (state_variable_type == CartesianStateVariable::ANGULAR_ACCELERATION
      || state_variable_type == CartesianStateVariable::ACCELERATIONS
      || state_variable_type == CartesianStateVariable::ALL) {
    result += (this->get_angular_acceleration() - state.get_angular_acceleration()).norm();
  }
  if (state_variable_type == CartesianStateVariable::FORCE || state_variable_type == CartesianStateVariable::WRENCH
      || state_variable_type == CartesianStateVariable::ALL) {
    result += (this->get_force() - state.get_force()).norm();
  }
  if (state_variable_type == CartesianStateVariable::TORQUE || state_variable_type == CartesianStateVariable::WRENCH
      || state_variable_type == CartesianStateVariable::ALL) {
    result += (this->get_torque() - state.get_torque()).norm();
  }
  return result;
}

std::vector<double> CartesianState::norms(const CartesianStateVariable& state_variable_type) const {
  // compute the norms for each independent state variable
  std::vector<double> norms;
  if (state_variable_type == CartesianStateVariable::POSITION || state_variable_type == CartesianStateVariable::POSE
      || state_variable_type == CartesianStateVariable::ALL) {
    norms.push_back(this->get_position().norm());
  }
  if (state_variable_type == CartesianStateVariable::ORIENTATION || state_variable_type == CartesianStateVariable::POSE
      || state_variable_type == CartesianStateVariable::ALL) {
    norms.push_back(this->get_orientation().norm());
  }
  if (state_variable_type == CartesianStateVariable::LINEAR_VELOCITY
      || state_variable_type == CartesianStateVariable::TWIST || state_variable_type == CartesianStateVariable::ALL) {
    norms.push_back(this->get_linear_velocity().norm());
  }
  if (state_variable_type == CartesianStateVariable::ANGULAR_VELOCITY
      || state_variable_type == CartesianStateVariable::TWIST || state_variable_type == CartesianStateVariable::ALL) {
    norms.push_back(this->get_angular_velocity().norm());
  }
  if (state_variable_type == CartesianStateVariable::LINEAR_ACCELERATION
      || state_variable_type == CartesianStateVariable::ACCELERATIONS
      || state_variable_type == CartesianStateVariable::ALL) {
    norms.push_back(this->get_linear_acceleration().norm());
  }
  if (state_variable_type == CartesianStateVariable::ANGULAR_ACCELERATION
      || state_variable_type == CartesianStateVariable::ACCELERATIONS
      || state_variable_type == CartesianStateVariable::ALL) {
    norms.push_back(this->get_angular_acceleration().norm());
  }
  if (state_variable_type == CartesianStateVariable::FORCE || state_variable_type == CartesianStateVariable::WRENCH
      || state_variable_type == CartesianStateVariable::ALL) {
    norms.push_back(this->get_force().norm());
  }
  if (state_variable_type == CartesianStateVariable::TORQUE || state_variable_type == CartesianStateVariable::WRENCH
      || state_variable_type == CartesianStateVariable::ALL) {
    norms.push_back(this->get_torque().norm());
  }
  return norms;
}

void CartesianState::normalize(const CartesianStateVariable& state_variable_type) {
  if (state_variable_type == CartesianStateVariable::POSITION || state_variable_type == CartesianStateVariable::POSE
      || state_variable_type == CartesianStateVariable::ALL) {
    this->position_.normalize();
  }
  // there shouldn't be a need to renormalize orientation as it is already normalized
  if (state_variable_type == CartesianStateVariable::ORIENTATION || state_variable_type == CartesianStateVariable::POSE
      || state_variable_type == CartesianStateVariable::ALL) {
    this->orientation_.normalize();
  }
  if (state_variable_type == CartesianStateVariable::LINEAR_VELOCITY
      || state_variable_type == CartesianStateVariable::TWIST || state_variable_type == CartesianStateVariable::ALL) {
    this->linear_velocity_.normalize();
  }
  if (state_variable_type == CartesianStateVariable::ANGULAR_VELOCITY
      || state_variable_type == CartesianStateVariable::TWIST || state_variable_type == CartesianStateVariable::ALL) {
    this->angular_velocity_.normalize();
  }
  if (state_variable_type == CartesianStateVariable::LINEAR_ACCELERATION
      || state_variable_type == CartesianStateVariable::ACCELERATIONS
      || state_variable_type == CartesianStateVariable::ALL) {
    this->linear_acceleration_.normalize();
  }
  if (state_variable_type == CartesianStateVariable::ANGULAR_ACCELERATION
      || state_variable_type == CartesianStateVariable::ACCELERATIONS
      || state_variable_type == CartesianStateVariable::ALL) {
    this->angular_acceleration_.normalize();
  }
  if (state_variable_type == CartesianStateVariable::FORCE || state_variable_type == CartesianStateVariable::WRENCH
      || state_variable_type == CartesianStateVariable::ALL) {
    this->force_.normalize();
  }
  if (state_variable_type == CartesianStateVariable::TORQUE || state_variable_type == CartesianStateVariable::WRENCH
      || state_variable_type == CartesianStateVariable::ALL) {
    this->torque_.normalize();
  }
}

CartesianState CartesianState::normalized(const CartesianStateVariable& state_variable_type) const {
  CartesianState result(*this);
  result.normalize(state_variable_type);
  return result;
}

std::ostream& operator<<(std::ostream& os, const CartesianState& state) {
  if (state.is_empty()) {
    os << "Empty CartesianState";
  } else {
    os << state.get_name() << " CartesianState expressed in " << state.get_reference_frame() << " frame" << std::endl;
    os << "position: (" << state.position_(0) << ", ";
    os << state.position_(1) << ", ";
    os << state.position_(2) << ")" << std::endl;
    os << "orientation: (" << state.orientation_.w() << ", ";
    os << state.orientation_.x() << ", ";
    os << state.orientation_.y() << ", ";
    os << state.orientation_.z() << ")";
    Eigen::AngleAxisd axis_angle(state.orientation_);
    os << " <=> theta: " << axis_angle.angle() << ", ";
    os << "axis: (" << axis_angle.axis()(0) << ", ";
    os << axis_angle.axis()(1) << ", ";
    os << axis_angle.axis()(2) << ")" << std::endl;
    os << "linear velocity: (" << state.linear_velocity_(0) << ", ";
    os << state.linear_velocity_(1) << ", ";
    os << state.linear_velocity_(2) << ")" << std::endl;
    os << "angular velocity: (" << state.angular_velocity_(0) << ", ";
    os << state.angular_velocity_(1) << ", ";
    os << state.angular_velocity_(2) << ")" << std::endl;
    os << "linear acceleration: (" << state.linear_acceleration_(0) << ", ";
    os << state.linear_acceleration_(1) << ", ";
    os << state.linear_acceleration_(2) << ")" << std::endl;
    os << "angular acceleration: (" << state.angular_acceleration_(0) << ", ";
    os << state.angular_acceleration_(1) << ", ";
    os << state.angular_acceleration_(2) << ")" << std::endl;
    os << "force: (" << state.force_(0) << ", ";
    os << state.force_(1) << ", ";
    os << state.force_(2) << ")" << std::endl;
    os << "torque: (" << state.torque_(0) << ", ";
    os << state.torque_(1) << ", ";
    os << state.torque_(2) << ")";
  }
  return os;
}

CartesianState operator*(double lambda, const CartesianState& state) {
  return state * lambda;
}

double dist(const CartesianState& s1, const CartesianState& s2, const CartesianStateVariable& state_variable_type) {
  return s1.dist(s2, state_variable_type);
}

void CartesianState::from_std_vector(const std::vector<double>&) {
  throw (NotImplementedException("from_std_vector() is not implemented for the base CartesianState class"));
}
}// namespace state_representation
