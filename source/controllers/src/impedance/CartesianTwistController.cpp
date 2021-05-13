#include "controllers/impedance/CartesianTwistController.hpp"

namespace controllers::impedance {

using namespace state_representation;

CartesianTwistController::CartesianTwistController(double linear_principle_damping,
                                                   double linear_orthogonal_damping,
                                                   double angular_stiffness,
                                                   double angular_damping) :
    linear_principle_damping_(std::make_shared<Parameter<double>>("linear_principle_damping", linear_principle_damping)),
    linear_orthogonal_damping_(std::make_shared<Parameter<double>>("linear_orthogonal_damping", linear_orthogonal_damping)),
    angular_stiffness_(std::make_shared<Parameter<double>>("angular_stiffness", angular_stiffness)),
    angular_damping_(std::make_shared<Parameter<double>>("angular_damping", angular_damping)),
    dissipative_ctrl_(ComputationalSpaceType::LINEAR),
    velocity_impedance_ctrl_(Eigen::MatrixXd::Zero(6, 6), Eigen::MatrixXd::Zero(6, 6)) {
  set_angular_gains(angular_stiffness, angular_damping);
}

CartesianTwistController::CartesianTwistController(const Eigen::Vector4d& gains) :
  CartesianTwistController(gains(0), gains(1), gains(2), gains(3)) {

}

CartesianTwistController::CartesianTwistController(const CartesianTwistController& other) :
    linear_principle_damping_(std::make_shared<Parameter<double>>("linear_principle_damping", other.get_gains()(0))),
    linear_orthogonal_damping_(std::make_shared<Parameter<double>>("linear_orthogonal_damping", other.get_gains()(1))),
    angular_stiffness_(std::make_shared<Parameter<double>>("angular_stiffness", other.get_gains()(2))),
    angular_damping_(std::make_shared<Parameter<double>>("angular_damping", other.get_gains()(3))),
    dissipative_ctrl_(other.dissipative_ctrl_),
    velocity_impedance_ctrl_(other.velocity_impedance_ctrl_) {
}

CartesianTwistController& CartesianTwistController::operator=(const CartesianTwistController& other) {
  CartesianTwistController tmp(other);
  swap(*this, tmp);
  return *this;
}

void CartesianTwistController::set_gains(double linear_principle_damping,
                                         double linear_orthogonal_damping,
                                         double angular_stiffness,
                                         double angular_damping) {
  set_linear_gains(linear_principle_damping, linear_orthogonal_damping);
  set_angular_gains(angular_stiffness, angular_damping);
}

void CartesianTwistController::set_gains(const Eigen::Vector4d& gains) {
  set_linear_gains(gains(0), gains(1));
  set_angular_gains(gains(2), gains(3));
}


Eigen::Vector4d CartesianTwistController::get_gains() const {
  return Eigen::Vector4d(linear_principle_damping_->get_value(),
                         linear_orthogonal_damping_->get_value(),
                         angular_stiffness_->get_value(),
                         angular_damping_->get_value());
}

void CartesianTwistController::set_linear_principle_damping(double linear_principle_damping) {
  set_linear_gains(linear_principle_damping, linear_orthogonal_damping_->get_value());
}

void CartesianTwistController::set_linear_orthogonal_damping(double linear_orthogonal_damping) {
  set_linear_gains(linear_principle_damping_->get_value(), linear_orthogonal_damping);
}

void CartesianTwistController::set_linear_gains(double linear_principle_damping, double linear_orthogonal_damping) {
  linear_principle_damping_->set_value(linear_principle_damping);
  linear_orthogonal_damping_->set_value(linear_orthogonal_damping);

  Eigen::VectorXd damping(6);
  damping << linear_principle_damping, linear_orthogonal_damping, linear_orthogonal_damping, 0, 0, 0;
  dissipative_ctrl_.set_damping_eigenvalues(damping);
}

void CartesianTwistController::set_angular_stiffness(double angular_stiffness) {
  set_angular_gains(angular_stiffness, angular_damping_->get_value());
}

void CartesianTwistController::set_angular_damping(double angular_damping) {
  set_angular_gains(angular_stiffness_->get_value(), angular_damping);
}

void CartesianTwistController::set_angular_gains(double angular_stiffness, double angular_damping) {
  angular_stiffness_->set_value(angular_stiffness);
  angular_damping_->set_value(angular_damping);

  Eigen::MatrixXd k(6, 6), d(6, 6);
  k.diagonal() << 0, 0, 0, angular_stiffness, angular_stiffness, angular_stiffness;
  d.diagonal() << 0, 0, 0, angular_damping, angular_damping, angular_damping;
  velocity_impedance_ctrl_ = VelocityImpedance<CartesianState>(k, d);
}

std::list<std::shared_ptr<state_representation::ParameterInterface>> CartesianTwistController::get_parameters() const {
  std::list<std::shared_ptr<state_representation::ParameterInterface>> param_list;
  param_list.push_back(linear_principle_damping_);
  param_list.push_back(linear_orthogonal_damping_);
  param_list.push_back(angular_stiffness_);
  param_list.push_back(angular_damping_);
  return param_list;
}

state_representation::CartesianState CartesianTwistController::compute_command(const state_representation::CartesianState& desired_state,
                                                                               const state_representation::CartesianState& feedback_state) {
  CartesianWrench command = dissipative_ctrl_.compute_command(CartesianTwist(desired_state), CartesianTwist(feedback_state));
  command += velocity_impedance_ctrl_.compute_command(desired_state, feedback_state);
  return command;
}

state_representation::JointState CartesianTwistController::compute_command(const state_representation::CartesianState& desired_state,
                                                                           const state_representation::CartesianState& feedback_state,
                                                                           const state_representation::Jacobian& jacobian) {
  CartesianWrench wrench = compute_command(desired_state, feedback_state);
  return jacobian.transpose() * wrench;
}

}