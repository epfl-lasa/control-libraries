#include "controllers/impedance/CompliantTwist.hpp"

#include "state_representation/exceptions/InvalidParameterException.hpp"

namespace controllers::impedance {

using namespace state_representation;

CompliantTwist::CompliantTwist(
    double linear_principle_damping, double linear_orthogonal_damping, double angular_stiffness, double angular_damping
) : linear_principle_damping_(
    std::make_shared<Parameter<double>>("linear_principle_damping", linear_principle_damping)),
    linear_orthogonal_damping_(
        std::make_shared<Parameter<double>>("linear_orthogonal_damping", linear_orthogonal_damping)),
    angular_stiffness_(std::make_shared<Parameter<double>>("angular_stiffness", angular_stiffness)),
    angular_damping_(std::make_shared<Parameter<double>>("angular_damping", angular_damping)),
    dissipative_ctrl_(ComputationalSpaceType::LINEAR),
    velocity_impedance_ctrl_(6) {
  this->parameters_.insert(std::make_pair("linear_principle_damping", linear_principle_damping_));
  this->parameters_.insert(std::make_pair("linear_orthogonal_damping", linear_orthogonal_damping_));
  this->parameters_.insert(std::make_pair("angular_stiffness", angular_stiffness_));
  this->parameters_.insert(std::make_pair("angular_damping", angular_damping_));

  set_linear_gains(linear_principle_damping, linear_orthogonal_damping);
  set_angular_gains(angular_stiffness, angular_damping);
}

CompliantTwist::CompliantTwist(
    const std::list<std::shared_ptr<state_representation::ParameterInterface>>& parameters
) : CompliantTwist(1, 1, 1, 1) {
  this->set_parameters(parameters);
}

void CompliantTwist::set_linear_principle_damping(double linear_principle_damping) {
  set_linear_gains(linear_principle_damping, linear_orthogonal_damping_->get_value());
}

void CompliantTwist::set_linear_orthogonal_damping(double linear_orthogonal_damping) {
  set_linear_gains(linear_principle_damping_->get_value(), linear_orthogonal_damping);
}

void CompliantTwist::set_linear_gains(double linear_principle_damping, double linear_orthogonal_damping) {
  linear_principle_damping_->set_value(linear_principle_damping);
  linear_orthogonal_damping_->set_value(linear_orthogonal_damping);

  Eigen::VectorXd damping(6);
  damping << linear_principle_damping, linear_orthogonal_damping, linear_orthogonal_damping, 0, 0, 0;
  dissipative_ctrl_.set_parameter_value("damping_eigenvalues", damping);
}

void CompliantTwist::set_angular_stiffness(double angular_stiffness) {
  set_angular_gains(angular_stiffness, angular_damping_->get_value());
}

void CompliantTwist::set_angular_damping(double angular_damping) {
  set_angular_gains(angular_stiffness_->get_value(), angular_damping);
}

void CompliantTwist::set_angular_gains(double angular_stiffness, double angular_damping) {
  angular_stiffness_->set_value(angular_stiffness);
  angular_damping_->set_value(angular_damping);

  Eigen::MatrixXd k(6, 6), d(6, 6);
  k.diagonal() << 0, 0, 0, angular_stiffness, angular_stiffness, angular_stiffness;
  d.diagonal() << 0, 0, 0, angular_damping, angular_damping, angular_damping;
  velocity_impedance_ctrl_.set_parameter_value("stiffness", k);
  velocity_impedance_ctrl_.set_parameter_value("damping", d);
}

CartesianState CompliantTwist::compute_command(
    const CartesianState& desired_state, const CartesianState& feedback_state
) {
  CartesianState
      command = dissipative_ctrl_.compute_command(CartesianTwist(desired_state), CartesianTwist(feedback_state));
  command += velocity_impedance_ctrl_.compute_command(desired_state, feedback_state);
  return command;
}

void CompliantTwist::validate_and_set_parameter(
    const std::shared_ptr<state_representation::ParameterInterface>& parameter
) {
  if (parameter->get_type() != StateType::PARAMETER_DOUBLE) {
    throw state_representation::exceptions::InvalidParameterException(
        "Parameter " + parameter->get_name() + " must be a double");
  }
  double value = std::static_pointer_cast<Parameter<double>>(parameter)->get_value();
  if (parameter->get_name() == "linear_principle_damping") {
    this->set_linear_principle_damping(value);
  } else if (parameter->get_name() == "linear_orthogonal_damping") {
    this->set_linear_orthogonal_damping(value);
  } else if (parameter->get_name() == "angular_stiffness") {
    this->set_angular_stiffness(value);
  } else if (parameter->get_name() == "angular_damping") {
    this->set_angular_damping(value);
  }
}

}