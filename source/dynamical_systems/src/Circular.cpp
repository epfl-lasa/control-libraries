#include "dynamical_systems/Circular.hpp"

#include "dynamical_systems/exceptions/EmptyAttractorException.hpp"
#include "dynamical_systems/exceptions/InvalidParameterException.hpp"

#include "state_representation/exceptions/EmptyStateException.hpp"

using namespace state_representation;

namespace dynamical_systems {

Circular::Circular() :
    limit_cycle_(std::make_shared<Parameter<Ellipsoid>>("limit_cycle", Ellipsoid("limit_cycle", "limit_cycle"))),
    planar_gain_(std::make_shared<Parameter<double>>("planar_gain", 1.0)),
    normal_gain_(std::make_shared<Parameter<double>>("normal_gain", 1.0)),
    circular_velocity_(std::make_shared<Parameter<double>>("circular_velocity", M_PI / 2)) {
  this->limit_cycle_->get_value().set_center_state(CartesianState("limit_cycle", "limit_cycle"));
  this->param_map_.insert(std::make_pair("limit_cycle", this->limit_cycle_));
  this->param_map_.insert(std::make_pair("planar_gain", this->planar_gain_));
  this->param_map_.insert(std::make_pair("normal_gain", this->normal_gain_));
  this->param_map_.insert(std::make_pair("circular_velocity", this->circular_velocity_));
}

void Circular::set_limit_cycle(Ellipsoid& limit_cycle) {
  const auto& center = limit_cycle.get_center_state();
  if (center.is_empty()) {
    throw state_representation::exceptions::EmptyStateException(center.get_name() + " state is empty");
  }
  if (this->get_base_frame().is_empty()) {
    IDynamicalSystem<CartesianState>::set_base_frame(
        CartesianState::Identity(
            center.get_reference_frame(), center.get_reference_frame()));
  }
  if (center.get_reference_frame() != this->get_base_frame().get_name()) {
    if (center.get_reference_frame() != this->get_base_frame().get_reference_frame()) {
      throw state_representation::exceptions::IncompatibleReferenceFramesException(
          "The reference frame of the center " + center.get_name() + " in frame " + center.get_reference_frame()
              + " is incompatible with the base frame of the dynamical system " + this->get_base_frame().get_name()
              + " in frame " + this->get_base_frame().get_reference_frame() + "."
      );
    }
    limit_cycle.set_center_state(this->get_base_frame().inverse() * center);
  }
  this->limit_cycle_->set_value(limit_cycle);
}

void Circular::set_base_frame(const CartesianState& base_frame) {
  if (base_frame.is_empty()) {
    throw state_representation::exceptions::EmptyStateException(base_frame.get_name() + " state is empty");
  }
  IDynamicalSystem<CartesianState>::set_base_frame(base_frame);
  // update reference frame of center
  if (!this->limit_cycle_->get_value().get_center_state().is_empty()) {
    auto center_state = this->limit_cycle_->get_value().get_center_state();
    center_state.set_reference_frame(base_frame.get_name());
    this->limit_cycle_->get_value().set_center_state(center_state);
  }
}

void Circular::validate_and_set_parameter(const std::shared_ptr<ParameterInterface>& parameter) {
  if (parameter->get_name() == "limit_cycle") {
    this->assert_parameter_valid(parameter);
    this->set_limit_cycle(std::static_pointer_cast<Parameter<Ellipsoid>>(parameter)->get_value());
  } else if (parameter->get_name() == "planar_gain") {
    this->assert_parameter_valid(parameter);
    this->planar_gain_->set_value(std::static_pointer_cast<Parameter<double>>(parameter)->get_value());
  } else if (parameter->get_name() == "normal_gain") {
    this->assert_parameter_valid(parameter);
    this->normal_gain_->set_value(std::static_pointer_cast<Parameter<double>>(parameter)->get_value());
  } else if (parameter->get_name() == "circular_velocity") {
    this->assert_parameter_valid(parameter);
    this->circular_velocity_->set_value(std::static_pointer_cast<Parameter<double>>(parameter)->get_value());
  } else {
    throw exceptions::InvalidParameterException("No parameter with name '" + parameter->get_name() + "' found");
  }
}

CartesianState Circular::compute_dynamics(const CartesianState& state) const {
  if (this->limit_cycle_->get_value().get_center_state().is_empty()) {
    throw exceptions::EmptyAttractorException("The limit cycle of the dynamical system is empty.");
  }
  // put the point in the reference of the center
  auto pose = CartesianPose(state);
  pose = this->limit_cycle_->get_value().get_rotation().inverse()
      * this->limit_cycle_->get_value().get_center_pose().inverse() * pose;

  CartesianTwist velocity(pose.get_name(), pose.get_reference_frame());
  Eigen::Vector3d linear_velocity;
  linear_velocity(2) = -this->normal_gain_->get_value() * pose.get_position()(2);

  std::vector<double> radiuses = this->limit_cycle_->get_value().get_axis_lengths();

  double a2ratio = (pose.get_position()[0] * pose.get_position()[0]) / (radiuses[0] * radiuses[0]);
  double b2ratio = (pose.get_position()[1] * pose.get_position()[1]) / (radiuses[1] * radiuses[1]);
  double dradius = -this->planar_gain_->get_value() * radiuses[0] * radiuses[1] * (a2ratio + b2ratio - 1);
  double tangent_velocity_x = -radiuses[0] / radiuses[1] * pose.get_position()[1];
  double tangent_velocity_y = radiuses[1] / radiuses[0] * pose.get_position()[0];

  linear_velocity(0) = this->circular_velocity_->get_value() * tangent_velocity_x + dradius * tangent_velocity_y;
  linear_velocity(1) = this->circular_velocity_->get_value() * tangent_velocity_y - dradius * tangent_velocity_x;

  velocity.set_linear_velocity(linear_velocity);
  velocity.set_angular_velocity(Eigen::Vector3d::Zero());

  //compute back the linear velocity in the desired frame
  auto frame = this->limit_cycle_->get_value().get_center_pose() * this->limit_cycle_->get_value().get_rotation();
  return CartesianState(frame) * velocity;
}
}// namespace dynamical_systems
