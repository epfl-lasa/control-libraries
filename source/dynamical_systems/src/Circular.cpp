#include "dynamical_systems/Circular.hpp"

#include "dynamical_systems/exceptions/EmptyAttractorException.hpp"

using namespace state_representation;

namespace dynamical_systems {

Circular::Circular() :
    DynamicalSystem<CartesianState>(),
    limit_cycle_(std::make_shared<Parameter<Ellipsoid>>("limit_cycle", Ellipsoid("limit_cycle", "limit_cycle"))),
    planar_gain_(std::make_shared<Parameter<double>>("planar_gain", 1.0)),
    normal_gain_(std::make_shared<Parameter<double>>("normal_gain", 1.0)),
    circular_velocity_(std::make_shared<Parameter<double>>("circular_velocity", M_PI / 2)) {
  this->limit_cycle_->get_value().set_center_state(CartesianState("limit_cycle", "limit_cycle"));
}

Circular::Circular(const CartesianState& center, double radius, double gain, double circular_velocity) :
    DynamicalSystem(center.get_reference_frame()),
    limit_cycle_(std::make_shared<Parameter<Ellipsoid>>("limit_cycle",
                                                        Ellipsoid(center.get_name(), center.get_reference_frame()))),
    planar_gain_(std::make_shared<Parameter<double>>("planar_gain", gain)),
    normal_gain_(std::make_shared<Parameter<double>>("normal_gain", gain)),
    circular_velocity_(std::make_shared<Parameter<double>>("circular_velocity", circular_velocity)) {
  this->limit_cycle_->get_value().set_center_state(center);
  this->limit_cycle_->get_value().set_axis_lengths({radius, radius});
}

Circular::Circular(const Ellipsoid& limit_cycle, double gain, double circular_velocity) :
    DynamicalSystem(limit_cycle.get_center_state().get_reference_frame()),
    limit_cycle_(std::make_shared<Parameter<Ellipsoid>>("limit_cycle", limit_cycle)),
    planar_gain_(std::make_shared<Parameter<double>>("planar_gain", gain)),
    normal_gain_(std::make_shared<Parameter<double>>("normal_gain", gain)),
    circular_velocity_(std::make_shared<Parameter<double>>("circular_velocity", circular_velocity)) {}

CartesianState Circular::compute_dynamics(const CartesianState& state) const {
  if (this->get_limit_cycle().get_center_state().is_empty()) {
    throw exceptions::EmptyAttractorException("The limit cycle of the dynamical system is empty.");
  }
  // put the point in the reference of the center
  auto pose = CartesianPose(state);
  pose = this->get_limit_cycle().get_rotation().inverse() * this->get_center().inverse() * pose;

  CartesianTwist velocity(pose.get_name(), pose.get_reference_frame());
  Eigen::Vector3d linear_velocity;
  linear_velocity(2) = -this->get_normal_gain() * pose.get_position()(2);

  std::vector<double> radiuses = this->get_radiuses();

  double a2ratio = (pose.get_position()[0] * pose.get_position()[0]) / (radiuses[0] * radiuses[0]);
  double b2ratio = (pose.get_position()[1] * pose.get_position()[1]) / (radiuses[1] * radiuses[1]);
  double dradius = -this->get_planar_gain() * radiuses[0] * radiuses[1] * (a2ratio + b2ratio - 1);
  double tangent_velocity_x = -radiuses[0] / radiuses[1] * pose.get_position()[1];
  double tangent_velocity_y = radiuses[1] / radiuses[0] * pose.get_position()[0];

  linear_velocity(0) = this->get_circular_velocity() * tangent_velocity_x + dradius * tangent_velocity_y;
  linear_velocity(1) = this->get_circular_velocity() * tangent_velocity_y - dradius * tangent_velocity_x;

  velocity.set_linear_velocity(linear_velocity);
  velocity.set_angular_velocity(Eigen::Vector3d::Zero());

  //compute back the linear velocity in the desired frame
  auto frame = this->get_center() * this->get_limit_cycle().get_rotation();
  return CartesianState(frame) * velocity;
}

std::list<std::shared_ptr<ParameterInterface>> Circular::get_parameters() const {
  std::list<std::shared_ptr<ParameterInterface>> param_list;
  param_list.push_back(this->limit_cycle_);
  param_list.push_back(this->planar_gain_);
  param_list.push_back(this->normal_gain_);
  param_list.push_back(this->circular_velocity_);
  return param_list;
}

void Circular::set_base_frame(const CartesianState& base_frame) {
  if (base_frame.is_empty()) {
    throw state_representation::exceptions::EmptyStateException(base_frame.get_name() + " state is empty");
  }
  DynamicalSystem<CartesianState>::set_base_frame(base_frame);
  // update reference frame of center
  if (!this->limit_cycle_->get_value().get_center_state().is_empty()) {
    auto center_state = this->limit_cycle_->get_value().get_center_state();
    center_state.set_reference_frame(base_frame.get_name());
    this->limit_cycle_->get_value().set_center_state(center_state);
  }
}
}// namespace dynamical_systems