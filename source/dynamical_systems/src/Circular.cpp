#include "dynamical_systems/Circular.hpp"

using namespace state_representation;

namespace DynamicalSystems {
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
    DynamicalSystem(limit_cycle.get_center_pose().get_reference_frame()),
    limit_cycle_(std::make_shared<Parameter<Ellipsoid>>("limit_cycle", limit_cycle)),
    planar_gain_(std::make_shared<Parameter<double>>("planar_gain", gain)),
    normal_gain_(std::make_shared<Parameter<double>>("normal_gain", gain)),
    circular_velocity_(std::make_shared<Parameter<double>>("circular_velocity", circular_velocity)) {}

CartesianState Circular::compute_dynamics(const CartesianState& state) const {
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
  double tangeant_velocity_x = -radiuses[0] / radiuses[1] * pose.get_position()[1];
  double tangeant_velocity_y = radiuses[1] / radiuses[0] * pose.get_position()[0];

  linear_velocity(0) = this->get_circular_velocity() * tangeant_velocity_x + dradius * tangeant_velocity_y;
  linear_velocity(1) = this->get_circular_velocity() * tangeant_velocity_y - dradius * tangeant_velocity_x;

  velocity.set_linear_velocity(linear_velocity);
  velocity.set_angular_velocity(Eigen::Vector3d::Zero());

  //compute back the linear velocity in the desired frame
  auto frame = this->get_center() * this->get_limit_cycle().get_rotation();
  return CartesianTwist(frame) * velocity;
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
  DynamicalSystem<CartesianState>::set_base_frame(base_frame);
  // update reference frame of center
  auto center_state = this->limit_cycle_->get_value().get_center_state();
  center_state.set_reference_frame(base_frame.get_name());
  this->limit_cycle_->get_value().set_center_state(center_state);
}
}// namespace DynamicalSystems