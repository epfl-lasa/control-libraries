#include "dynamical_systems/Ring.hpp"

#include "dynamical_systems/exceptions/EmptyAttractorException.hpp"
#include "state_representation/space/cartesian/CartesianTwist.hpp"
#include "state_representation/exceptions/IncompatibleReferenceFramesException.hpp"
#include "state_representation/exceptions/EmptyStateException.hpp"

using namespace state_representation;

namespace dynamical_systems {

Ring::Ring() :
    DynamicalSystem<CartesianState>(),
    center_(std::make_shared<Parameter<CartesianPose>>("center", CartesianPose())),
    rotation_offset_(std::make_shared<Parameter<CartesianPose>>("rotation_offset", CartesianPose())),
    radius_(std::make_shared<Parameter<double>>("radius", 1.0)),
    width_(std::make_shared<Parameter<double>>("width", 0.5)),
    speed_(std::make_shared<Parameter<double>>("speed", 1.0)),
    field_strength_(std::make_shared<Parameter<double>>("field_strength", 1.0)),
    normal_gain_(std::make_shared<Parameter<double>>("normal_gain", 1.0)),
    angular_gain_(std::make_shared<Parameter<double>>("angular_gain", 1.0)) {
  this->center_->get_value().set_empty();
  this->set_rotation_offset(Eigen::Quaterniond::Identity());
}

Ring::Ring(const CartesianState& center,
           double radius,
           double width,
           double speed,
           double field_strength,
           double normal_gain,
           double angular_gain) :
    DynamicalSystem<CartesianState>(center.get_reference_frame()),
    center_(std::make_shared<Parameter<CartesianPose>>("center", center)),
    rotation_offset_(std::make_shared<Parameter<CartesianPose>>("rotation_offset", center)),
    radius_(std::make_shared<Parameter<double>>("radius", radius)),
    width_(std::make_shared<Parameter<double>>("width", width)),
    speed_(std::make_shared<Parameter<double>>("speed", speed)),
    field_strength_(std::make_shared<Parameter<double>>("field_strength", field_strength)),
    normal_gain_(std::make_shared<Parameter<double>>("normal_gain", normal_gain)),
    angular_gain_(std::make_shared<Parameter<double>>("angular_gain", angular_gain)) {
  this->set_center(center);
  this->set_rotation_offset(Eigen::Quaterniond::Identity());
}

CartesianState Ring::compute_dynamics(const CartesianState& state) const {
  if (this->get_center().is_empty()) {
    throw exceptions::EmptyAttractorException("The center of the dynamical system is empty.");
  }
  // put the point in the reference of the center
  CartesianPose pose(state);
  pose = this->get_center().inverse() * pose;
  // apply the rotation offset
  pose.set_orientation(pose.get_orientation() * this->get_rotation_offset().conjugate());

  CartesianTwist twist(pose.get_name(), pose.get_reference_frame());
  double local_field_strength;
  twist.set_linear_velocity(this->calculate_local_linear_velocity(pose, local_field_strength));
  twist.set_angular_velocity(this->calculate_local_angular_velocity(pose,
                                                                    twist.get_linear_velocity(),
                                                                    local_field_strength));

  // transform the twist back to the base reference frame
  return CartesianState(this->get_center()) * twist;
}

Eigen::Vector3d Ring::calculate_local_linear_velocity(const CartesianPose& pose, double& local_field_strength) const {
  Eigen::Vector3d local_linear_velocity = Eigen::Vector3d::Zero();

  // get the 2d components of position on the XY plane
  Eigen::Vector2d position2d(pose.get_position().x(), pose.get_position().y());

  double d = position2d.norm();
  if (d < 1e-7) {
    return local_linear_velocity;
  }

  double re = M_PI_2 * (d - this->get_radius()) / this->get_width();
  if (re > M_PI_2) {
    re = M_PI_2;
  } else if (re < -M_PI_2) {
    re = -M_PI_2;
  }

  // calculate the velocity of a point as an orthogonal unit vector, rectified towards the radius based on re
  Eigen::Matrix2d R;
  R << -sin(re), -cos(re), cos(re), -sin(re);
  Eigen::Vector2d velocity2d = R * position2d / d;

  // scale by the nominal speed
  velocity2d *= this->get_speed();

  // calculate the normal velocity
  double vz = -this->get_normal_gain() * pose.get_position().z();

  // combine into 3D velocity
  local_linear_velocity << velocity2d, vz;

  // calculate the field strength and scale the velocity
  local_field_strength = this->get_field_strength() + (1 - this->get_field_strength()) * cos(re);
  local_linear_velocity *= local_field_strength;

  return local_linear_velocity;
}

Eigen::Vector3d Ring::calculate_local_angular_velocity(const CartesianPose& pose,
                                                       const Eigen::Vector3d& linearVelocity,
                                                       double local_field_strength) const {
  Eigen::Vector3d local_angular_velocity = Eigen::Vector3d::Zero();

  double theta = atan2(pose.get_position().y(), pose.get_position().x());

  Eigen::Quaterniond qd = Eigen::Quaterniond::Identity();
  qd.w() = cos(theta / 2);
  qd.z() = sin(theta / 2);

  if (pose.get_orientation().dot(qd) < 0) {
    qd.coeffs() = -qd.coeffs();
  }

  Eigen::Quaterniond deltaQ = qd * pose.get_orientation().conjugate();
  if (deltaQ.vec().norm() < 1e-7) {
    return local_angular_velocity;
  }

  //dOmega = 2 * ln (deltaQ)
  Eigen::Quaterniond deltaOmega = Eigen::Quaterniond::Identity();
  deltaOmega.w() = 0;
  double phi = atan2(deltaQ.vec().norm(), deltaQ.w());
  deltaOmega.vec() = 2 * deltaQ.vec() * phi / sin(phi);

  local_angular_velocity = get_angular_gain() * deltaOmega.vec();
  local_angular_velocity *= local_field_strength;

  Eigen::Vector2d position2d(pose.get_position().x(), pose.get_position().y());
  Eigen::Vector2d linear_velocity2d(linearVelocity.x(), linearVelocity.y());
  if (position2d.norm() < 1e-7 || linear_velocity2d.norm() < 1e-7) {
    return local_angular_velocity;
  }

  // clamp linear velocity magnitude to half the position, so that the angular displacement
  // around the local Z axis caused by the linear velocity stays within one quadrant
  if (linear_velocity2d.norm() > (0.5 * position2d.norm())) {
    linear_velocity2d = linear_velocity2d.normalized() * (0.5 * position2d).norm();
  }
  double projection = position2d.normalized().dot((position2d + linear_velocity2d).normalized());
  double dthetaZ = 0;
  if (1 - abs(projection) > 1e-7) {
    dthetaZ = acos(projection);
  }
  local_angular_velocity.z() += dthetaZ;

  return local_angular_velocity;
}

void Ring::set_base_frame(const state_representation::CartesianState& base_frame) {
  if (base_frame.is_empty()) {
    throw state_representation::exceptions::EmptyStateException(base_frame.get_name() + " state is empty");
  }
  DynamicalSystem<state_representation::CartesianState>::set_base_frame(base_frame);
  // update reference frame of center
  if (!this->get_center().is_empty()) {
    auto center = this->get_center();
    center.set_reference_frame(base_frame.get_name());
    this->set_center(center);
  }
}

void Ring::set_center(const CartesianPose& center) {
  if (center.is_empty()) {
    throw state_representation::exceptions::EmptyStateException(center.get_name() + " state is empty");
  }
  if (this->get_base_frame().is_empty()) {
    DynamicalSystem<CartesianState>::set_base_frame(CartesianState::Identity(center.get_reference_frame(),
                                                                             center.get_reference_frame()));
  }
  // validate that the reference frame of the center is always compatible with the DS reference frame
  if (center.get_reference_frame() != this->get_base_frame().get_name()) {
    if (center.get_reference_frame() != this->get_base_frame().get_reference_frame()) {
      throw state_representation::exceptions::IncompatibleReferenceFramesException(
          "The reference frame of the center " + center.get_name() + " in frame " + center.get_reference_frame()
              + " is incompatible with the base frame of the dynamical system " + this->get_base_frame().get_name()
              + " in frame " + this->get_base_frame().get_reference_frame() + ".");
    }
    this->center_->set_value(this->get_base_frame().inverse() * center);
  } else {
    this->center_->set_value(center);
  }
  this->set_rotation_offset(this->get_rotation_offset());
}

void Ring::set_rotation_offset(const Eigen::Quaterniond& rotation) {
  auto pose = CartesianPose("rotation", rotation, this->get_center().get_name());
  this->rotation_offset_->set_value(pose);
}

void Ring::set_radius(double radius) {
  this->radius_->set_value(radius);
}

void Ring::set_width(double width) {
  this->width_->set_value(width);
}

void Ring::set_speed(double speed) {
  this->speed_->set_value(speed);
}

void Ring::set_field_strength(double field_strength) {
  this->field_strength_->set_value(field_strength);
}

void Ring::set_normal_gain(double normal_gain) {
  this->normal_gain_->set_value(normal_gain);
}

void Ring::set_angular_gain(double angular_gain) {
  this->angular_gain_->set_value(angular_gain);
}

const CartesianPose& Ring::get_center() const {
  return this->center_->get_value();
}

Eigen::Quaterniond Ring::get_rotation_offset() const {
  return this->rotation_offset_->get_value().get_orientation();
}

double Ring::get_radius() const {
  return this->radius_->get_value();
}

double Ring::get_width() const {
  return this->width_->get_value();
}

double Ring::get_speed() const {
  return this->speed_->get_value();
}

double Ring::get_field_strength() const {
  return this->field_strength_->get_value();
}

double Ring::get_normal_gain() const {
  return this->normal_gain_->get_value();
}

double Ring::get_angular_gain() const {
  return this->angular_gain_->get_value();
}

std::list<std::shared_ptr<ParameterInterface>> Ring::get_parameters() const {
  std::list<std::shared_ptr<ParameterInterface>> param_list;
  param_list.push_back(this->center_);
  param_list.push_back(this->rotation_offset_);
  param_list.push_back(this->radius_);
  param_list.push_back(this->width_);
  param_list.push_back(this->speed_);
  param_list.push_back(this->field_strength_);
  param_list.push_back(this->normal_gain_);
  param_list.push_back(this->angular_gain_);
  return param_list;
}
}// namespace dynamical_systems