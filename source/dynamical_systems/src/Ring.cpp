#include "dynamical_systems/Ring.hpp"

#include "dynamical_systems/exceptions/EmptyAttractorException.hpp"
#include "dynamical_systems/exceptions/InvalidParameterException.hpp"
#include "state_representation/space/cartesian/CartesianTwist.hpp"
#include "state_representation/exceptions/IncompatibleReferenceFramesException.hpp"
#include "state_representation/exceptions/EmptyStateException.hpp"

using namespace state_representation;

namespace dynamical_systems {

Ring::Ring() :
    center_(std::make_shared<Parameter<CartesianPose>>("center", CartesianPose())),
    rotation_offset_(std::make_shared<Parameter<CartesianPose>>("rotation_offset", CartesianPose())),
    radius_(std::make_shared<Parameter<double>>("radius", 1.0)),
    width_(std::make_shared<Parameter<double>>("width", 0.5)),
    speed_(std::make_shared<Parameter<double>>("speed", 1.0)),
    field_strength_(std::make_shared<Parameter<double>>("field_strength", 1.0)),
    normal_gain_(std::make_shared<Parameter<double>>("normal_gain", 1.0)),
    angular_gain_(std::make_shared<Parameter<double>>("angular_gain", 1.0)) {
  this->center_->get_value().set_empty();
  this->rotation_offset_->set_value(
      CartesianPose("rotation", Eigen::Quaterniond::Identity(), this->center_->get_value().get_name()));
  this->param_map_.insert(std::make_pair("center", this->center_));
  this->param_map_.insert(std::make_pair("rotation_offset", this->rotation_offset_));
  this->param_map_.insert(std::make_pair("radius", this->radius_));
  this->param_map_.insert(std::make_pair("width", this->width_));
  this->param_map_.insert(std::make_pair("speed", this->speed_));
  this->param_map_.insert(std::make_pair("field_strength", this->field_strength_));
  this->param_map_.insert(std::make_pair("normal_gain", this->normal_gain_));
  this->param_map_.insert(std::make_pair("angular_gain", this->angular_gain_));
}

void Ring::set_center(const CartesianPose& center) {
  if (center.is_empty()) {
    throw state_representation::exceptions::EmptyStateException(center.get_name() + " state is empty");
  }
  if (this->get_base_frame().is_empty()) {
    IDynamicalSystem<CartesianState>::set_base_frame(
        CartesianState::Identity(center.get_reference_frame(), center.get_reference_frame()));
  }
  // validate that the reference frame of the center is always compatible with the DS reference frame
  if (center.get_reference_frame() != this->get_base_frame().get_name()) {
    if (center.get_reference_frame() != this->get_base_frame().get_reference_frame()) {
      throw state_representation::exceptions::IncompatibleReferenceFramesException(
          "The reference frame of the center " + center.get_name() + " in frame " + center.get_reference_frame()
              + " is incompatible with the base frame of the dynamical system " + this->get_base_frame().get_name()
              + " in frame " + this->get_base_frame().get_reference_frame() + "."
      );
    }
    this->center_->set_value(this->get_base_frame().inverse() * center);
  } else {
    this->center_->set_value(center);
  }
  this->set_rotation_offset(this->get_rotation_offset());
}

void Ring::set_rotation_offset(const Eigen::Quaterniond& rotation) {
  auto pose = CartesianPose("rotation", rotation, this->center_->get_value().get_name());
  this->rotation_offset_->set_value(pose);
}

Eigen::Quaterniond Ring::get_rotation_offset() const {
  return this->rotation_offset_->get_value().get_orientation();
}

void Ring::set_base_frame(const CartesianState& base_frame) {
  if (base_frame.is_empty()) {
    throw state_representation::exceptions::EmptyStateException(base_frame.get_name() + " state is empty");
  }
  IDynamicalSystem<CartesianState>::set_base_frame(base_frame);
  // update reference frame of center
  if (!this->center_->get_value().is_empty()) {
    auto center = this->center_->get_value();
    center.set_reference_frame(base_frame.get_name());
    this->center_->set_value(center);
  }
}

void Ring::validate_and_set_parameter(const std::shared_ptr<ParameterInterface>& parameter) {
  if (parameter->get_name() == "center") {
    this->assert_parameter_valid(parameter);
    this->set_center(std::static_pointer_cast<Parameter<CartesianPose>>(parameter)->get_value());
  } else if (parameter->get_name() == "rotation_offset") {
    this->assert_parameter_valid(parameter);
    this->rotation_offset_->set_value(std::static_pointer_cast<Parameter<CartesianPose>>(parameter)->get_value());
  } else if (parameter->get_name() == "radius") {
    this->assert_parameter_valid(parameter);
    this->radius_->set_value(std::static_pointer_cast<Parameter<double>>(parameter)->get_value());
  } else if (parameter->get_name() == "width") {
    this->assert_parameter_valid(parameter);
    this->width_->set_value(std::static_pointer_cast<Parameter<double>>(parameter)->get_value());
  } else if (parameter->get_name() == "speed") {
    this->assert_parameter_valid(parameter);
    this->speed_->set_value(std::static_pointer_cast<Parameter<double>>(parameter)->get_value());
  } else if (parameter->get_name() == "field_strength") {
    this->assert_parameter_valid(parameter);
    this->field_strength_->set_value(std::static_pointer_cast<Parameter<double>>(parameter)->get_value());
  } else if (parameter->get_name() == "normal_gain") {
    this->assert_parameter_valid(parameter);
    this->normal_gain_->set_value(std::static_pointer_cast<Parameter<double>>(parameter)->get_value());
  } else if (parameter->get_name() == "angular_gain") {
    this->assert_parameter_valid(parameter);
    this->angular_gain_->set_value(std::static_pointer_cast<Parameter<double>>(parameter)->get_value());
  } else {
    throw exceptions::InvalidParameterException("No parameter with name '" + parameter->get_name() + "' found");
  }
}

Eigen::Vector3d Ring::calculate_local_linear_velocity(const CartesianPose& pose, double& local_field_strength) const {
  Eigen::Vector3d local_linear_velocity = Eigen::Vector3d::Zero();

  // get the 2d components of position on the XY plane
  Eigen::Vector2d position2d(pose.get_position().x(), pose.get_position().y());

  double d = position2d.norm();
  if (d < 1e-7) {
    return local_linear_velocity;
  }

  double re = M_PI_2 * (d - this->radius_->get_value()) / this->width_->get_value();
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
  velocity2d *= this->speed_->get_value();

  // calculate the normal velocity
  double vz = -this->normal_gain_->get_value() * pose.get_position().z();

  // combine into 3D velocity
  local_linear_velocity << velocity2d, vz;

  // calculate the field strength and scale the velocity
  local_field_strength = this->field_strength_->get_value() + (1 - this->field_strength_->get_value()) * cos(re);
  local_linear_velocity *= local_field_strength;

  return local_linear_velocity;
}

Eigen::Vector3d Ring::calculate_local_angular_velocity(
    const CartesianPose& pose, const Eigen::Vector3d& linearVelocity, double local_field_strength
) const {
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

  local_angular_velocity = this->angular_gain_->get_value() * deltaOmega.vec();
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

CartesianState Ring::compute_dynamics(const CartesianState& state) const {
  if (this->center_->get_value().is_empty()) {
    throw exceptions::EmptyAttractorException("The center of the dynamical system is empty.");
  }
  // put the point in the reference of the center
  CartesianPose pose(state);
  pose = this->center_->get_value().inverse() * pose;
  // apply the rotation offset
  pose.set_orientation(pose.get_orientation() * this->get_rotation_offset().conjugate());

  CartesianTwist twist(pose.get_name(), pose.get_reference_frame());
  double local_field_strength;
  twist.set_linear_velocity(this->calculate_local_linear_velocity(pose, local_field_strength));
  twist.set_angular_velocity(
      this->calculate_local_angular_velocity(
          pose, twist.get_linear_velocity(), local_field_strength
      ));

  // transform the twist back to the base reference frame
  return CartesianState(this->center_->get_value()) * twist;
}
}// namespace dynamical_systems
