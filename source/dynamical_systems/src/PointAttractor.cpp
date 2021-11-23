#include "dynamical_systems/PointAttractor.hpp"

#include "dynamical_systems/exceptions/EmptyAttractorException.hpp"
#include "dynamical_systems/exceptions/InvalidParameterException.hpp"
#include "dynamical_systems/exceptions/IncompatibleSizeException.hpp"
#include "state_representation/exceptions/EmptyStateException.hpp"
#include "state_representation/exceptions/IncompatibleReferenceFramesException.hpp"
#include "state_representation/space/cartesian/CartesianPose.hpp"
#include "state_representation/robot/JointPositions.hpp"
#include "state_representation/robot/JointState.hpp"

using namespace state_representation;

namespace dynamical_systems {

template<>
PointAttractor<CartesianState>::PointAttractor() :
    attractor_(std::make_shared<Parameter<CartesianState>>(Parameter<CartesianPose>("attractor", CartesianPose()))),
    gain_(std::make_shared<Parameter<Eigen::MatrixXd>>("gain", Eigen::MatrixXd::Identity(6, 6))) {
  this->param_map_.insert(std::make_pair("attractor", attractor_));
  this->param_map_.insert(std::make_pair("gain", gain_));
}

template<>
PointAttractor<JointState>::PointAttractor() :
    attractor_(std::make_shared<Parameter<JointState>>(Parameter<JointPositions>("attractor"))),
    gain_(std::make_shared<Parameter<Eigen::MatrixXd>>("gain")) {
  this->param_map_.insert(std::make_pair("attractor", attractor_));
  this->param_map_.insert(std::make_pair("gain", gain_));
}

template<class S>
void PointAttractor<S>::set_gain(const std::shared_ptr<ParameterInterface>& parameter, unsigned int expected_size) {
  if (parameter->get_type() == StateType::PARAMETER_DOUBLE) {
    auto gain = std::static_pointer_cast<Parameter<double>>(parameter);
    this->gain_->set_value(gain->get_value() * Eigen::MatrixXd::Identity(expected_size, expected_size));
  } else if (parameter->get_type() == StateType::PARAMETER_DOUBLE_ARRAY) {
    auto gain = std::static_pointer_cast<Parameter<std::vector<double>>>(parameter);
    if (gain->get_value().size() != expected_size) {
      throw exceptions::IncompatibleSizeException(
          "The provided diagonal coefficients do not correspond to the expected size of the attractor"
      );
    }
    Eigen::VectorXd diagonal = Eigen::VectorXd::Map(gain->get_value().data(), expected_size);
    this->gain_->set_value(diagonal.asDiagonal());
  } else if (parameter->get_type() == StateType::PARAMETER_MATRIX) {
    auto gain = std::static_pointer_cast<Parameter<Eigen::MatrixXd>>(parameter);
    if (gain->get_value().rows() != expected_size && gain->get_value().cols() != expected_size) {
      throw exceptions::IncompatibleSizeException(
          "The provided gain matrix do not have the expected size (" + std::to_string(expected_size) + "x"
              + std::to_string(expected_size) + ")"
      );
    }
    this->gain_->set_value(gain->get_value());
  } else {
    throw exceptions::InvalidParameterException("Parameter 'gain' has incorrect type");
  }
}

template<>
void PointAttractor<CartesianState>::set_attractor(const CartesianState& attractor) {
  if (attractor.is_empty()) {
    throw state_representation::exceptions::EmptyStateException(attractor.get_name() + " state is empty");
  }
  if (this->get_base_frame().is_empty()) {
    IDynamicalSystem<CartesianState>::set_base_frame(
        CartesianState::Identity(attractor.get_reference_frame(), attractor.get_reference_frame()));
  }
  // validate that the reference frame of the attractor is always compatible with the DS reference frame
  if (attractor.get_reference_frame() != this->get_base_frame().get_name()) {
    if (attractor.get_reference_frame() != this->get_base_frame().get_reference_frame()) {
      throw state_representation::exceptions::IncompatibleReferenceFramesException(
          "The reference frame of the attractor " + attractor.get_name() + " in frame "
              + attractor.get_reference_frame() + " is incompatible with the base frame of the dynamical system "
              + this->get_base_frame().get_name() + " in frame " + this->get_base_frame().get_reference_frame() + "."
      );
    }
    this->attractor_->set_value(this->get_base_frame().inverse() * attractor);
  } else {
    this->attractor_->set_value(attractor);
  }
}

template<>
void PointAttractor<JointState>::set_attractor(const JointState& attractor) {
  if (attractor.is_empty()) {
    throw state_representation::exceptions::EmptyStateException(attractor.get_name() + " state is empty");
  }
  if (this->get_base_frame().is_empty()) {
    IDynamicalSystem<JointState>::set_base_frame(JointState::Zero(attractor.get_name(), attractor.get_names()));
  }
  // validate that the attractor is compatible with the DS reference name
  if (!this->is_compatible(attractor)) {
    throw state_representation::exceptions::IncompatibleReferenceFramesException(
        "The attractor " + attractor.get_name() + " is incompatible with the base frame of the dynamical system "
            + this->get_base_frame().get_name() + "."
    );
  }
  this->attractor_->set_value(attractor);
  if (this->gain_->get_value().size() == 0) {
    this->gain_->set_value(Eigen::MatrixXd::Identity(attractor.get_size(), attractor.get_size()));
  }
}

template<>
void PointAttractor<CartesianState>::set_base_frame(const CartesianState& base_frame) {
  if (base_frame.is_empty()) {
    throw state_representation::exceptions::EmptyStateException(base_frame.get_name() + " state is empty");
  }
  IDynamicalSystem<CartesianState>::set_base_frame(base_frame);
  if (!this->attractor_->get_value().is_empty()) {
    // update reference frame of attractor
    auto attractor = this->attractor_->get_value();
    attractor.set_reference_frame(base_frame.get_name());
    this->set_attractor(attractor);
  }
}

template<>
void PointAttractor<JointState>::set_base_frame(const JointState& base_frame) {
  if (base_frame.is_empty()) {
    throw state_representation::exceptions::EmptyStateException(base_frame.get_name() + " state is empty");
  }
  IDynamicalSystem<JointState>::set_base_frame(base_frame);
  if (!this->attractor_->get_value().is_empty()) {
    // update name of attractor
    auto attractor = this->attractor_->get_value();
    attractor.set_name(base_frame.get_name());
    attractor.set_names(base_frame.get_names());
    this->set_attractor(attractor);
  }
}

template<>
void PointAttractor<CartesianState>::validate_and_set_parameter(const std::shared_ptr<ParameterInterface>& parameter) {
  if (parameter->get_name() == "attractor") {
    this->set_attractor(std::static_pointer_cast<Parameter<CartesianPose>>(parameter)->get_value());
  } else if (parameter->get_name() == "gain") {
    this->set_gain(parameter, 6);
  } else {
    throw exceptions::InvalidParameterException("No parameter with name '" + parameter->get_name() + "' found");
  }
}

template<>
void PointAttractor<JointState>::validate_and_set_parameter(const std::shared_ptr<ParameterInterface>& parameter) {
  if (parameter->get_name() == "attractor") {
    this->assert_parameter_valid("attractor", parameter->get_type());
    this->set_attractor(std::static_pointer_cast<Parameter<JointState>>(parameter)->get_value());
  } else if (parameter->get_name() == "gain") {
    this->set_gain(parameter, this->attractor_->get_value().get_size());
  } else {
    throw exceptions::InvalidParameterException("No parameter with name '" + parameter->get_name() + "' found");
  }
}

template<>
CartesianState PointAttractor<CartesianState>::compute_dynamics(const CartesianState& state) const {
  if (this->attractor_->get_value().is_empty()) {
    throw exceptions::EmptyAttractorException("The attractor of the dynamical system is empty.");
  }
  CartesianTwist twist = CartesianPose(this->attractor_->get_value()) - CartesianPose(state);
  twist *= this->gain_->get_value();
  return twist;
}

template<>
JointState PointAttractor<JointState>::compute_dynamics(const JointState& state) const {
  if (this->attractor_->get_value().is_empty()) {
    throw exceptions::EmptyAttractorException("The attractor of the dynamical system is empty.");
  }
  JointVelocities velocities = JointPositions(this->attractor_->get_value()) - JointPositions(state);
  velocities *= this->gain_->get_value();
  return velocities;
}
}// namespace dynamical_systems
