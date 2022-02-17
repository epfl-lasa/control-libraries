#include "dynamical_systems/PointAttractor.hpp"

#include "dynamical_systems/exceptions/NotImplementedException.hpp"
#include "dynamical_systems/exceptions/EmptyAttractorException.hpp"
#include "state_representation/exceptions/InvalidParameterException.hpp"
#include "dynamical_systems/exceptions/IncompatibleSizeException.hpp"
#include "state_representation/exceptions/EmptyStateException.hpp"
#include "state_representation/exceptions/IncompatibleReferenceFramesException.hpp"
#include "state_representation/exceptions/IncompatibleStatesException.hpp"
#include "state_representation/space/cartesian/CartesianPose.hpp"
#include "state_representation/space/cartesian/CartesianState.hpp"
#include "state_representation/space/joint/JointPositions.hpp"
#include "state_representation/space/joint/JointState.hpp"

using namespace state_representation;

namespace dynamical_systems {

template<>
PointAttractor<CartesianState>::PointAttractor() :
    attractor_(make_shared_parameter<CartesianState>("attractor", CartesianState())),
    gain_(make_shared_parameter<Eigen::MatrixXd>("gain", Eigen::MatrixXd::Identity(6, 6))) {
  this->parameters_.insert(std::make_pair("attractor", attractor_));
  this->parameters_.insert(std::make_pair("gain", gain_));
}

template<>
PointAttractor<JointState>::PointAttractor() :
    attractor_(make_shared_parameter<JointState>("attractor", JointState())),
    gain_(std::make_shared<Parameter<Eigen::MatrixXd>>("gain")) {
  this->parameters_.insert(std::make_pair("attractor", attractor_));
  this->parameters_.insert(std::make_pair("gain", gain_));
}

template<class S>
bool PointAttractor<S>::is_compatible(const S& state) const {
  return IDynamicalSystem<S>::is_compatible(state);
}

template bool PointAttractor<CartesianState>::is_compatible(const CartesianState& state) const;

template<>
bool PointAttractor<JointState>::is_compatible(const JointState& state) const {
  auto attractor = this->get_parameter_value<JointState>("attractor");
  if (attractor.is_empty()) {
    throw exceptions::EmptyAttractorException("The attractor of the dynamical system is empty.");
  }
  bool compatible = (attractor.get_size() == state.get_size());
  if (compatible) {
    for (unsigned int i = 0; i < attractor.get_size(); ++i) {
      compatible = (compatible && attractor.get_names()[i] == state.get_names()[i]);
    }
  }
  return compatible;
}

template<class S>
void PointAttractor<S>::set_gain(const std::shared_ptr<ParameterInterface>& parameter, unsigned int expected_size) {
  if (parameter->get_type() == StateType::PARAMETER_DOUBLE) {
    auto gain = parameter->get_parameter_value<double>();
    this->gain_->set_value(gain * Eigen::MatrixXd::Identity(expected_size, expected_size));
  } else if (parameter->get_type() == StateType::PARAMETER_DOUBLE_ARRAY) {
    auto gain = parameter->get_parameter_value<std::vector<double>>();
    if (gain.size() != expected_size) {
      throw exceptions::IncompatibleSizeException(
          "The provided diagonal coefficients do not correspond to the expected size of the attractor");
    }
    Eigen::VectorXd diagonal = Eigen::VectorXd::Map(gain.data(), expected_size);
    this->gain_->set_value(diagonal.asDiagonal());
  } else if (parameter->get_type() == StateType::PARAMETER_MATRIX) {
    auto gain = parameter->get_parameter_value<Eigen::MatrixXd>();
    if (gain.rows() != expected_size && gain.cols() != expected_size) {
      throw exceptions::IncompatibleSizeException(
          "The provided gain matrix do not have the expected size (" + std::to_string(expected_size) + "x"
              + std::to_string(expected_size) + ")");
    }
    this->gain_->set_value(gain);
  } else {
    throw state_representation::exceptions::InvalidParameterException("Parameter 'gain' has incorrect type");
  }
}

template<class S>
void PointAttractor<S>::set_attractor(const S&) {
  throw exceptions::NotImplementedException("set_attractor is not implemented for this type of DS");
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
              + this->get_base_frame().get_name() + " in frame " + this->get_base_frame().get_reference_frame() + ".");
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
  this->attractor_->set_value(attractor);
  if (this->gain_->get_value().size() == 0) {
    this->gain_->set_value(Eigen::MatrixXd::Identity(attractor.get_size(), attractor.get_size()));
  }
}

template<class S>
void PointAttractor<S>::set_base_frame(const S& base_frame) {
  this->IDynamicalSystem<S>::set_base_frame(base_frame);
}

template void PointAttractor<JointState>::set_base_frame(const JointState& base_frame);

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
void PointAttractor<CartesianState>::validate_and_set_parameter(const std::shared_ptr<ParameterInterface>& parameter) {
  if (parameter->get_name() == "attractor") {
    this->set_attractor(parameter->get_parameter_value<CartesianState>());
  } else if (parameter->get_name() == "gain") {
    this->set_gain(parameter, 6);
  } else {
    throw state_representation::exceptions::InvalidParameterException(
        "No parameter with name '" + parameter->get_name() + "' found");
  }
}

template<>
void PointAttractor<JointState>::validate_and_set_parameter(const std::shared_ptr<ParameterInterface>& parameter) {
  if (parameter->get_name() == "attractor") {
    this->set_attractor(parameter->get_parameter_value<JointState>());
  } else if (parameter->get_name() == "gain") {
    this->set_gain(parameter, this->attractor_->get_value().get_size());
  } else {
    throw state_representation::exceptions::InvalidParameterException(
        "No parameter with name '" + parameter->get_name() + "' found");
  }
}

template<class S>
S PointAttractor<S>::compute_dynamics(const S&) const {
  throw exceptions::NotImplementedException("compute_dynamics is not implemented for this type of DS");
}

template<>
CartesianState PointAttractor<CartesianState>::compute_dynamics(const CartesianState& state) const {
  if (this->attractor_->get_value().is_empty()) {
    throw exceptions::EmptyAttractorException("The attractor of the dynamical system is empty.");
  }
  CartesianTwist twist = CartesianPose(this->attractor_->get_value()) - CartesianPose(state);
  twist *= this->gain_->get_value();
  return CartesianTwist(state.get_name(), twist.get_twist(), this->attractor_->get_value().get_reference_frame());
}

template<>
JointState PointAttractor<JointState>::compute_dynamics(const JointState& state) const {
  JointVelocities velocities = JointPositions(this->attractor_->get_value()) - JointPositions(state);
  velocities *= this->gain_->get_value();
  return JointVelocities(state.get_name(), this->attractor_->get_value().get_names(), velocities.get_velocities());
}
}// namespace dynamical_systems
