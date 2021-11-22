#include "dynamical_systems/IDynamicalSystem.hpp"

#include "dynamical_systems/exceptions/EmptyBaseFrameException.hpp"
#include "dynamical_systems/exceptions/InvalidParameterException.hpp"
#include "dynamical_systems/exceptions/NotImplementedException.hpp"

#include "state_representation/exceptions/IncompatibleReferenceFramesException.hpp"
#include "state_representation/parameters/Parameter.hpp"
#include "state_representation/robot/JointState.hpp"

using namespace state_representation;

namespace dynamical_systems {

template<class S>
IDynamicalSystem<S>::IDynamicalSystem() = default;

template<>
IDynamicalSystem<JointState>::IDynamicalSystem() : base_frame_(JointState()) {}

template<>
IDynamicalSystem<CartesianState>::IDynamicalSystem() : base_frame_(CartesianState()) {}

template<class S>
bool IDynamicalSystem<S>::is_compatible(const S&) const {
  throw exceptions::NotImplementedException("is_compatible(state) not implemented for this type of state.");
}

template<>
bool IDynamicalSystem<CartesianState>::is_compatible(const CartesianState& state) const {
  return !(state.get_reference_frame() != this->get_base_frame().get_name()
      && state.get_reference_frame() != this->get_base_frame().get_reference_frame());
}

template<>
bool IDynamicalSystem<JointState>::is_compatible(const JointState& state) const {
  return this->base_frame_.is_compatible(state);
}

template<class S>
S IDynamicalSystem<S>::evaluate(const S& state) const {
  return this->compute_dynamics(state);
}

template<>
CartesianState IDynamicalSystem<CartesianState>::evaluate(const CartesianState& state) const {
  if (this->get_base_frame().is_empty()) {
    throw exceptions::EmptyBaseFrameException("The base frame of the dynamical system is empty.");
  }
  if (state.get_reference_frame() != this->get_base_frame().get_name()) {
    if (state.get_reference_frame() != this->get_base_frame().get_reference_frame()) {
      throw state_representation::exceptions::IncompatibleReferenceFramesException(
          "The evaluated state " + state.get_name() + " in frame " + state.get_reference_frame()
              + " is incompatible with the base frame of the dynamical system " + this->get_base_frame().get_name()
              + " in frame " + this->get_base_frame().get_reference_frame() + "."
      );
    }
    CartesianState result = this->get_base_frame().inverse() * state;
    result = this->compute_dynamics(result);
    return this->get_base_frame() * result;
  } else {
    return this->compute_dynamics(state);
  }
}

template<>
JointState IDynamicalSystem<JointState>::evaluate(const JointState& state) const {
  if (this->get_base_frame().is_empty()) {
    throw exceptions::EmptyBaseFrameException("The base frame of the dynamical system is empty.");
  }
  if (!this->is_compatible(state)) {
    throw state_representation::exceptions::IncompatibleReferenceFramesException(
        "The evaluated state " + state.get_name() + " is incompatible with the base frame of the dynamical system "
            + this->get_base_frame().get_name() + "."
    );
  }
  return this->compute_dynamics(state);
}

template<class S>
void IDynamicalSystem<S>::assert_parameter_valid(const std::string& name, StateType state_type) {
  if (this->param_map_.at(name)->get_type() != state_type) {
    throw exceptions::InvalidParameterException("Parameter '" + name + "'exists, but has unexpected type.");
  }
}

template void IDynamicalSystem<CartesianState>::assert_parameter_valid(const std::string&, StateType);
template void IDynamicalSystem<JointState>::assert_parameter_valid(const std::string&, StateType);

template<class S>
S IDynamicalSystem<S>::get_base_frame() const {
  return this->base_frame_;
}

template CartesianState IDynamicalSystem<CartesianState>::get_base_frame() const;
template JointState IDynamicalSystem<JointState>::get_base_frame() const;

template<class S>
void IDynamicalSystem<S>::set_base_frame(const S& base_frame) {
  this->base_frame_ = base_frame;
}

template<class S>
std::shared_ptr<ParameterInterface> IDynamicalSystem<S>::get_parameter(const std::string& name) {
  if (this->param_map_.find(name) == this->param_map_.cend()) {
    throw exceptions::InvalidParameterException("Could not find a parameter named '" + name + "'.");
  }
  return this->param_map_.at(name);
}

template<class S>
std::map<std::string, std::shared_ptr<ParameterInterface>> IDynamicalSystem<S>::get_parameters() const {
  return this->param_map_;
}

template std::map<std::string, std::shared_ptr<ParameterInterface>>
IDynamicalSystem<CartesianState>::get_parameters() const;
template std::map<std::string, std::shared_ptr<ParameterInterface>>
IDynamicalSystem<JointState>::get_parameters() const;

template<class S>
template<typename T>
T IDynamicalSystem<S>::get_parameter_value(const std::string& name) {
  return std::static_pointer_cast<Parameter<T>>(this->get_parameter(name))->get_value();
}

template CartesianPose IDynamicalSystem<CartesianState>::get_parameter_value(const std::string&);

template<class S>
std::list<std::shared_ptr<ParameterInterface>> IDynamicalSystem<S>::get_parameter_list() const {
  std::list<std::shared_ptr<ParameterInterface>> param_list;
  for (const auto& param_it: this->param_map_) {
    param_list.template emplace_back(param_it.second);
  }
  return param_list;
}

template std::list<std::shared_ptr<ParameterInterface>> IDynamicalSystem<CartesianState>::get_parameter_list() const;
template std::list<std::shared_ptr<ParameterInterface>> IDynamicalSystem<JointState>::get_parameter_list() const;

template<class S>
void IDynamicalSystem<S>::set_parameter(const std::shared_ptr<ParameterInterface>& parameter) {
  this->validate_and_set_parameter(parameter);
}

template void IDynamicalSystem<CartesianState>::set_parameter(const std::shared_ptr<ParameterInterface>&);
template void IDynamicalSystem<JointState>::set_parameter(const std::shared_ptr<ParameterInterface>&);

template<class S>
void IDynamicalSystem<S>::set_parameters(const std::list<std::shared_ptr<ParameterInterface>>& parameters) {
  for (const auto& param: parameters) {
    this->set_parameter(param);
  }
}

template void IDynamicalSystem<CartesianState>::set_parameters(const std::list<std::shared_ptr<ParameterInterface>>&);
template void IDynamicalSystem<JointState>::set_parameters(const std::list<std::shared_ptr<ParameterInterface>>&);

template<class S>
void IDynamicalSystem<S>::set_parameters(const std::map<std::string, std::shared_ptr<ParameterInterface>>& parameters) {
  for (const auto& param_it: parameters) {
    this->set_parameter(param_it.second);
  }
}

template void IDynamicalSystem<CartesianState>::set_parameters(const std::map<std::string, std::shared_ptr<ParameterInterface>>&);
template void IDynamicalSystem<JointState>::set_parameters(const std::map<std::string, std::shared_ptr<ParameterInterface>>&);
}// namespace dynamical_systems
