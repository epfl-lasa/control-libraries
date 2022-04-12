#include "state_representation/parameters/ParameterInterface.hpp"

namespace state_representation {

ParameterInterface::ParameterInterface(
    const std::string& name, const ParameterType& type, const StateType& parameter_state_type
) : State(StateType::PARAMETER, name), parameter_type_(type), parameter_state_type_(parameter_state_type) {}

ParameterInterface::ParameterInterface(const ParameterInterface& parameter) :
    State(parameter),
    parameter_type_(parameter.get_parameter_type()),
    parameter_state_type_(parameter.get_parameter_state_type()) {}

ParameterInterface& ParameterInterface::operator=(const ParameterInterface& state) {
  State::operator=(state);
  this->parameter_type_ = state.get_parameter_type();
  this->parameter_state_type_ = state.get_parameter_state_type();
  return (*this);
}

ParameterType ParameterInterface::get_parameter_type() const {
  return parameter_type_;
}

StateType ParameterInterface::get_parameter_state_type() const {
  return parameter_state_type_;
}

}// namespace state_representation
