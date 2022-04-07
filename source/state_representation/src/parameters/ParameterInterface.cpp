#include "state_representation/parameters/ParameterInterface.hpp"

namespace state_representation {

ParameterInterface::ParameterInterface(const ParameterType& type, const std::string& name) :
    State(StateType::PARAMETER, name), parameter_type_(type), parameter_state_type_(StateType::NONE) {}

ParameterInterface::ParameterInterface(const StateType& parameter_state_type, const std::string& name) :
    State(StateType::PARAMETER, name),
    parameter_type_(ParameterType::STATE),
    parameter_state_type_(parameter_state_type) {}

ParameterInterface::ParameterInterface(const ParameterInterface& parameter) : State(parameter) {}

ParameterInterface& ParameterInterface::operator=(const ParameterInterface& state) {
  State::operator=(state);
  return (*this);
}

ParameterType ParameterInterface::get_parameter_type() const {
  return parameter_type_;
}

StateType ParameterInterface::get_parameter_state_type() const {
  return parameter_state_type_;
}

}// namespace state_representation
