#include "state_representation/parameters/ParameterInterface.hpp"

namespace state_representation {

ParameterInterface::ParameterInterface(const StateType& type, const std::string& name) : State(type, name) {}

ParameterInterface::ParameterInterface(const ParameterInterface& parameter) : State(parameter) {}

inline ParameterInterface& ParameterInterface::operator=(const ParameterInterface& state) {
  State::operator=(state);
  return (*this);
}
}// namespace state_representation
