#pragma once

#include "state_representation/State.hpp"

namespace StateRepresentation {
class ParameterInterface : public State {
public:
  /**
   * @brief Constructor with parameter name and type of the parameter
   * @param type the type of the parameter
   * @param name the name of the parameter
   */
  explicit ParameterInterface(const StateType& type, const std::string& name);

  /**
   * @brief Copy constructor
   * @param parameter the parameter to copy
   */
  ParameterInterface(const ParameterInterface& parameter);

  /**
   * @brief Copy assignement operator that have to be defined to the custom assignement operator
   * @param state the state with value to assign
   * @return reference to the current state with new values
   */
  ParameterInterface& operator=(const ParameterInterface& state);
};

inline ParameterInterface& ParameterInterface::operator=(const ParameterInterface& state) {
  State::operator=(state);
  return (*this);
}
}// namespace StateRepresentation
