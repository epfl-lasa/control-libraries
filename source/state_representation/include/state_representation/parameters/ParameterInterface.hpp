#pragma once

#include "state_representation/State.hpp"

namespace state_representation {

class ParameterInterface : public State {
public:
  /**
   * @brief Constructor with parameter name and type of the parameter.
   * @param type The type of the parameter
   * @param name The name of the parameter
   */
  explicit ParameterInterface(const StateType& type, const std::string& name);

  /**
   * @brief Copy constructor
   * @param parameter The parameter to copy
   */
  ParameterInterface(const ParameterInterface& parameter);

  /**
   * @brief Copy assignment operator that has to be defined
   * to the custom assignment operator.
   * @param state The state with value to assign
   * @return Reference to the current state with new values
   */
  ParameterInterface& operator=(const ParameterInterface& state);
};
}// namespace state_representation
