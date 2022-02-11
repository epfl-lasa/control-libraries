#pragma once

#include "state_representation/exceptions/InvalidParameterException.hpp"
#include "state_representation/parameters/Parameter.hpp"

namespace state_representation {

/**
 * @class ParameterMap
 * @brief A wrapper class to contain a map of Parameter pointers by name and provide robust access methods
 */
class ParameterMap {
public:

  /**
   * @brief Empty constructor
   */
  ParameterMap() = default;

  /**
   * @brief Get a parameter by its name.
   * @param name The name of the parameter
   * @return The parameter, if it exists
   */
  [[nodiscard]] std::shared_ptr<state_representation::ParameterInterface> get_parameter(const std::string& name) const;

  /**
   * @brief Get a map of all the <name, parameter> pairs.
   * @return The map of parameters
   */
  [[nodiscard]] std::map<std::string, std::shared_ptr<state_representation::ParameterInterface>> get_parameters() const;

  /**
   * @brief Get a parameter value by its name.
   * @tparam T Type of the parameter value
   * @param name The name of the parameter
   * @return The value of the parameter, if the parameter exists
   */
  template<typename T>
  [[nodiscard]] T get_parameter_value(const std::string& name) const;

  /**
   * @brief Get a list of all the parameters.
   * @return The list of parameters
   */
  [[nodiscard]] std::list<std::shared_ptr<state_representation::ParameterInterface>> get_parameter_list() const;

  /**
   * @brief Set a parameter.
   * @param parameter The new parameter
   */
  void set_parameter(const std::shared_ptr<state_representation::ParameterInterface>& parameter);

  /**
   * @brief Set parameters from a list of parameters.
   * @param parameters The list of parameters
   */
  void set_parameters(const std::list<std::shared_ptr<state_representation::ParameterInterface>>& parameters);

  /**
   * @brief Set parameters from a map with <name, parameter> pairs.
   * @param parameters The map of parameters
   */
  void
  set_parameters(const std::map<std::string, std::shared_ptr<state_representation::ParameterInterface>>& parameters);

  /**
   * @brief Set a parameter value by its name.
   * @tparam T Type of the parameter value
   * @param name The name of the parameter
   * @param value The new value of the parameter
   */
  template<typename T>
  void set_parameter_value(const std::string& name, const T& value);

protected:

  /**
   * @brief Validate and set a parameter in the map.
   * @details This virtual function should be redefined in any base class to provide proper validation of parameters
   * by name, value and type. In the base form, it allows any type of parameter name to be added to the map.
   * @param parameter The parameter to be validated
   */
  virtual void validate_and_set_parameter(const std::shared_ptr<state_representation::ParameterInterface>& parameter);

  /**
   * @brief Check if a parameter has the expected type, throw an exception otherwise.
   * @param parameter The parameter to be validated
   */
  void assert_parameter_valid(const std::shared_ptr<state_representation::ParameterInterface>& parameter);

  std::map<std::string, std::shared_ptr<state_representation::ParameterInterface>>
      parameters_; ///< map of parameters by name

};

inline std::shared_ptr<state_representation::ParameterInterface>
ParameterMap::get_parameter(const std::string& name) const {
  if (this->parameters_.find(name) == this->parameters_.cend()) {
    throw exceptions::InvalidParameterException("Could not find a parameter named '" + name + "'.");
  }
  return this->parameters_.at(name);
}

template<typename T>
inline T ParameterMap::get_parameter_value(const std::string& name) const {
  return std::static_pointer_cast<state_representation::Parameter<T>>(this->get_parameter(name))->get_value();
}

inline std::map<std::string, std::shared_ptr<state_representation::ParameterInterface>>
ParameterMap::get_parameters() const {
  return this->parameters_;
}

inline std::list<std::shared_ptr<state_representation::ParameterInterface>> ParameterMap::get_parameter_list() const {
  std::list<std::shared_ptr<state_representation::ParameterInterface>> param_list;
  for (const auto& param_it: this->parameters_) {
    param_list.template emplace_back(param_it.second);
  }
  return param_list;
}

inline void ParameterMap::set_parameter(const std::shared_ptr<state_representation::ParameterInterface>& parameter) {
  this->validate_and_set_parameter(parameter);
}

inline void ParameterMap::set_parameters(
    const std::list<std::shared_ptr<state_representation::ParameterInterface>>& parameters
) {
  for (const auto& param: parameters) {
    this->set_parameter(param);
  }
}

inline void ParameterMap::set_parameters(
    const std::map<std::string, std::shared_ptr<state_representation::ParameterInterface>>& parameters
) {
  for (const auto& param_it: parameters) {
    this->set_parameter(param_it.second);
  }
}

template<typename T>
inline void ParameterMap::set_parameter_value(const std::string& name, const T& value) {
  using namespace state_representation;
  this->validate_and_set_parameter(std::make_shared<Parameter<T>>(Parameter<T>(name, value)));
}

inline void ParameterMap::assert_parameter_valid(
    const std::shared_ptr<state_representation::ParameterInterface>& parameter
) {
  if (this->parameters_.at(parameter->get_name())->get_type() != parameter->get_type()) {
    throw exceptions::InvalidParameterException(
        "Parameter '" + parameter->get_name() + "' exists, but has unexpected type.");
  }
}

inline void
ParameterMap::validate_and_set_parameter(const std::shared_ptr<state_representation::ParameterInterface>& parameter) {
  this->parameters_[parameter->get_name()] = parameter;
}

}