#pragma once

#include <memory>

#include "state_representation/parameters/ParameterInterface.hpp"

namespace state_representation {

template<typename T>
class Parameter : public ParameterInterface {
private:
  T value_;///< Value of the parameter

public:
  /**
   * @brief Constructor with name of the parameter.
   * @param name The name of the parameter
   */
  explicit Parameter(const std::string& name);

  /**
   * @brief Constructor with a name and a value.
   * @param name The name of the parameter
   * @param value The value of the parameter
   */
  explicit Parameter(const std::string& name, const T& value);

  /**
   * @brief Copy constructor
   * @param parameter The parameter to copy
   */
  template<typename U>
  Parameter(const Parameter<U>& parameter);

  /**
   * @brief Conversion equality
   */
  template<typename U>
  Parameter<T>& operator=(const Parameter<U>& parameter);

  /**
   * @brief Getter of the value attribute.
   * @return The value attribute
   */
  const T& get_value() const;

  /**
   * @brief Getter of the value attribute.
   * @return The value attribute
   */
  T& get_value();

  /**
   * @brief Setter of the value attribute.
   * @param The new value attribute
   */
  virtual void set_value(const T& value);

  /**
   * @brief Overload the ostream operator for printing.
   * @param os The ostream to append the string representing the State to
   * @param parameter The Parameter to print
   * @return The appended ostream
   */
  template<typename U>
  friend std::ostream& operator<<(std::ostream& os, const Parameter<U>& parameter);
};

template<typename T>
template<typename U>
Parameter<T>::Parameter(const Parameter<U>& parameter) : ParameterInterface(parameter), value_(parameter.get_value()) {}

template<typename T>
template<typename U>
Parameter<T>& Parameter<T>::operator=(const Parameter<U>& parameter) {
  Parameter<T> temp(parameter);
  *this = temp;
  return *this;
}

template<typename T>
inline const T& Parameter<T>::get_value() const {
  return this->value_;
}

template<typename T>
inline T& Parameter<T>::get_value() {
  return this->value_;
}

template<typename T>
inline void Parameter<T>::set_value(const T& value) {
  this->set_filled();
  this->value_ = value;
}

template<typename T>
static std::shared_ptr<Parameter<T>> make_shared_parameter(const std::string& name, const T& param_value) {
  return std::make_shared<Parameter<T>>(name, param_value);
}
}// namespace state_representation
