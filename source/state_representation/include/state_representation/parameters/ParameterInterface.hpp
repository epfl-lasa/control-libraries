#pragma once

#include <memory>

#include "state_representation/parameters/ParameterType.hpp"
#include "state_representation/exceptions/InvalidParameterCastException.hpp"
#include "state_representation/exceptions/InvalidPointerException.hpp"
#include "state_representation/State.hpp"

namespace state_representation {

// forward declaration of derived Parameter class
template<typename T>
class Parameter;

class ParameterInterface : public State {
public:
  /**
   * @brief Constructor with parameter name and type.
   * @param name The name of the parameter
   * @param type The type of the parameter
   * @param parameter_state_type The state type of the parameter, if applicable
   */
  ParameterInterface(
      const std::string& name, const ParameterType& type, const StateType& parameter_state_type = StateType::NONE
  );

  /**
   * @brief Copy constructor
   * @param parameter The parameter to copy
   */
  ParameterInterface(const ParameterInterface& parameter);

  /**
   * @brief Default virtual destructor
   */
  virtual ~ParameterInterface() = default;

  /**
   * @brief Copy assignment operator that has to be defined
   * to the custom assignment operator.
   * @param state The state with value to assign
   * @return Reference to the current state with new values
   */
  ParameterInterface& operator=(const ParameterInterface& state);

  /**
   * @brief Get a pointer to a derived Parameter instance from a ParameterInterface pointer.
   * @details If a ParameterInterface pointer is used to address a derived Parameter instance,
   * this method will return a pointer to that derived instance through dynamic down-casting.
   * The downcast will fail if the base ParameterInterface object has no reference count
   * (if the object is not owned by any pointer), or if the derived object is not a correctly
   * typed instance of a Parameter. By default, an InvalidParameterCastException is thrown when
   * the downcast fails. If this validation is disabled by setting the validate_pointer flag to false,
   * it will not throw an exception and instead return a null pointer.
   * @tparam T The state type of the Parameter
   * @param validate_pointer If true, throw an exception when downcasting fails
   * @return A pointer to a derived Parameter instance of the desired state type, or a null pointer
   * if downcasting failed and validate_pointer was set to false.
   */
  template<typename T>
  std::shared_ptr<Parameter<T>> get_parameter(bool validate_pointer = true) const;

  /**
   * @brief Get the parameter value of a derived Parameter instance through the ParameterInterface pointer.
   * @details This throws an InvalidParameterCastException if the ParameterInterface does not point to
   * a valid Parameter instance or if the specified type does not match the type of the Parameter instance.
   * @see ParameterInterface::get_parameter()
   * @tparam T The state type of the Parameter
   * @return The value contained in the underlying Parameter instance
   */
  template<typename T>
  T get_parameter_value() const;

  /**
   * @brief Set the parameter value of a derived Parameter instance through the ParameterInterface pointer.
   * @details This throws an InvalidParameterCastException if the ParameterInterface does not point to
   * a valid Parameter instance or if the specified type does not match the type of the Parameter instance.
   * @see ParameterInterface::get_parameter()
   * @tparam T The state type of the Parameter
   * @param value The value to set in the underlying Parameter instance
   */
  template<typename T>
  void set_parameter_value(const T& value);

  /**
   * @brief Get the parameter type.
   * @return The type of the underlying parameter
   */
  ParameterType get_parameter_type() const;

  /**
   * @brief Get the state type of the parameter.
   * @details If the parameter type from get_parameter_type() is not ParameterType::STATE,
   * this will return StateType::NONE.
   * @return The state type of the underlying parameter
   */
  StateType get_parameter_state_type() const;

private:
  ParameterType parameter_type_;
  StateType parameter_state_type_;
};

template<typename T>
inline std::shared_ptr<Parameter<T>> ParameterInterface::get_parameter(bool validate_pointer) const {
  std::shared_ptr<Parameter<T>> parameter_ptr;
  try {
    parameter_ptr = std::dynamic_pointer_cast<Parameter<T>>(std::const_pointer_cast<State>(shared_from_this()));
  } catch (const std::exception&) {
    if (validate_pointer) {
      throw exceptions::InvalidPointerException(
          "Parameter interface \"" + get_name() + "\" is not managed by a valid pointer");
    }
  }
  if (parameter_ptr == nullptr && validate_pointer) {
    std::string type_name(typeid(T).name());
    throw exceptions::InvalidParameterCastException(
        "Unable to cast parameter interface \"" + get_name() + "\" to a parameter pointer of requested type "
            + type_name);
  }
  return parameter_ptr;
}

template<typename T>
inline T ParameterInterface::get_parameter_value() const {
  return get_parameter<T>(true)->get_value();
}

template<typename T>
inline void ParameterInterface::set_parameter_value(const T& value) {
  get_parameter<T>(true)->set_value(value);
}

}// namespace state_representation
