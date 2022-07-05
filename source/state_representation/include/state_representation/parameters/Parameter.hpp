#pragma once

#include <memory>

#include "state_representation/parameters/ParameterInterface.hpp"
#include "state_representation/geometry/Ellipsoid.hpp"
#include "state_representation/space/cartesian/CartesianPose.hpp"
#include "state_representation/space/joint/JointPositions.hpp"

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
   * @brief Default virtual destructor
   */
  virtual ~Parameter() = default;

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
   * @tparam U The expected type of the parameter
   * @return The value attribute
   */
  template<typename U>
  U get_value() const;

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
Parameter<T>::Parameter(const Parameter<U>& parameter) :
    Parameter<T>(parameter.get_name(), static_cast<T>(parameter.get_value())) {}

template<typename T>
template<typename U>
Parameter<T>& Parameter<T>::operator=(const Parameter<U>& parameter) {
  Parameter<T> temp(parameter);
  *this = temp;
  return *this;
}

template<typename T>
template<typename U>
U Parameter<T>::get_value() const {
  return static_cast<U>(this->value_);
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

[[maybe_unused]] static std::shared_ptr<ParameterInterface> make_shared_parameter_interface(
    const std::string& name, const ParameterType& type, const StateType& parameter_state_type = StateType::NONE
) {
  switch (type) {
    case ParameterType::BOOL:
      return std::make_shared<Parameter<bool>>(name);
    case ParameterType::BOOL_ARRAY:
      return std::make_shared<Parameter<std::vector<bool>>>(name);
    case ParameterType::INT:
      return std::make_shared<Parameter<int>>(name);
    case ParameterType::INT_ARRAY:
      return std::make_shared<Parameter<std::vector<int>>>(name);
    case ParameterType::DOUBLE:
      return std::make_shared<Parameter<double>>(name);
    case ParameterType::DOUBLE_ARRAY:
      return std::make_shared<Parameter<std::vector<double>>>(name);
    case ParameterType::STRING:
      return std::make_shared<Parameter<std::string>>(name);
    case ParameterType::STRING_ARRAY:
      return std::make_shared<Parameter<std::vector<std::string>>>(name);
    case ParameterType::STATE: {
      switch (parameter_state_type) {
        case StateType::CARTESIAN_STATE:
          return std::make_shared<Parameter<CartesianState>>(name);
        case StateType::CARTESIAN_POSE:
          return std::make_shared<Parameter<CartesianPose>>(name);
        case StateType::JOINT_STATE:
          return std::make_shared<Parameter<JointState>>(name);
        case StateType::JOINT_POSITIONS:
          return std::make_shared<Parameter<JointPositions>>(name);
        case StateType::GEOMETRY_ELLIPSOID:
          return std::make_shared<Parameter<Ellipsoid>>(name);
        default:
          throw exceptions::InvalidParameterException("This StateType is not supported for parameters.");
      }
    }
    case ParameterType::VECTOR:
      return std::make_shared<Parameter<Eigen::VectorXd>>(name);
    case ParameterType::MATRIX:
      return std::make_shared<Parameter<Eigen::MatrixXd>>(name);
  }
}
}// namespace state_representation
