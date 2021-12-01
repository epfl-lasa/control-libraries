#pragma once

#include <list>
#include <map>
#include <memory>

#include "dynamical_systems/exceptions/InvalidParameterException.hpp"
#include "state_representation/parameters/Parameter.hpp"
#include "state_representation/space/cartesian/CartesianPose.hpp"
#include "state_representation/space/cartesian/CartesianState.hpp"

/**
 * @namespace dynamical_systems
 * @brief Systems of equations relating state variables to their derivatives
 */
namespace dynamical_systems {

/**
 * @class IDynamicalSystem
 * @brief Abstract class for a dynamical system.
 * @tparam S Underlying state type of the dynamical system
 */
template<class S>
class IDynamicalSystem {
public:
  /**
   * @brief Empty constructor
   */
  IDynamicalSystem();

  /**
   * @brief Check compatibility between a state and the dynamical system.
   * @param state The state to check for compatibility
   * @return True if the state is compatible with the dynamical system
   */
  [[nodiscard]] virtual bool is_compatible(const S& state) const;

  /**
   * @brief Evaluate the value of the dynamical system at a given state.
   * @param state State at which to perform the evaluation
   * @return The resulting state (velocity) of the dynamical system
   */
  [[nodiscard]] S evaluate(const S& state) const;

  /**
   * @brief Return the base frame of the dynamical system.
   * @return The base frame
   */
  [[nodiscard]] S get_base_frame() const;

  /**
   * @brief Set the base frame of the dynamical system.
   * @param base_frame The new base frame
   */
  virtual void set_base_frame(const S& base_frame);

  /**
   * @brief Get a parameter of the dynamical system by its name.
   * @param name The name of the parameter
   * @return The parameter, if it exists
   */
  std::shared_ptr<state_representation::ParameterInterface> get_parameter(const std::string& name);

  /**
   * @brief Get a map of all the <name, parameter> pairs of the dynamical system.
   * @return The map of parameters
   */
  [[nodiscard]] std::map<std::string, std::shared_ptr<state_representation::ParameterInterface>> get_parameters() const;

  /**
 * @brief Get a parameter of the dynamical system by its name.
 * @tparam T Type of the parameter value
 * @param name The name of the parameter
 * @return The value of the parameter, if the parameter exists
 */
  template<typename T>
  T get_parameter_value(const std::string& name);

  /**
   * @brief Get a list of all the parameters of the dynamical system.
   * @return The list of parameters
   */
  [[nodiscard]] std::list<std::shared_ptr<state_representation::ParameterInterface>> get_parameter_list() const;

  /**
   * @brief Set a parameter of the dynamical system.
   * @param parameter The new parameter
   */
  void set_parameter(const std::shared_ptr<state_representation::ParameterInterface>& parameter);

  /**
   * @brief Set parameters of the dynamical list from a list of parameters.
   * @param parameters The list of parameters
   */
  void set_parameters(const std::list<std::shared_ptr<state_representation::ParameterInterface>>& parameters);

  /**
   * @brief Set parameters of the dynamical system from a map with <name, parameter> pairs.
   * @param parameters The map of parameters
   */
  void
  set_parameters(const std::map<std::string, std::shared_ptr<state_representation::ParameterInterface>>& parameters);

protected:
  /**
   * @brief Compute the dynamics of the input state. Internal function,
   * to be redefined based on the type of dynamical system, called
   * by the evaluate function.
   * @param state The input state
   * @return The output state
   */
  [[nodiscard]] virtual S compute_dynamics(const S& state) const = 0;

  /**
   * @brief Check if a parameter has the expected type, throw an exception otherwise.
   * @param parameter The parameter to be validated
   */
  void assert_parameter_valid(const std::shared_ptr<state_representation::ParameterInterface>& parameter);

  /**
   * @brief Validate and set a parameter of the dynamical system.
   * @details Internal function, to be redefined based on the
   * dynamical system, called by the set_parameter function.
   * @param parameter The parameter to be validated
   */
  virtual void
  validate_and_set_parameter(const std::shared_ptr<state_representation::ParameterInterface>& parameter) = 0;

  std::map<std::string, std::shared_ptr<state_representation::ParameterInterface>>
      param_map_; ///< map containing the names and values of all parameters of the dynamical system

private:
  S base_frame_; ///< frame in which the dynamical system is expressed
};

template<class S>
void IDynamicalSystem<S>::assert_parameter_valid(const std::shared_ptr<state_representation::ParameterInterface>& parameter) {
  if (this->param_map_.at(parameter->get_name())->get_type() != parameter->get_type()) {
    throw dynamical_systems::exceptions::InvalidParameterException(
        "Parameter '" + parameter->get_name() + "'exists, but has unexpected type."
    );
  }
}

template<class S>
S IDynamicalSystem<S>::get_base_frame() const {
  return this->base_frame_;
}

template<class S>
std::shared_ptr<state_representation::ParameterInterface> IDynamicalSystem<S>::get_parameter(const std::string& name) {
  if (this->param_map_.find(name) == this->param_map_.cend()) {
    throw exceptions::InvalidParameterException("Could not find a parameter named '" + name + "'.");
  }
  return this->param_map_.at(name);
}

template<class S>
template<typename T>
T IDynamicalSystem<S>::get_parameter_value(const std::string& name) {
  return std::static_pointer_cast<state_representation::Parameter<T>>(this->get_parameter(name))->get_value();
}

template<class S>
std::map<std::string, std::shared_ptr<state_representation::ParameterInterface>>
IDynamicalSystem<S>::get_parameters() const {
  return this->param_map_;
}

template<class S>
std::list<std::shared_ptr<state_representation::ParameterInterface>> IDynamicalSystem<S>::get_parameter_list() const {
  std::list<std::shared_ptr<state_representation::ParameterInterface>> param_list;
  for (const auto& param_it: this->param_map_) {
    param_list.template emplace_back(param_it.second);
  }
  return param_list;
}

template<class S>
void IDynamicalSystem<S>::set_parameter(const std::shared_ptr<state_representation::ParameterInterface>& parameter) {
  this->validate_and_set_parameter(parameter);
}

template<class S>
void IDynamicalSystem<S>::set_parameters(
    const std::list<std::shared_ptr<state_representation::ParameterInterface>>& parameters
) {
  for (const auto& param: parameters) {
    this->set_parameter(param);
  }
}

template<class S>
void IDynamicalSystem<S>::set_parameters(
    const std::map<std::string, std::shared_ptr<state_representation::ParameterInterface>>& parameters
) {
  for (const auto& param_it: parameters) {
    this->set_parameter(param_it.second);
  }
}

}// namespace dynamical_systems
