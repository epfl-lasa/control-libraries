#pragma once

#include <list>
#include <map>
#include <memory>

#include "state_representation/space/cartesian/CartesianPose.hpp"
#include "state_representation/space/cartesian/CartesianState.hpp"
#include "state_representation/parameters/ParameterInterface.hpp"

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
   * @brief Check if a parameter with provided name exists, throw an
   * exception otherwise.
   * @param name The name of the parameter
   */
  void assert_parameter_exists(const std::string& name);

  /**
   * @brief Check if a parameter with provided name exists and has the
   * expected type, throw an exception otherwise.
   * @param name The name of the parameter
   * @param state_type The type of the parameter
   */
  void assert_parameter_valid(const std::string& name, state_representation::StateType state_type);

  std::map<std::string, std::shared_ptr<state_representation::ParameterInterface>>
      param_map_; ///> map containing the names and values of all parameters of the dynamical system

private:
  S base_frame_; ///< frame in which the dynamical system is expressed
};
}// namespace dynamical_systems
