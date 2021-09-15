#pragma once

#include "state_representation/MathTools.hpp"
#include <cassert>
#include <chrono>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <typeinfo>

/**
 * @namespace state_representation
 * @brief Core state variables and objects
 */
namespace state_representation {
enum class StateType {
  STATE,
  CARTESIANSTATE,
  DUALQUATERNIONSTATE,
  JOINTSTATE,
  JACOBIANMATRIX,
  TRAJECTORY,
  GEOMETRY_SHAPE,
  GEOMETRY_ELLIPSOID,
  PARAMETER_INT,
  PARAMETER_INT_ARRAY,
  PARAMETER_DOUBLE,
  PARAMETER_DOUBLE_ARRAY,
  PARAMETER_BOOL,
  PARAMETER_BOOL_ARRAY,
  PARAMETER_STRING,
  PARAMETER_STRING_ARRAY,
  PARAMETER_CARTESIANSTATE,
  PARAMETER_CARTESIANPOSE,
  PARAMETER_JOINTSTATE,
  PARAMETER_JOINTPOSITIONS,
  PARAMETER_ELLIPSOID,
  PARAMETER_MATRIX,
  PARAMETER_VECTOR
};

/**
 * @class State
 * @brief Abstract class to represent a state
 */
class State {
private:
  StateType type_;                                              ///< type of the State
  std::string name_;                                            ///< name of the state
  bool empty_;                                                  ///< indicate if the state is empty
  std::chrono::time_point<std::chrono::steady_clock> timestamp_;///< time since last modification made to the state

public:
  /**
   * @brief Empty constructor
   */
  explicit State();

  /**
   * @brief Constructor only specifying the type of the state from the StateType enumeration
   * @param type the type of State 
   */
  explicit State(const StateType& type);

  /**
   * @brief Constructor with name specification
   * @param type the type of State
   * @param name the name of the State
   * @param empty specify if the state is initialized as empty, default true
   */
  explicit State(const StateType& type, const std::string& name, const bool& empty = true);

  /**
   * @brief Copy constructor from another State
   */
  State(const State& state);

  /**
   * @brief Swap the values of the two States
   * @param state1 State to be swapped with 2
   * @param state2 State to be swapped with 1
   */
  friend void swap(State& state1, State& state2);

  /**
   * @brief Copy assignment operator that have to be defined to the custom assignment operator
   * @param state the state with value to assign
   * @return reference to the current state with new values
   */
  State& operator=(const State& state);

  /**
   * @brief Getter of the type attribute
   * @return the type of the State
   */
  const StateType& get_type() const;

  /**
   * @brief Getter of the empty attribute
   */
  bool is_empty() const;

  /**
   * @brief Setter of the empty attribute
   * @param empty bool if the state should be empty or not (default is true)
   */
  void set_empty(bool empty = true);

  /**
   * @brief Setter of the empty attribute to false and also reset the timestamp
   */
  void set_filled();

  /**
   * @brief Getter of the timestamp attribute
   */
  const std::chrono::time_point<std::chrono::steady_clock>& get_timestamp() const;

  /**
   * @brief Reset the timestamp attribute to now
   */
  void reset_timestamp();

  /**
   * @brief Getter of the name as const reference
   */
  const std::string& get_name() const;

  /**
   * @brief Setter of the name
   */
  virtual void set_name(const std::string& name);

  /**
   * @brief Check if the state is deprecated given a certain time delay
   * @param time_delay the time after which to consider the state as deprecated
   */
  template<typename DurationT>
  bool is_deprecated(const std::chrono::duration<int64_t, DurationT>& time_delay);

  /**
   * @brief Check if the state is compatible for operations with the state given as argument
   * @param state the state to check compatibility with
   */
  virtual bool is_compatible(const State& state) const;

  /**
   * @brief Initialize the State to a zero value
   */
  virtual void initialize();

  /**
   * @brief Set the data of the state from a single Eigen vector
   * @param the data vector
   */
  virtual void set_data(const Eigen::VectorXd& data);

  /**
   * @brief Set the data of the state from a single std vector
   * @param the data vector
   */
  virtual void set_data(const std::vector<double>& data);

  /**
   * @brief Set the data of the state from an Eigen matrix
   * @param the data vector
   */
  virtual void set_data(const Eigen::MatrixXd& data);

  /**
   * @brief Overload the ostream operator for printing
   * @param os the ostream to append the string representing the State to
   * @param state the State to print
   * @return the appended ostream 
   */
  friend std::ostream& operator<<(std::ostream& os, const State& state);
};

inline void swap(State& state1, State& state2) {
  std::swap(state1.type_, state2.type_);
  std::swap(state1.name_, state2.name_);
  std::swap(state1.empty_, state2.empty_);
  std::swap(state1.timestamp_, state2.timestamp_);
}

inline State& State::operator=(const State& state) {
  State tmp(state);
  swap(*this, tmp);
  return *this;
}

inline const StateType& State::get_type() const {
  return this->type_;
}

inline bool State::is_empty() const {
  return this->empty_;
}

inline void State::set_empty(bool empty) {
  this->empty_ = empty;
}

inline void State::set_filled() {
  this->empty_ = false;
  this->reset_timestamp();
}

inline const std::chrono::time_point<std::chrono::steady_clock>& State::get_timestamp() const {
  return this->timestamp_;
}

inline void State::reset_timestamp() {
  this->timestamp_ = std::chrono::steady_clock::now();
}

template<typename DurationT>
inline bool State::is_deprecated(const std::chrono::duration<int64_t, DurationT>& time_delay) {
  return ((std::chrono::steady_clock::now() - this->timestamp_) > time_delay);
}

inline const std::string& State::get_name() const {
  return this->name_;
}

inline void State::set_name(const std::string& name) {
  this->name_ = name;
}

inline bool State::is_compatible(const State& state) const {
  bool compatible = (this->name_ == state.name_);
  return compatible;
}

inline void State::initialize() {
  this->empty_ = true;
}
}// namespace state_representation
