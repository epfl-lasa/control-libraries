#pragma once

#include "state_representation/State.hpp"
#include "state_representation/exceptions/IncompatibleSizeException.hpp"

using namespace state_representation::exceptions;

namespace state_representation {
class JointState;

/**
 * @enum JointStateVariable
 * @brief Enum representing all the fields (positions, velocities, accelerations and torques)
 * of the JointState
 */
enum class JointStateVariable {
  POSITIONS, VELOCITIES, ACCELERATIONS, TORQUES, ALL
};

/**
 * @brief Compute the distance between two JointState.
 * @param s1 The first JointState
 * @param s2 The second JointState
 * @param state_variable_type Name of the field from the JointStateVariable structure to apply
 * the distance on (default ALL for full distance across all dimensions)
 * @return The distance between the two states
 */
double dist(
    const JointState& s1, const JointState& s2, const JointStateVariable& state_variable_type = JointStateVariable::ALL
);

/**
 * @class JointState
 * @brief Class to define a state in joint space.
 */
class JointState : public State {
private:
  std::vector<std::string> names_;///< names of the joints
  Eigen::VectorXd positions_;     ///< joints positions
  Eigen::VectorXd velocities_;    ///< joints velocities
  Eigen::VectorXd accelerations_; ///< joints accelerations
  Eigen::VectorXd torques_;       ///< joints torques

  /**
   * @brief Getter of all the state variables (positions, velocities, accelerations and torques).
   * @return The concatenated vector of all the state variables
   */
  Eigen::VectorXd get_all_state_variables() const;

  /**
   * @brief Set new_value in the provided state_variable (positions, velocities, accelerations or torques).
   * @param state_variable The state variable to fill
   * @param new_value The new value of the state variable
   */
  void set_state_variable(Eigen::VectorXd& state_variable, const Eigen::VectorXd& new_value);

  /**
   * @brief Set new_value in the provided state_variable (positions, velocities, accelerations or torques).
   * @param state_variable The state variable to fill
   * @param new_value The new value of the state variable
   */
  void set_state_variable(Eigen::VectorXd& state_variable, const std::vector<double>& new_value);

  /**
   * @brief Set new_value in all the state variables (positions, velocities, accelerations and torques).
   * @param new_values The new values of the state variables
   */
  void set_all_state_variables(const Eigen::VectorXd& new_values);

protected:
  /**
   * @brief Proxy function that multiply the specified state variable by an array of gain.
   * @param lambda The gain array to multiply with
   * @param state_variable_type The state variable on which to apply the multiplication
   */
  void multiply_state_variable(const Eigen::ArrayXd& lambda, const JointStateVariable& state_variable_type);

  /**
   * @brief Proxy function that multiply the specified state variable by an array of gain.
   * @param lambda The gain array to multiply with
   * @param state_variable_type The state variable on which to apply the multiplication
   */
  void multiply_state_variable(const Eigen::MatrixXd& lambda, const JointStateVariable& state_variable_type);

  /**
   * @brief Getter of the variable value corresponding to the input.
   * @param state_variable_type The type of variable to get
   * @return The value of the variable as a vector
   */
  Eigen::VectorXd get_state_variable(const JointStateVariable& state_variable_type) const;

  /**
   * @brief Setter of the variable value corresponding to the input.
   * @param new_value The new value of the variable
   * @param state_variable_type The type of variable to get
   */
  void set_state_variable(const Eigen::VectorXd& new_value, const JointStateVariable& state_variable_type);

public:
  /**
   * @brief Empty constructor for a JointState.
   */
  explicit JointState();

  /**
   * @brief Constructor with name and number of joints provided.
   * @param robot_name The name of the associated robot
   * @param nb_joints The number of joints for initialization
   */
  explicit JointState(const std::string& robot_name, unsigned int nb_joints = 0);

  /**
   * @brief Constructor with name and list of joint names provided.
   * @param robot_name The name of the associated robot
   * @param joint_names List of joint names
   */
  explicit JointState(const std::string& robot_name, const std::vector<std::string>& joint_names);

  /**
   * @brief Copy constructor of a JointState.
   * @param state The JointState to copy from
   */
  JointState(const JointState& state) = default;

  /**
   * @brief Constructor for a zero JointState.
   * @param robot_name The name of the associated robot
   * @param nb_joints The number of joints for initialization
   * @return JointState with zero values in all attributes
   */
  static JointState Zero(const std::string& robot_name, unsigned int nb_joints);

  /**
   * @brief Constructor for a zero JointState.
   * @param robot_name The name of the associated robot
   * @param joint_names List of joint names
   * @return JointState with zero values in all attributes
   */
  static JointState Zero(const std::string& robot_name, const std::vector<std::string>& joint_names);

  /**
   * @brief Constructor for a random JointState.
   * @param robot_name The name of the associated robot
   * @param nb_joints The number of joints for initialization
   * @return JointState with random values in all attributes
   */
  static JointState Random(const std::string& robot_name, unsigned int nb_joints);

  /**
   * @brief Constructor for a random JointState.
   * @param robot_name The name of the associated robot
   * @param joint_names List of joint names
   * @return JointState with random values in all attributes
   */
  static JointState Random(const std::string& robot_name, const std::vector<std::string>& joint_names);

  /**
   * @brief Swap the values of the two JointState.
   * @param state1 JointState to be swapped with 2
   * @param state2 JointState to be swapped with 1
   */
  friend void swap(JointState& state1, JointState& state2);

  /**
   * @brief Copy assignment operator that have to be defined to the custom assignment operator.
   * @param state The state with value to assign
   * @return Reference to the current state with new values
   */
  JointState& operator=(const JointState& state);

  /**
   * @brief Getter of the size from the attributes.
   * @return The size of the joint state
   */
  unsigned int get_size() const;

  /**
   * @brief Getter of the names attribute.
   * @return The vector of strings containing the joint names
   */
  const std::vector<std::string>& get_names() const;

  /**
   * @brief Setter of the names attribute from the number of joints.
   * @param nb_joints The number of joints of the joint state
   */
  void set_names(unsigned int nb_joints);

  /**
   * @brief Setter of the names attribute.
   * @param names The vector of strings containing the joint names
   */
  void set_names(const std::vector<std::string>& names);

  /**
   * @brief Get joint index by the name of the joint, if it exists.
   * @details A JointNotFoundException exception is thrown if the
   * desired joint doesn't exist.
   * @param joint_name The name of the desired joint
   * @return The index of the joint, if it exists
   */
  unsigned int get_joint_index(const std::string& joint_name) const;

  /**
   * @brief Getter of the positions attribute.
   * @return The joint positions
   */
  const Eigen::VectorXd& get_positions() const;

  /**
   * @brief Get the position of a joint by its name, if it exists.
   * @copydetails JointState::get_joint_index()
   * @param joint_name The name of the joint
   * @return The position of the joint, if it exists
   */
  double get_position(const std::string& joint_name) const;

  /**
   * @brief Get the position of a joint by its index, if it exists.
   * @copydetails JointState::get_joint_index()
   * @param joint_index The index of the joint
   * @return The position of the joint, if it exists
   */
  double get_position(unsigned int joint_index) const;

  /**
   * @brief Setter of the positions attribute.
   * @param positions The joint positions as Eigen vector
   */
  void set_positions(const Eigen::VectorXd& positions);

  /**
   * @brief Setter of the positions from std vector.
   * @param positions The joint positions as std vector
   */
  void set_positions(const std::vector<double>& positions);

  /**
   * @brief Set the position of a joint by its name.
   * @param position The position of the joint
   * @param joint_name The name of the joint
   */
  void set_position(double position, const std::string& joint_name);

  /**
   * @brief Set the position of a joint by its index.
   * @param position The position of the joint
   * @param joint_index The name of the joint
   */
  void set_position(double position, unsigned int joint_index);

  /**
   * @brief Getter of the velocities attribute.
   * @return The joint velocities
   */
  const Eigen::VectorXd& get_velocities() const;

  /**
   * @brief Get the velocity of a joint by its name, if it exists.
   * @copydetails JointState::get_joint_index()
   * @param joint_name The name of the joint
   * @return The velocity of the joint, if it exists
   */
  double get_velocity(const std::string& joint_name) const;

  /**
   * @brief Get the velocity of a joint by its index, if it exists.
   * @copydetails JointState::get_joint_index()
   * @param joint_index The index of the joint
   * @return The velocity of the joint, if it exists
   */
  double get_velocity(unsigned int joint_index) const;

  /**
   * @brief Setter of the velocities attribute.
   * @param velocities The joint velocities as Eigen vector
   */
  void set_velocities(const Eigen::VectorXd& velocities);

  /**
   * @brief Setter of the velocities from std vector.
   * @param velocities The joint velocities as std vector
   */
  void set_velocities(const std::vector<double>& velocities);

  /**
   * @brief Set the velocity of a joint by its name.
   * @param velocity The velocity of the joint
   * @param joint_name The name of the joint
   */
  void set_velocity(double velocity, const std::string& joint_name);

  /**
   * @brief Set the velocity of a joint by its index.
   * @param velocity The velocity of the joint
   * @param joint_index The name of the joint
   */
  void set_velocity(double velocity, unsigned int joint_index);

  /**
   * @brief Getter of the accelerations attribute.
   * @return The joint accelerations
   */
  const Eigen::VectorXd& get_accelerations() const;

  /**
   * @brief Get the acceleration of a joint by its name, if it exists.
   * @copydetails JointState::get_joint_index()
   * @param joint_name The name of the joint
   * @return The acceleration of the joint, if it exists
   */
  double get_acceleration(const std::string& joint_name) const;

  /**
   * @brief Get the acceleration of a joint by its index, if it exists.
   * @copydetails JointState::get_joint_index()
   * @param joint_index The index of the joint
   * @return The acceleration of the joint, if it exists
   */
  double get_acceleration(unsigned int joint_index) const;

  /**
   * @brief Setter of the accelerations attribute.
   * @param accelerations The joint accelerations as Eigen vector
   */
  void set_accelerations(const Eigen::VectorXd& accelerations);

  /**
   * @brief Setter of the accelerations from std vector.
   * @param accelerations The joint accelerations as std vector
   */
  void set_accelerations(const std::vector<double>& accelerations);

  /**
   * @brief Set the acceleration of a joint by its name.
   * @param acceleration The acceleration of the joint
   * @param joint_name The name of the joint
   */
  void set_acceleration(double acceleration, const std::string& joint_name);

  /**
   * @brief Set the acceleration of a joint by its index.
   * @param acceleration The acceleration of the joint
   * @param joint_index The name of the joint
   */
  void set_acceleration(double acceleration, unsigned int joint_index);

  /**
   * @brief Getter of the torques attribute.
   * @return The joint torques
   */
  const Eigen::VectorXd& get_torques() const;

  /**
   * @brief Get the torque of a joint by its name, if it exists.
   * @copydetails JointState::get_joint_index()
   * @param joint_name The name of the joint
   * @return The torque of the joint, if it exists
   */
  double get_torque(const std::string& joint_name) const;

  /**
   * @brief Get the torque of a joint by its index, if it exists.
   * @copydetails JointState::get_joint_index()
   * @param joint_index The index of the joint
   * @return The torque of the joint, if it exists
   */
  double get_torque(unsigned int joint_index) const;

  /**
   * @brief Setter of the torques attribute.
   * @param torques The joint torques as Eigen vector
   */
  void set_torques(const Eigen::VectorXd& torques);

  /**
   * @brief Setter of the torques attributes.
   * @param torques The joint torques as std vector
   */
  void set_torques(const std::vector<double>& torques);

  /**
   * @brief Set the torque of a joint by its name.
   * @param torque The torque of the joint
   * @param joint_name The name of the joint
   */
  void set_torque(double torque, const std::string& joint_name);

  /**
   * @brief Set the torque of a joint by its index.
   * @param torque The torque of the joint
   * @param joint_index The name of the joint
   */
  void set_torque(double torque, unsigned int joint_index);

  /**
   * @brief Check if the state is compatible for operations with the state given as argument.
   * @param state The state to check compatibility with
   */
  bool is_compatible(const State& state) const;

  /**
   * @brief Initialize the State to a zero value.
   */
  void initialize();

  /**
   * @brief Set the JointState to a zero value.
   */
  void set_zero();

  /**
   * @brief Clamp inplace the magnitude of the a specific joint state variable.
   * @param max_absolute_value The maximum absolute value of the state variable
   * @param state_variable_type Name of the variable from the JointStateVariable structure to clamp
   * @param noise_ratio If provided, this value will be used to apply a dead zone relative to the maximum absolute value
   * under which the state variable will be set to 0
   */
  void clamp_state_variable(
      double max_absolute_value, const JointStateVariable& state_variable_type, double noise_ratio = 0
  );

  /**
   * @brief Clamp inplace the magnitude of the a specific joint state variable
   * for each individual joint.
   * @param max_absolute_value_array The maximum absolute value of the state variable for each joints individually
   * @param state_variable_type Name of the variable from the JointStateVariable structure to clamp
   * @param noise_ratio_array Those values will be used to apply a dead zone relative to the maximum absolute value
   * under which the state variable will be set to 0 for each individual joint
   */
  void clamp_state_variable(
      const Eigen::ArrayXd& max_absolute_value_array, const JointStateVariable& state_variable_type,
      const Eigen::ArrayXd& noise_ratio_array
  );

  /**
   * @brief Return a copy of the JointState.
   * @return The copy
   */
  JointState copy() const;

  /**
   * @brief Returns the data as the concatenation of
   * all the state variables in a single vector.
   * @return The concatenated data vector
   */
  virtual Eigen::VectorXd data() const;

  /**
   * @brief Set the data of the state from
   * all the state variables in a single Eigen vector.
   * @param The concatenated data vector
   */
  virtual void set_data(const Eigen::VectorXd& data) override;

  /**
   * @brief Set the data of the state from
   * all the state variables in a single std vector.
   * @param The concatenated data vector
   */
  virtual void set_data(const std::vector<double>& data) override;

  /**
   * @brief Returns the data vector as an Eigen Array.
   * @return The concatenated data array
   */
  Eigen::ArrayXd array() const;

  /**
   * @brief Overload the += operator.
   * @param state JointState to add
   * @return The current JointState added the JointState given in argument
   */
  JointState& operator+=(const JointState& state);

  /**
   * @brief Overload the + operator.
   * @param state JointState to add
   * @return The current JointState added the JointState given in argument
   */
  JointState operator+(const JointState& state) const;

  /**
   * @brief Overload the -= operator.
   * @param state JointState to subtract
   * @return The current JointState subtracted the JointState given in argument
   */
  JointState& operator-=(const JointState& state);

  /**
   * @brief Overload the - operator.
   * @param state JointState to subtract
   * @return The current JointState subtracted the JointState given in argument
   */
  JointState operator-(const JointState& state) const;

  /**
   * @brief Overload the *= operator with a double gain.
   * @param lambda The gain to multiply with
   * @return The JointState multiplied by lambda
   */
  JointState& operator*=(double lambda);

  /**
   * @brief Overload the * operator with a double gain.
   * @param lambda The gain to multiply with
   * @return The JointState multiplied by lambda
   */
  JointState operator*(double lambda) const;

  /**
   * @brief Overload the *= operator with an array of gains.
   * @param lambda The gain array to multiply with
   * @return The JointState multiplied by lambda
   */
  JointState& operator*=(const Eigen::ArrayXd& lambda);

  /**
   * @brief Overload the * operator with an array of gains.
   * @param lambda The gain array to multiply with
   * @return The JointState multiplied by lambda
   */
  JointState operator*(const Eigen::ArrayXd& lambda) const;

  /**
   * @brief Overload the *= operator with a matrix of gains.
   * @param lambda The matrix to multiply with
   * @return The JointState multiplied by lambda
   */
  JointState& operator*=(const Eigen::MatrixXd& lambda);

  /**
   * @brief Overload the * operator with a matrix of gains.
   * @param lambda The matrix to multiply with
   * @return The JointState multiplied by lambda
   */
  JointState operator*(const Eigen::MatrixXd& lambda) const;

  /**
   * @brief Overload the /= operator with a scalar.
   * @param lambda The scalar to divide with
   * @return The JointState divided by lambda
   */
  JointState& operator/=(double lambda);

  /**
   * @brief Overload the / operator with a scalar.
   * @param lambda The scalar to divide with
   * @return The JointState divided by lambda
   */
  JointState operator/(double lambda) const;

  /**
   * @brief Compute the distance to another state as the sum of distances between each attribute.
   * @param state The second state
   * @param state_variable_type Name of the variable from the JointStateVariable structure to apply
   * the distance on (default ALL for full distance across all dimensions)
   * @return dist The distance value as a double
   */
  double dist(const JointState& state, const JointStateVariable& state_variable_type = JointStateVariable::ALL) const;

  /**
   * @brief Overload the ostream operator for printing.
   * @param os The ostream to append the string representing the state
   * @param state The state to print
   * @return The appended ostream
   */
  friend std::ostream& operator<<(std::ostream& os, const JointState& state);

  /**
   * @brief Overload the * operator with a scalar.
   * @param lambda The scalar to multiply with
   * @return The JointState provided multiply by lambda
   */
  friend JointState operator*(double lambda, const JointState& state);

  /**
   * @brief Overload the * operator with an array of gains.
   * @param lambda The gain array to multiply with
   * @return The JointState provided multiply by lambda
   */
  friend JointState operator*(const Eigen::ArrayXd& lambda, const JointState& state);

  /**
   * @brief Overload the * operator with a matrix of gains.
   * @param lambda The matrix to multiply with
   * @return The JointState provided multiply by lambda
   */
  friend JointState operator*(const Eigen::MatrixXd& lambda, const JointState& state);

  /**
   * @brief Return the joint state as a std vector of floats.
   * @return The joint vector as a std vector
   */
  std::vector<double> to_std_vector() const;
};

inline void swap(JointState& state1, JointState& state2) {
  swap(static_cast<State&>(state1), static_cast<State&>(state2));
  std::swap(state1.names_, state2.names_);
  std::swap(state1.positions_, state2.positions_);
  std::swap(state1.velocities_, state2.velocities_);
  std::swap(state1.accelerations_, state2.accelerations_);
  std::swap(state1.torques_, state2.torques_);
}

inline JointState& JointState::operator=(const JointState& state) {
  JointState tmp(state);
  swap(*this, tmp);
  return *this;
}

inline Eigen::VectorXd JointState::get_all_state_variables() const {
  Eigen::VectorXd all_fields(this->get_size() * 4);
  all_fields << this->get_positions(), this->get_velocities(), this->get_accelerations(), this->get_torques();
  return all_fields;
}

inline void JointState::set_state_variable(Eigen::VectorXd& state_variable, const Eigen::VectorXd& new_value) {
  if (new_value.size() != this->get_size()) {
    throw IncompatibleSizeException(
        "Input vector is of incorrect size: expected " + std::to_string(this->get_size()) + ", given "
            + std::to_string(new_value.size()));
  }
  this->set_filled();
  state_variable = new_value;
}

inline void JointState::set_state_variable(Eigen::VectorXd& state_variable, const std::vector<double>& new_value) {
  this->set_state_variable(state_variable, Eigen::VectorXd::Map(new_value.data(), new_value.size()));
}

inline void JointState::set_all_state_variables(const Eigen::VectorXd& new_values) {
  if (new_values.size() != 4 * this->get_size()) {
    throw IncompatibleSizeException(
        "Input is of incorrect size: expected " + std::to_string(this->get_size()) + ", given "
            + std::to_string(new_values.size()));
  }
  this->set_positions(new_values.segment(0, this->get_size()));
  this->set_velocities(new_values.segment(this->get_size(), this->get_size()));
  this->set_accelerations(new_values.segment(2 * this->get_size(), this->get_size()));
  this->set_torques(new_values.segment(3 * this->get_size(), this->get_size()));
}

inline bool JointState::is_compatible(const State& state) const {
  bool compatible = this->State::is_compatible(state);
  compatible = compatible && (this->names_.size() == dynamic_cast<const JointState&>(state).names_.size());
  if (compatible) {
    for (unsigned int i = 0; i < this->names_.size(); ++i) {
      compatible = (compatible && this->names_[i] == dynamic_cast<const JointState&>(state).names_[i]);
    }
  }
  return compatible;
}

inline unsigned int JointState::get_size() const {
  return this->names_.size();
}

inline const std::vector<std::string>& JointState::get_names() const {
  return this->names_;
}

inline void JointState::set_names(unsigned int nb_joints) {
  if (this->get_size() != nb_joints) {
    throw exceptions::IncompatibleSizeException(
        "Input number of joints is of incorrect size, expected " + std::to_string(this->get_size()) + " got "
            + std::to_string(nb_joints));
  }
  this->names_.resize(nb_joints);
  for (unsigned int i = 0; i < nb_joints; ++i) {
    this->names_[i] = "joint" + std::to_string(i);
  }
}

inline void JointState::set_names(const std::vector<std::string>& names) {
  if (this->get_size() != names.size()) {
    throw exceptions::IncompatibleSizeException(
        "Input number of joints is of incorrect size, expected " + std::to_string(this->get_size()) + " got "
            + std::to_string(names.size()));
  }
  this->names_ = names;
}

inline Eigen::VectorXd JointState::get_state_variable(const JointStateVariable& state_variable_type) const {
  switch (state_variable_type) {
    case JointStateVariable::POSITIONS:
      return this->get_positions();

    case JointStateVariable::VELOCITIES:
      return this->get_velocities();

    case JointStateVariable::ACCELERATIONS:
      return this->get_accelerations();

    case JointStateVariable::TORQUES:
      return this->get_torques();

    case JointStateVariable::ALL:
      return this->get_all_state_variables();
  }
  // this never goes here but is compulsory to avoid a warning
  return Eigen::Vector3d::Zero();
}

inline void JointState::set_state_variable(
    const Eigen::VectorXd& new_value, const JointStateVariable& state_variable_type
) {
  switch (state_variable_type) {
    case JointStateVariable::POSITIONS:
      this->set_positions(new_value);
      break;

    case JointStateVariable::VELOCITIES:
      this->set_velocities(new_value);
      break;

    case JointStateVariable::ACCELERATIONS:
      this->set_accelerations(new_value);
      break;

    case JointStateVariable::TORQUES:
      this->set_torques(new_value);
      break;

    case JointStateVariable::ALL:
      this->set_all_state_variables(new_value);
      break;
  }
}

inline std::vector<double> JointState::to_std_vector() const {
  Eigen::VectorXd data = this->data();
  return std::vector<double>(data.data(), data.data() + data.size());
}
}// namespace state_representation
