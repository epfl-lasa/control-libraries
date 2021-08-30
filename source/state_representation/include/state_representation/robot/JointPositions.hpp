#pragma once

#include "state_representation/robot/JointState.hpp"
#include "state_representation/robot/JointVelocities.hpp"

namespace state_representation {
class JointVelocities;

/**
 * @class JointPositions
 * @brief Class to define a positions of the joints
 */
class JointPositions : public JointState {
private:
  using JointState::clamp_state_variable;

public:
  const Eigen::VectorXd& get_velocities() const = delete;
  void set_velocities(const Eigen::VectorXd& velocities) = delete;
  void set_velocities(const std::vector<double>& velocities) = delete;
  const Eigen::VectorXd& get_accelerations() const = delete;
  void set_accelerations(const Eigen::VectorXd& accelerations) = delete;
  void set_accelerations(const std::vector<double>& accelerations) = delete;
  const Eigen::VectorXd& get_torques() const = delete;
  void set_torques(const Eigen::VectorXd& torques) = delete;
  void set_torques(const std::vector<double>& torques) = delete;

  /**
   * @brief Empty constructor
   */
  explicit JointPositions() = default;

  /**
   * @brief Constructor with name and number of joints provided
   * @brief name the name of the state
   * @brief nb_joints the number of joints for initialization
   */
  explicit JointPositions(const std::string& robot_name, unsigned int nb_joints = 0);

  /**
   * @brief Constructor with name and list of joint names provided
   * @brief name the name of the state
   * @brief joint_names list of joint names
   */
  explicit JointPositions(const std::string& robot_name, const std::vector<std::string>& joint_names);

  /**
   * @brief Constructor with name and position values provided
   * @brief name the name of the state
   * @brief positions the vector of positions
   */
  explicit JointPositions(const std::string& robot_name, const Eigen::VectorXd& positions);

  /**
   * @brief Constructor with name, a list of joint names and position values provided
   * @brief name the name of the state
   * @brief joint_names list of joint names
   * @brief positions the vector of positions
   */
  explicit JointPositions(
      const std::string& robot_name, const std::vector<std::string>& joint_names, const Eigen::VectorXd& positions
  );

  /**
   * @brief Copy constructor
   */
  JointPositions(const JointPositions& positions);

  /**
   * @brief Copy constructor from a JointState
   */
  JointPositions(const JointState& state);

  /**
   * @brief Integration constructor from a JointVelocities by considering that it is equivalent to multiplying the
   * velocities by 1 second
   */
  JointPositions(const JointVelocities& velocities);

  /**
   * @brief Constructor for the zero JointPositions
   * @param robot_name the name of the associated robot
   * @param nb_joints the number of joints for initialization
   * @return JointPositions with zero positions values
   */
  static JointPositions Zero(const std::string& robot_name, unsigned int nb_joints);

  /**
   * @brief Constructor for the zero JointPositions
   * @param robot_name the name of the associated robot
   * @param joint_names list of joint names
   * @return JointPositions with zero positions values
   */
  static JointPositions Zero(const std::string& robot_name, const std::vector<std::string>& joint_names);

  /**
   * @brief Constructor for the random JointPositions
   * @param robot_name the name of the associated robot
   * @param nb_joints the number of joints for initialization
   * @return JointPositions with random positions values
   */
  static JointPositions Random(const std::string& robot_name, unsigned int nb_joints);

  /**
   * @brief Constructor for the random JointPositions
   * @param robot_name the name of the associated robot
   * @param joint_names list of joint names
   * @return JointPositions with random positions values
   */
  static JointPositions Random(const std::string& robot_name, const std::vector<std::string>& joint_names);

  /**
   * @brief Copy assignment operator that have to be defined to the custom assignment operator
   * @param positions the state with value to assign
   * @return reference to the current state with new values
   */
  JointPositions& operator=(const JointPositions& positions) = default;

  /**
   * @brief Overload the += operator
   * @param positions JointPositions to add
   * @return the current JointPositions added the JointPositions given in argument
   */
  JointPositions& operator+=(const JointPositions& positions);

  /**
   * @brief Overload the + operator
   * @param positions JointPositions to add
   * @return the current JointPositions added the JointPositions given in argument
   */
  JointPositions operator+(const JointPositions& positions) const;

  /**
   * @brief Overload the -= operator
   * @param positions JointPositions to subtract
   * @return the current JointPositions subtracted the JointPositions given in argument
   */
  JointPositions& operator-=(const JointPositions& positions);

  /**
   * @brief Overload the - operator
   * @param positions JointPositions to subtract
   * @return the current JointPositions subtracted the JointPositions given in argument
   */
  JointPositions operator-(const JointPositions& positions) const;

  /**
   * @brief Overload the *= operator with a double gain
   * @param lambda the gain to multiply with
   * @return the JointPositions multiplied by lambda
   */
  JointPositions& operator*=(double lambda);

  /**
   * @brief Overload the * operator with a double gain
   * @param lambda the gain to multiply with
   * @return the JointPositions multiplied by lambda
   */
  JointPositions operator*(double lambda) const;

  /**
   * @brief Overload the *= operator with an array of gains
   * @param lambda the gain array to multiply with
   * @return the JointPositions multiplied by lambda
   */
  JointPositions& operator*=(const Eigen::ArrayXd& lambda);

  /**
   * @brief Overload the *= operator with an array of gains
   * @param lambda the gain array to multiply with
   * @return the JointPositions multiplied by lambda
   */
  JointPositions operator*(const Eigen::ArrayXd& lambda) const;

  /**
   * @brief Overload the *= operator with a matrix of gains
   * @param lambda the matrix to multiply with
   * @return the JointPositions multiplied by lambda
   */
  JointPositions& operator*=(const Eigen::MatrixXd& lambda);

  /**
   * @brief Overload the * operator with a matrix of gains
   * @param lambda the matrix to multiply with
   * @return the JointPositions multiplied by lambda
   */
  JointPositions operator*(const Eigen::MatrixXd& lambda) const;

  /**
   * @brief Overload the /= operator with a scalar
   * @param lambda the scalar to divide with
   * @return the JointPositions divided by lambda
   */
  JointPositions& operator/=(double lambda);

  /**
   * @brief Overload the / operator with a scalar
   * @param lambda the scalar to divide with
   * @return the JointPositions divided by lambda
   */
  JointPositions operator/(double lambda) const;

  /**
   * @brief Overload the / operator with a time period
   * @param dt the time period to multiply with
   * @return the JointVelocities corresponding to the velocities over the time period
   */
  JointVelocities operator/(const std::chrono::nanoseconds& dt) const;

  /**
   * @brief Return a copy of the JointPositions
   * @return the copy
   */
  JointPositions copy() const;

  /**
   * @brief Returns the positions data as an Eigen vector
   * @return the positions data vector
   */
  Eigen::VectorXd data() const override;

  /**
   * @brief Set the positions data from an Eigen vector
   * @param the positions data vector
   */
  virtual void set_data(const Eigen::VectorXd& data) override;

  /**
   * @brief Set the positions data from a std vector
   * @param the positions data vector
   */
  virtual void set_data(const std::vector<double>& data) override;

  /**
   * @brief Clamp inplace the magnitude of the positions to the value in argument
   * @param max_absolute_value the maximum value of position for all the joints
   * @param noise_ratio if provided, this value will be used to apply a dead zone relative to the maximum absolute value
   * under which the position will be set to 0
   */
  void clamp(double max_absolute_value, double noise_ratio = 0.);

  /**
   * @brief Return the position clamped to the value in argument
   * @param max_absolute_value the maximum value of position for all the joints
   * @param noise_ratio if provided, this value will be used to apply a dead zone relative to the maximum absolute value
   * under which the position will be set to 0
   * @return the clamped JointPositions
   */
  JointPositions clamped(double max_absolute_value, double noise_ratio = 0.) const;

  /**
   * @brief Clamp inplace the magnitude of the positions to the values in argument
   * @param max_absolute_value_array the maximum value of position for each joint
   * @param noise_ratio_array those values will be used to apply a dead zone relative to the maximum absolute value
   * under which the position will be set to 0 for each individual joint
   */
  void clamp(const Eigen::ArrayXd& max_absolute_value_array, const Eigen::ArrayXd& noise_ratio_array);

  /**
   * @brief Return the position clamped to the values in argument
   * @param max_absolute_value_array the maximum value of position for each joint
   * @param noise_ratio_array those values will be used to apply a dead zone relative to the maximum absolute value
   * under which the position will be set to 0 for each individual joint
   * @return the clamped JointPositions
   */
  JointPositions clamped(const Eigen::ArrayXd& max_absolute_value_array, const Eigen::ArrayXd& noise_ratio_array) const;

  /**
   * @brief Overload the ostream operator for printing
   * @param os the ostream to append the string representing the state
   * @param positions the state to print
   * @return the appended ostream
   */
  friend std::ostream& operator<<(std::ostream& os, const JointPositions& positions);

  /**
   * @brief Overload the * operator with a scalar
   * @param lambda the scalar to multiply with
   * @return the JointPositions provided multiply by lambda
   */
  friend JointPositions operator*(double lambda, const JointPositions& positions);

  /**
   * @brief Overload the * operator with an array of gains
   * @param lambda the array to multiply with
   * @return the JointPositions provided multiply by lambda
   */
  friend JointPositions operator*(const Eigen::ArrayXd& lambda, const JointPositions& positions);

  /**
   * @brief Overload the * operator with a matrix of gains
   * @param lambda the matrix to multiply with
   * @return the JointPositions provided multiply by lambda
   */
  friend JointPositions operator*(const Eigen::MatrixXd& lambda, const JointPositions& positions);

  /**
   * @brief Set the value from a std vector
   * @param value the value as a std vector
   */
  [[deprecated]] void from_std_vector(const std::vector<double>& value);
};
}// namespace state_representation
