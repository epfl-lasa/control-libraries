#pragma once

#include "state_representation/robot/JointState.hpp"
#include "state_representation/robot/JointPositions.hpp"

namespace state_representation {
class JointPositions;

/**
 * @class JointVelocities
 * @brief Class to define velocities of the joints
 */
class JointVelocities : public JointState {
private:
  using JointState::clamp_state_variable;

public:
  const Eigen::VectorXd& get_positions() const = delete;
  void set_positions(const Eigen::VectorXd& positions) = delete;
  void set_positions(const std::vector<double>& positions) = delete;
  const Eigen::VectorXd& get_accelerations() const = delete;
  void set_accelerations(const Eigen::VectorXd& accelerations) = delete;
  void set_accelerations(const std::vector<double>& accelerations) = delete;
  const Eigen::VectorXd& get_torques() const = delete;
  void set_torques(const Eigen::VectorXd& torques) = delete;
  void set_torques(const std::vector<double>& torques) = delete;

  /**
   * @brief Empty constructor
   */
  explicit JointVelocities() = default;

  /**
   * @brief Constructor with name and number of joints provided
   * @brief name the name of the state
   * @brief nb_joints the number of joints for initialization
   */
  explicit JointVelocities(const std::string& robot_name, unsigned int nb_joints = 0);

  /**
   * @brief Constructor with name and list of joint names provided
   * @brief name the name of the state
   * @brief joint_names list of joint names
   */
  explicit JointVelocities(const std::string& robot_name, const std::vector<std::string>& joint_names);

  /**
   * @brief Constructor with name and velocity values provided
   * @brief name the name of the state
   * @brief velocities the vector of velocities
   */
  explicit JointVelocities(const std::string& robot_name, const Eigen::VectorXd& velocities);

  /**
   * @brief Constructor with name, a list of joint names and velocity values provided
   * @brief name the name of the state
   * @brief joint_names list of joint names
   * @brief velocities the vector of velocities
   */
  explicit JointVelocities(const std::string& robot_name, const std::vector<std::string>& joint_names,
                           const Eigen::VectorXd& velocities);

  /**
   * @brief Copy constructor
   */
  JointVelocities(const JointVelocities& velocities);

  /**
   * @brief Copy constructor from a JointState
   */
  JointVelocities(const JointState& state);

  /**
   * @brief Copy constructor from a JointPositions by considering that it is equivalent to dividing the positions by 1 second
   */
  JointVelocities(const JointPositions& positions);

  /**
   * @brief Constructor for the zero JointVelocities
   * @param robot_name the name of the associated robot
   * @param nb_joints the number of joints for initialization
   * @return JointVelocities with zero velocities values
   */
  static JointVelocities Zero(const std::string& robot_name, unsigned int nb_joints);

  /**
   * @brief Constructor for the zero JointVelocities
   * @param robot_name the name of the associated robot
   * @param joint_names list of joint names
   * @return JointVelocities with zero velocities values
   */
  static JointVelocities Zero(const std::string& robot_name, const std::vector<std::string>& joint_names);

  /**
   * @brief Constructor for the random JointVelocities
   * @param robot_name the name of the associated robot
   * @param nb_joints the number of joints for initialization
   * @return JointVelocities with random velocities values
   */
  static JointVelocities Random(const std::string& robot_name, unsigned int nb_joints);

  /**
   * @brief Constructor for the random JointVelocities
   * @param robot_name the name of the associated robot
   * @param joint_names list of joint names
   * @return JointVelocities with random velocities values
   */
  static JointVelocities Random(const std::string& robot_name, const std::vector<std::string>& joint_names);

  /**
   * @brief Copy assignment operator that have to be defined to the custom assignment operator
   * @param velocities the state with value to assign
   * @return reference to the current state with new values
   */
  JointVelocities& operator=(const JointVelocities& velocities) = default;

  /**
   * @brief Overload the += operator
   * @param velocities JointVelocities to add
   * @return the current JointVelocities added the JointVelocities given in argument
   */
  JointVelocities& operator+=(const JointVelocities& velocities);

  /**
   * @brief Overload the + operator
   * @param velocities JointVelocities to add
   * @return the current JointVelocities added the JointVelocities given in argument
   */
  JointVelocities operator+(const JointVelocities& velocities) const;

  /**
   * @brief Overload the -= operator
   * @param velocities JointVelocities to subtract
   * @return the current JointVelocities subtracted the JointVelocities given in argument
   */
  JointVelocities& operator-=(const JointVelocities& velocities);

  /**
   * @brief Overload the - operator
   * @param velocities JointVelocities to subtract
   * @return the current JointVelocities subtracted the JointVelocities given in argument
   */
  JointVelocities operator-(const JointVelocities& velocities) const;

  /**
   * @brief Overload the *= operator with a double gain
   * @param lambda the gain to multiply with
   * @return the JointVelocities multiplied by lambda
   */
  JointVelocities& operator*=(double lambda);

  /**
   * @brief Overload the * operator with a double gain
   * @param lambda the gain to multiply with
   * @return the JointVelocities multiplied by lambda
   */
  JointVelocities operator*(double lambda) const;

  /**
   * @brief Overload the *= operator with an array of gains
   * @param lambda the gain array to multiply with
   * @return the JointVelocities multiplied by lambda
   */
  JointVelocities& operator*=(const Eigen::ArrayXd& lambda);

  /**
   * @brief Overload the *= operator with an array of gains
   * @param lambda the gain array to multiply with
   * @return the JointVelocities multiplied by lambda
   */
  JointVelocities operator*(const Eigen::ArrayXd& lambda) const;

  /**
   * @brief Overload the *= operator with a matrix of gains
   * @param lambda the matrix to multiply with
   * @return the JointVelocities multiplied by lambda
   */
  JointVelocities& operator*=(const Eigen::MatrixXd& lambda);

  /**
   * @brief Overload the * operator with a matrix of gains
   * @param lambda the matrix to multiply with
   * @return the JointVelocities multiplied by lambda
   */
  JointVelocities operator*(const Eigen::MatrixXd& lambda) const;

  /**
   * @brief Overload the /= operator with a scalar
   * @param lambda the scalar to divide with
   * @return the JointVelocities divided by lambda
   */
  JointVelocities& operator/=(double lambda);

  /**
   * @brief Overload the / operator with a scalar
   * @param lambda the scalar to divide with
   * @return the JointVelocities divided by lambda
   */
  JointVelocities operator/(double lambda) const;

  /**
   * @brief Overload the * operator with a time period
   * @param dt the time period to multiply with
   * @return the JointPositions corresponding to the displacement over the time period
   */
  JointPositions operator*(const std::chrono::nanoseconds& dt) const;

  /**
   * @brief Return a copy of the JointVelocities
   * @return the copy
   */
  JointVelocities copy() const;

  /**
   * @brief Returns the velocities data as an Eigen vector
   * @return the velocities data vector
   */
  Eigen::VectorXd data() const override;

  /**
   * @brief Set the velocities data from an Eigen vector
   * @param the velocities data vector
   */
  virtual void set_data(const Eigen::VectorXd& data) override;

  /**
   * @brief Set the velocities data from an std vector
   * @param the velocities data vector
   */
  virtual void set_data(const std::vector<double>& data) override;

  /**
   * @brief Clamp inplace the magnitude of the velocity to the values in argument
   * @param max_absolute_value the maximum magnitude of torque for all the joints
   * @param noise_ratio if provided, this value will be used to apply a dead zone under which
   * the torque will be set to 0
   */
  void clamp(double max_absolute_value, double noise_ratio = 0.);

  /**
   * @brief Return the velocity clamped to the values in argument
   * @param max_absolute_value the maximum magnitude of torque for all the joints
   * @param noise_ratio if provided, this value will be used to apply a dead zone under which
   * the torque will be set to 0
   * @return the clamped JointVelocities
   */
  JointVelocities clamped(double max_absolute_value, double noise_ratio = 0.) const;

  /**
   * @brief Clamp inplace the magnitude of the velocity to the values in argument
   * @param max_absolute_value_array the maximum magnitude of torque for each joint
   * @param noise_ratio_array if provided, this value will be used to apply a dead zone under which
   * the torque will be set to 0
   */
  void clamp(const Eigen::ArrayXd& max_absolute_value_array, const Eigen::ArrayXd& noise_ratio_array);

  /**
   * @brief Return the velocity clamped to the values in argument
   * @param max_absolute_value_array the maximum magnitude of torque for each joint
   * @param noise_ratio_array if provided, this value will be used to apply a dead zone under which
   * the torque will be set to 0
   * @return the clamped JointVelocities
   */
  JointVelocities clamped(const Eigen::ArrayXd& max_absolute_value_array,
                          const Eigen::ArrayXd& noise_ratio_array) const;

  /**
   * @brief Overload the ostream operator for printing
   * @param os the ostream to append the string representing the state
   * @param state the state to print
   * @return the appended ostream
   */
  friend std::ostream& operator<<(std::ostream& os, const JointVelocities& velocities);

  /**
   * @brief Overload the * operator with a scalar
   * @param lambda the scalar to multiply with
   * @return the JointVelocities provided multiply by lambda
   */
  friend JointVelocities operator*(double lambda, const JointVelocities& velocities);

  /**
   * @brief Overload the * operator with an array of gains
   * @param lambda the array to multiply with
   * @return the JointVelocities provided multiply by lambda
   */
  friend JointVelocities operator*(const Eigen::ArrayXd& lambda, const JointVelocities& velocities);

  /**
   * @brief Overload the * operator with a matrix of gains
   * @param lambda the matrix to multiply with
   * @return the JointVelocities provided multiply by lambda
   */
  friend JointVelocities operator*(const Eigen::MatrixXd& lambda, const JointVelocities& velocities);

  /**
   * @brief Overload the * operator with a time period
   * @param dt the time period to multiply with
   * @return the JointPositions corresponding to the displacement over the time period
   */
  friend JointPositions operator*(const std::chrono::nanoseconds& dt, const JointVelocities& velocities);
};
}// namespace state_representation
