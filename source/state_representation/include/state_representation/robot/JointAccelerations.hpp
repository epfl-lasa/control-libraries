#pragma once

#include "state_representation/robot/JointState.hpp"
#include "state_representation/robot/JointVelocities.hpp"

namespace state_representation {
class JointVelocities;

/**
 * @class JointAccelerations
 * @brief Class to define accelerations of the joints
 */
class JointAccelerations : public JointState {
private:
  using JointState::clamp_state_variable;

public:
  const Eigen::VectorXd& get_positions() const = delete;
  void set_positions(const Eigen::VectorXd& positions) = delete;
  void set_positions(const std::vector<double>& positions) = delete;
  const Eigen::VectorXd& get_velocities() const = delete;
  void set_velocities(const Eigen::VectorXd& accelerations) = delete;
  void set_velocities(const std::vector<double>& accelerations) = delete;
  const Eigen::VectorXd& get_torques() const = delete;
  void set_torques(const Eigen::VectorXd& torques) = delete;
  void set_torques(const std::vector<double>& torques) = delete;

  /**
   * @brief Empty constructor
   */
  explicit JointAccelerations() = default;

  /**
   * @brief Constructor with name and number of joints provided
   * @brief name the name of the state
   * @brief nb_joints the number of joints for initialization
   */
  explicit JointAccelerations(const std::string& robot_name, unsigned int nb_joints = 0);

  /**
   * @brief Constructor with name and list of joint names provided
   * @brief name the name of the state
   * @brief joint_names list of joint names
   */
  explicit JointAccelerations(const std::string& robot_name, const std::vector<std::string>& joint_names);

  /**
   * @brief Constructor with name and acceleration values provided
   * @brief name the name of the state
   * @brief accelerations the vector of accelerations
   */
  explicit JointAccelerations(const std::string& robot_name, const Eigen::VectorXd& accelerations);

  /**
   * @brief Constructor with name, a list of joint names and accelerations values provided
   * @brief name the name of the state
   * @brief joint_names list of joint names
   * @brief accelerations the vector of accelerations
   */
  explicit JointAccelerations(const std::string& robot_name,
                              const std::vector<std::string>& joint_names,
                              const Eigen::VectorXd& accelerations);

  /**
   * @brief Copy constructor
   */
  JointAccelerations(const JointAccelerations& accelerations);

  /**
   * @brief Copy constructor from a JointState
   */
  JointAccelerations(const JointState& state);

  /**
   * @brief Copy constructor from a JointVelocities by considering that it is equivalent to dividing the velocities by 1 second
   */
  JointAccelerations(const JointVelocities& velocities);

/**
 * @brief Constructor for the zero JointAccelerations
 * @param robot_name the name of the associated robot
 * @param nb_joints the number of joints for initialization
 * @return JointAccelerations with zero accelerations values
 */
  static JointAccelerations Zero(const std::string& robot_name, unsigned int nb_joints);

  /**
   * @brief Constructor for the zero JointAccelerations
   * @param robot_name the name of the associated robot
   * @param joint_names list of joint names
   * @return JointAccelerations with zero accelerations values
   */
  static JointAccelerations Zero(const std::string& robot_name, const std::vector<std::string>& joint_names);

  /**
   * @brief Constructor for the random JointAccelerations
   * @param robot_name the name of the associated robot
   * @param nb_joints the number of joints for initialization
   * @return JointAccelerations with random accelerations values
   */
  static JointAccelerations Random(const std::string& robot_name, unsigned int nb_joints);

  /**
   * @brief Constructor for the random JointAccelerations
   * @param robot_name the name of the associated robot
   * @param joint_names list of joint names
   * @return JointAccelerations with random accelerations values
   */
  static JointAccelerations Random(const std::string& robot_name, const std::vector<std::string>& joint_names);

  /**
   * @brief Copy assignment operator that have to be defined to the custom assignment operator
   * @param accelerations the state with value to assign
   * @return reference to the current state with new values
   */
  JointAccelerations& operator=(const JointAccelerations& accelerations) = default;

  /**
   * @brief Overload the += operator
   * @param accelerations JointAccelerations to add
   * @return the current JointAccelerations added the JointAccelerations given in argument
   */
  JointAccelerations& operator+=(const JointAccelerations& accelerations);

  /**
   * @brief Overload the + operator
   * @param accelerations JointAccelerations to add
   * @return the current JointAccelerations added the JointAccelerations given in argument
   */
  JointAccelerations operator+(const JointAccelerations& accelerations) const;

  /**
   * @brief Overload the -= operator
   * @param accelerations JointAccelerations to subtract
   * @return the current JointAccelerations subtracted the JointAccelerations given in argument
   */
  JointAccelerations& operator-=(const JointAccelerations& accelerations);

  /**
   * @brief Overload the - operator
   * @param accelerations JointAccelerations to subtract
   * @return the current JointAccelerations subtracted the JointAccelerations given in argument
   */
  JointAccelerations operator-(const JointAccelerations& accelerations) const;

  /**
   * @brief Overload the *= operator with a double gain
   * @param lambda the gain to multiply with
   * @return the JointAccelerations multiplied by lambda
   */
  JointAccelerations& operator*=(double lambda);

  /**
   * @brief Overload the * operator with a double gain
   * @param lambda the gain to multiply with
   * @return the JointAccelerations multiplied by lambda
   */
  JointAccelerations operator*(double lambda) const;

  /**
   * @brief Overload the *= operator with an array of gains
   * @param lambda the gain array to multiply with
   * @return the JointAccelerations multiplied by lambda
   */
  JointAccelerations& operator*=(const Eigen::ArrayXd& lambda);

  /**
   * @brief Overload the *= operator with an array of gains
   * @param lambda the gain array to multiply with
   * @return the JointAccelerations multiplied by lambda
   */
  JointAccelerations operator*(const Eigen::ArrayXd& lambda) const;

  /**
   * @brief Overload the *= operator with a matrix of gains
   * @param lambda the matrix to multiply with
   * @return the JointAccelerations multiplied by lambda
   */
  JointAccelerations& operator*=(const Eigen::MatrixXd& lambda);

  /**
   * @brief Overload the * operator with a matrix of gains
   * @param lambda the matrix to multiply with
   * @return the JointAccelerations multiplied by lambda
   */
  JointAccelerations operator*(const Eigen::MatrixXd& lambda) const;

  /**
   * @brief Overload the /= operator with a scalar
   * @param lambda the scalar to divide with
   * @return the JointAccelerations divided by lambda
   */
  JointAccelerations& operator/=(double lambda);

  /**
   * @brief Overload the / operator with a scalar
   * @param lambda the scalar to divide with
   * @return the JointAccelerations divided by lambda
   */
  JointAccelerations operator/(double lambda) const;

  /**
   * @brief Overload the * operator with a time period
   * @param dt the time period to multiply with
   * @return the JointVelocities corresponding to the velocities over the time period
   */
  JointVelocities operator*(const std::chrono::nanoseconds& dt) const;

  /**
   * @brief Return a copy of the JointAccelerations
   * @return the copy
   */
  JointAccelerations copy() const;

  /**
   * @brief Returns the accelerations data as an Eigen vector
   * @return the accelerations data vector
   */
  Eigen::VectorXd data() const override;

  /**
   * @brief Set the accelerations data from an Eigen vector
   * @param the accelerations data vector
   */
  virtual void set_data(const Eigen::VectorXd& data) override;

  /**
   * @brief Set the accelerations data from a std vector
   * @param the accelerations data vector
   */
  virtual void set_data(const std::vector<double>& data) override;

  /**
   * @brief Clamp inplace the magnitude of the acceleration to the values in argument
   * @param max_absolute_value the maximum magnitude of acceleration for all the joints
   * @param noise_ratio if provided, this value will be used to apply a dead zone under which
   * the acceleration will be set to 0
   */
  void clamp(double max_absolute_value, double noise_ratio = 0.);

  /**
   * @brief Return the acceleration clamped to the values in argument
   * @param max_absolute_value the maximum magnitude of acceleration for all the joints
   * @param noise_ratio if provided, this value will be used to apply a dead zone under which
   * the acceleration will be set to 0
   * @return the clamped JointAccelerations
   */
  JointAccelerations clamped(double max_absolute_value, double noise_ratio = 0.) const;

  /**
   * @brief Clamp inplace the magnitude of the acceleration to the values in argument
   * @param max_absolute_value_array the maximum magnitude of acceleration for each joint
   * @param noise_ratio_array if provided, this value will be used to apply a dead zone under which
   * the acceleration will be set to 0
   */
  void clamp(const Eigen::ArrayXd& max_absolute_value_array, const Eigen::ArrayXd& noise_ratio_array);

  /**
   * @brief Return the acceleration clamped to the values in argument
   * @param max_absolute_value_array the maximum magnitude of acceleration for each joint
   * @param noise_ratio_array if provided, this value will be used to apply a dead zone under which
   * the acceleration will be set to 0
   * @return the clamped JointAccelerations
   */
  JointAccelerations clamped(const Eigen::ArrayXd& max_absolute_value_array,
                             const Eigen::ArrayXd& noise_ratio_array) const;

  /**
   * @brief Overload the ostream operator for printing
   * @param os the ostream to append the string representing the state
   * @param state the state to print
   * @return the appended ostream
   */
  friend std::ostream& operator<<(std::ostream& os, const JointAccelerations& accelerations);

  /**
   * @brief Overload the * operator with a scalar
   * @param lambda the scalar to multiply with
   * @return the JointAccelerations provided multiply by lambda
   */
  friend JointAccelerations operator*(double lambda, const JointAccelerations& accelerations);

  /**
   * @brief Overload the * operator with an array of gains
   * @param lambda the array to multiply with
   * @return the JointAccelerations provided multiply by lambda
   */
  friend JointAccelerations operator*(const Eigen::ArrayXd& lambda, const JointAccelerations& accelerations);

  /**
   * @brief Overload the * operator with a matrix of gains
   * @param lambda the matrix to multiply with
   * @return the JointAccelerations provided multiply by lambda
   */
  friend JointAccelerations operator*(const Eigen::MatrixXd& lambda, const JointAccelerations& accelerations);

  /**
   * @param dt the time period to multiply with
   * @return the JointVelocities corresponding to the velocities over the time period
   */
  friend JointVelocities operator*(const std::chrono::nanoseconds& dt, const JointAccelerations& accelerations);
};
}// namespace state_representation
