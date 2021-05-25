/**
 * @author Baptiste Busch
 * @date 2019/09/13
 */

#pragma once

#include "state_representation/robot/JointState.hpp"

namespace state_representation {
/**
 * @class JointTorques
 * @brief Class to define torques of the joints
 */
class JointTorques : public JointState {
private:
  using JointState::clamp_state_variable;

public:
  const Eigen::VectorXd& get_positions() const = delete;
  void set_positions(const Eigen::VectorXd& positions) = delete;
  void set_positions(const std::vector<double>& positions) = delete;
  const Eigen::VectorXd& get_velocities() const = delete;
  void set_velocities(const Eigen::VectorXd& velocities) = delete;
  void set_velocities(const std::vector<double>& velocities) = delete;
  const Eigen::VectorXd& get_accelerations() const = delete;
  void set_accelerations(const Eigen::VectorXd& accelerations) = delete;
  void set_accelerations(const std::vector<double>& accelerations) = delete;

  /**
   * @brief Empty constructor
   */
  explicit JointTorques() = default;

  /**
   * @brief Constructor with name and number of joints provided
   * @brief name the name of the state
   * @brief nb_joints the number of joints for initialization
   */
  explicit JointTorques(const std::string& robot_name, unsigned int nb_joints = 0);

  /**
   * @brief Constructor with name and list of joint names provided
   * @brief name the name of the state
   * @brief joint_names list of joint names
   */
  explicit JointTorques(const std::string& robot_name, const std::vector<std::string>& joint_names);

  /**
   * @brief Constructor with name and torque values provided
   * @brief name the name of the state
   * @brief torques the vector of torques
   */
  explicit JointTorques(const std::string& robot_name, const Eigen::VectorXd& torques);

  /**
   * @brief Constructor with name, a list of joint names and torque values provided
   * @brief name the name of the state
   * @brief joint_names list of joint names
   * @brief torques the vector of torques
   */
  explicit JointTorques(const std::string& robot_name, const std::vector<std::string>& joint_names,
                        const Eigen::VectorXd& torques);

  /**
   * @brief Copy constructor
   */
  JointTorques(const JointTorques& torques);

  /**
   * @brief Copy constructor from a JointState
   */
  JointTorques(const JointState& state);

  /**
   * @brief Constructor for the zero JointTorques
   * @param robot_name the name of the associated robot
   * @param nb_joints the number of joints for initialization
   * @return JointTorques with zero torques values
   */
  static JointTorques Zero(const std::string& robot_name, unsigned int nb_joints);

  /**
   * @brief Constructor for the zero JointTorques
   * @param robot_name the name of the associated robot
   * @param joint_names list of joint names
   * @return JointTorques with zero torques values
   */
  static JointTorques Zero(const std::string& robot_name, const std::vector<std::string>& joint_names);

  /**
   * @brief Constructor for the random JointTorques
   * @param robot_name the name of the associated robot
   * @param nb_joints the number of joints for initialization
   * @return JointTorques with random torques values
   */
  static JointTorques Random(const std::string& robot_name, unsigned int nb_joints);

  /**
   * @brief Constructor for the random JointTorques
   * @param robot_name the name of the associated robot
   * @param joint_names list of joint names
   * @return JointTorques with random torques values
   */
  static JointTorques Random(const std::string& robot_name, const std::vector<std::string>& joint_names);

  /**
   * @brief Copy assignment operator that have to be defined to the custom assignment operator
   * @param torques the state with value to assign
   * @return reference to the current state with new values
   */
  JointTorques& operator=(const JointTorques& torques) = default;

  /**
   * @brief Overload the += operator
   * @param torques JointTorques to add
   * @return the current JointTorques added the JointTorques given in argument
   */
  JointTorques& operator+=(const JointTorques& torques);

  /**
   * @brief Overload the + operator
   * @param torques JointTorques to add
   * @return the current JointTorques added the JointTorques given in argument
   */
  JointTorques operator+(const JointTorques& torques) const;

  /**
   * @brief Overload the -= operator
   * @param torques JointTorques to subtract
   * @return the current JointTorques subtracted the JointTorques given in argument
   */
  JointTorques& operator-=(const JointTorques& torques);

  /**
   * @brief Overload the - operator
   * @param torques JointTorques to subtract
   * @return the current JointTorques subtracted the JointTorques given in argument
   */
  JointTorques operator-(const JointTorques& torques) const;

  /**
   * @brief Overload the *= operator with a double gain
   * @param lambda the gain to multiply with
   * @return the JointTorques multiplied by lambda
   */
  JointTorques& operator*=(double lambda);

  /**
   * @brief Overload the * operator with a double gain
   * @param lambda the gain to multiply with
   * @return the JointTorques multiplied by lambda
   */
  JointTorques operator*(double lambda) const;

  /**
   * @brief Overload the *= operator with an array of gains
   * @param lambda the gain array to multiply with
   * @return the JointTorques multiplied by lambda
   */
  JointTorques& operator*=(const Eigen::ArrayXd& lambda);

  /**
   * @brief Overload the *= operator with an array of gains
   * @param lambda the gain array to multiply with
   * @return the JointTorques multiplied by lambda
   */
  JointTorques operator*(const Eigen::ArrayXd& lambda) const;

  /**
   * @brief Overload the *= operator with a matrix of gains
   * @param lambda the matrix to multiply with
   * @return the JointTorques multiplied by lambda
   */
  JointTorques& operator*=(const Eigen::MatrixXd& lambda);

  /**
   * @brief Overload the * operator with a matrix of gains
   * @param lambda the matrix to multiply with
   * @return the JointTorques multiplied by lambda
   */
  JointTorques operator*(const Eigen::MatrixXd& lambda) const;

  /**
   * @brief Overload the /= operator with a scalar
   * @param lambda the scalar to divide with
   * @return the JointTorques divided by lambda
   */
  JointTorques& operator/=(double lambda);

  /**
   * @brief Overload the / operator with a scalar
   * @param lambda the scalar to divide with
   * @return the JointTorques divided by lambda
   */
  JointTorques operator/(double lambda) const;

  /**
   * @brief Return a copy of the JointTorques
   * @return the copy
   */
  JointTorques copy() const;

  /**
   * @brief Returns the torques data as an Eigen vector
   * @return the torque data vector
   */
  Eigen::VectorXd data() const override;

  /**
   * @brief Clamp inplace the magnitude of the torque to the values in argument
   * @param max_absolute_value the maximum magnitude of torque for all the joints
   * @param noise_ratio if provided, this value will be used to apply a dead zone under which
   * the torque will be set to 0
   */
  void clamp(double max_absolute_value, double noise_ratio = 0.);

  /**
   * @brief Return the torque clamped to the values in argument
   * @param max_absolute_value the maximum magnitude of torque for all the joints
   * @param noise_ratio if provided, this value will be used to apply a dead zone under which
   * the torque will be set to 0
   * @return the clamped JointTorques
   */
  JointTorques clamped(double max_absolute_value, double noise_ratio = 0.) const;

  /**
   * @brief Clamp inplace the magnitude of the torque to the values in argument
   * @param max_absolute_value_array the maximum magnitude of torque for each joint
   * @param noise_ratio_array if provided, this value will be used to apply a dead zone under which
   * the torque will be set to 0
   */
  void clamp(const Eigen::ArrayXd& max_absolute_value_array, const Eigen::ArrayXd& noise_ratio_array);

  /**
   * @brief Return the torque clamped to the values in argument
   * @param max_absolute_value_array the maximum magnitude of torque for each joint
   * @param noise_ratio_array if provided, this value will be used to apply a dead zone under which
   * the torque will be set to 0
   * @return the clamped JointTorques
   */
  JointTorques clamped(const Eigen::ArrayXd& max_absolute_value_array, const Eigen::ArrayXd& noise_ratio_array) const;

  /**
   * @brief Overload the ostream operator for printing
   * @param os the ostream to append the string representing the state
   * @param state the state to print
   * @return the appended ostream
   */
  friend std::ostream& operator<<(std::ostream& os, const JointTorques& torques);

  /**
   * @brief Overload the * operator with a scalar
   * @param lambda the scalar to multiply with
   * @return the JointTorques provided multiply by lambda
   */
  friend JointTorques operator*(double lambda, const JointTorques& torques);

  /**
   * @brief Overload the * operator with an array of gains
   * @param lambda the array to multiply with
   * @return the JointTorques provided multiply by lambda
   */
  friend JointTorques operator*(const Eigen::ArrayXd& lambda, const JointTorques& torques);

  /**
   * @brief Overload the * operator with a matrix of gains
   * @param lambda the matrix to multiply with
   * @return the JointTorques provided multiply by lambda
   */
  friend JointTorques operator*(const Eigen::MatrixXd& lambda, const JointTorques& torques);
};
}// namespace state_representation
