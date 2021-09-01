#pragma once

#include "state_representation/space/cartesian/CartesianState.hpp"
#include "state_representation/space/cartesian/CartesianPose.hpp"
#include "state_representation/space/cartesian/CartesianTwist.hpp"

namespace state_representation {
class CartesianPose;
class CartesianTwist;
/**
 * @class CartesianWrench
 * @brief Class to define wrench in cartesian space as 3D force and torque vectors
 */
class CartesianWrench : public CartesianState {
private:
  using CartesianState::clamp_state_variable;

public:
  // delete inaccessible getter and setters
  const Eigen::Vector3d& get_linear_velocity() const = delete;
  const Eigen::Vector3d& get_angular_velocity() const = delete;
  Eigen::Matrix<double, 6, 1> get_twist() const = delete;
  const Eigen::Vector3d& get_position() const = delete;
  const Eigen::Quaterniond& get_orientation() const = delete;
  Eigen::Vector4d get_orientation_coefficients() const = delete;
  Eigen::Matrix<double, 7, 1> get_pose() const = delete;
  Eigen::Matrix4d get_transformation_matrix() const = delete;
  const Eigen::Vector3d& get_linear_acceleration() const = delete;
  const Eigen::Vector3d& get_angular_acceleration() const = delete;
  Eigen::Matrix<double, 6, 1> get_accelerations() const = delete;
  void set_position(const Eigen::Vector3d& position) = delete;
  void set_position(const std::vector<double>& position) = delete;
  void set_position(const double& x, const double& y, const double& z) = delete;
  void set_orientation(const Eigen::Quaterniond& orientation) = delete;
  void set_orientation(const Eigen::Vector4d& orientation) = delete;
  void set_orientation(const std::vector<double>& orientation) = delete;
  void set_pose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) = delete;
  void set_pose(const Eigen::Matrix<double, 7, 1>& pose) = delete;
  void set_pose(const std::vector<double>& pose) = delete;
  void set_linear_velocity(const Eigen::Vector3d& linear_velocity) = delete;
  void set_angular_velocity(const Eigen::Vector3d& angular_velocity) = delete;
  void set_twist(const Eigen::Matrix<double, 6, 1>& twist) = delete;
  void set_linear_acceleration(const Eigen::Vector3d& linear_acceleration) = delete;
  void set_angular_acceleration(const Eigen::Vector3d& angular_acceleration) = delete;
  void set_accelerations(const Eigen::Matrix<double, 6, 1>& accelerations) = delete;

  /**
   * @brief Empty constructor
   */
  explicit CartesianWrench() = default;

  /**
    * @brief Constructor with name and reference frame provided
    * @param name the name of the state
    * @param reference the name of the reference frame
    */
  explicit CartesianWrench(const std::string& name, const std::string& reference = "world");

  /**
   * @brief Copy constructor
   */
  CartesianWrench(const CartesianWrench& wrench);

  /**
   * @brief Copy constructor from a CartesianState
   */
  CartesianWrench(const CartesianState& state);

  /**
   * @brief Construct a CartesianWrench from a force given as a vector.
   */
  explicit CartesianWrench(
      const std::string& name, const Eigen::Vector3d& force, const std::string& reference = "world"
  );

  /**
   * @brief Construct a CartesianWrench from a force and torque given as vectors.
   */
  explicit CartesianWrench(
      const std::string& name, const Eigen::Vector3d& force, const Eigen::Vector3d& torque,
      const std::string& reference = "world"
  );

  /**
   * @brief Construct a CartesianWrench from a single 6d wrench vector
   */
  explicit CartesianWrench(
      const std::string& name, const Eigen::Matrix<double, 6, 1>& wrench, const std::string& reference = "world"
  );

  /**
   * @brief Constructor for the zero wrench
   * @param name the name of the state
   * @param reference the name of the reference frame
   * @return CartesianWrench with zero values
   */
  static CartesianWrench Zero(const std::string& name, const std::string& reference = "world");

  /**
   * @brief Constructor for a random wrench
   * @param name the name of the state
   * @param reference the name of the reference frame
   * @return CartesianWrench random wrench
   */
  static CartesianWrench Random(const std::string& name, const std::string& reference = "world");

  /**
   * @brief Copy assignment operator that have to be defined to the custom assignment operator
   * @param wrench the wrench with value to assign
   * @return reference to the current wrench with new values
   */
  CartesianWrench& operator=(const CartesianWrench& wrench) = default;

  /**
   * @brief Overload the *= operator
   * @param wrench CartesianWrench to multiply with
   * @return the current CartesianWrench multiplied by the CartesianWrench given in argument
   */
  CartesianWrench& operator*=(const CartesianWrench& wrench);

  /**
   * @brief Overload the * operator with a wrench
   * @param wrench CartesianWrench to multiply with
   * @return the current CartesianWrench multiplied by the CartesianWrench given in argument
   */
  [[deprecated]] CartesianWrench operator*(const CartesianWrench& wrench) const;

  /**
   * @brief Overload the * operator
   * @param state CartesianState to multiply with
   * @return the current CartesianWrench multiplied by the CartesianState given in argument
   */
  [[deprecated]] CartesianState operator*(const CartesianState& state) const;

  /**
   * @brief Overload the * operator
   * @param state CartesianPose to multiply with
   * @return the current CartesianWrench multiplied by the CartesianPose given in argument
   */
  [[deprecated]] CartesianPose operator*(const CartesianPose& pose) const;

  /**
   * @brief Overload the * operator
   * @param state CartesianWrench to multiply with
   * @return the current CartesianWrench multiplied by the CartesianTwist given in argument
   */
  [[deprecated]] CartesianTwist operator*(const CartesianTwist& twist) const;

  /**
   * @brief Overload the += operator
   * @param wrench CartesianWrench to add
   * @return the current CartesianWrench added the CartesianWrench given in argument
   */
  CartesianWrench& operator+=(const CartesianWrench& wrench);

  /**
   * @brief Overload the + operator
   * @param wrench CartesianWrench to add
   * @return the current CartesianWrench added the CartesianWrench given in argument
   */
  CartesianWrench operator+(const CartesianWrench& wrench) const;

  /**
   * @brief Overload the -= operator
   * @param wrench CartesianWrench to subtract
   * @return the current CartesianWrench minus the CartesianWrench given in argument
   */
  CartesianWrench& operator-=(const CartesianWrench& wrench);

  /**
   * @brief Overload the - operator
   * @param wrench CartesianWrench to subtract
   * @return the current CartesianWrench minus the CartesianWrench given in argument
   */
  CartesianWrench operator-(const CartesianWrench& wrench) const;

  /**
   * @brief Overload the *= operator with a scalar
   * @param lambda the scalar to multiply with
   * @return the CartesianWrench multiplied by lambda
   */
  CartesianWrench& operator*=(double lambda);

  /**
   * @brief Overload the * operator with a scalar
   * @param lambda the scalar to multiply with
   * @return the CartesianWrench multiplied by lambda
   */
  CartesianWrench operator*(double lambda) const;

  /**
   * @brief Overload the /= operator with a scalar
   * @param lambda the scalar to divide with
   * @return the CartesianWrench divided by lambda
   */
  CartesianWrench& operator/=(double lambda);

  /**
   * @brief Overload the / operator with a scalar
   * @param lambda the scalar to divide with
   * @return the CartesianWrench divided by lambda
   */
  CartesianWrench operator/(double lambda) const;

  /**
   * @brief Clamp inplace the magnitude of the wrench to the values in argument
   * @param max_force the maximum magnitude of the force
   * @param max_torque the maximum magnitude of the torque
   * @param force_noise_ratio if provided, this value will be used to apply a deadzone under which
   * the force will be set to 0
   * @param torque_noise_ratio if provided, this value will be used to apply a deadzone under which
   * the torque will be set to 0
   */
  void clamp(double max_force, double max_torque, double force_noise_ratio = 0, double torque_noise_ratio = 0);

  /**
   * @brief Return the clamped wrench
   * @param max_force the maximum magnitude of the force
   * @param max_torque the maximum magnitude of the torque
   * @param force_noise_ratio if provided, this value will be used to apply a deadzone under which
   * the force will be set to 0
   * @param torque_noise_ratio if provided, this value will be used to apply a deadzone under which
   * the torque will be set to 0
   * @return the clamped wrench
   */
  CartesianWrench clamped(
      double max_force, double max_torque, double force_noise_ratio = 0, double torque_noise_ratio = 0
  ) const;

  /**
   * @brief Return a copy of the CartesianWrench
   * @return the copy
   */
  CartesianWrench copy() const;

  /**
   * @brief Returns the wrench data as an Eigen vector
   * @return the wrench data vector
   */
  Eigen::VectorXd data() const override;

  /**
   * @brief Set the wrench data from an Eigen vector
   * @param the wrench data vector
   */
  void set_data(const Eigen::VectorXd& data) override;

  /**
   * @brief Set the wrench data from a std vector
   * @param the wrench data vector
   */
  void set_data(const std::vector<double>& data) override;

  /**
 * @brief Compute the inverse of the current CartesianWrench
 * @return the inverse corresponding to b_S_f (assuming this is f_S_b)
 */
  CartesianWrench inverse() const;

  /**
   * @brief Compute the norms of the state variable specified by the input type (default is full wrench)
   * @param state_variable_type the type of state variable to compute the norms on
   * @return the norms of the state variables as a vector
   */
  std::vector<double>
  norms(const CartesianStateVariable& state_variable_type = CartesianStateVariable::WRENCH) const override;

  /**
   * @brief Compute the normalized wrench at the state variable given in argument (default is full wrench)
   * @param state_variable_type the type of state variable to compute the norms on
   * @return the normalized wrench
   */
  CartesianWrench normalized(const CartesianStateVariable& state_variable_type = CartesianStateVariable::WRENCH) const;

  /**
   * @brief Overload the ostream operator for printing
   * @param os the ostream to append the string representing the CartesianWrench to
   * @param CartesianWrench the CartesianWrench to print
   * @return the appended ostream
   */
  friend std::ostream& operator<<(std::ostream& os, const CartesianWrench& wrench);

  /**
   * @brief Overload the * operator with a CartesianState
   * @param state the state to multiply with
   * @return the CartesianWrench provided multiplied by the state
   */
  friend CartesianWrench operator*(const CartesianState& state, const CartesianWrench& wrench);

  /**
   * @brief Overload the * operator with a scalar
   * @param lambda the scalar to multiply with
   * @return the CartesianWrench provided multiplied by lambda
   */
  friend CartesianWrench operator*(double lambda, const CartesianWrench& wrench);
};

inline std::vector<double> CartesianWrench::norms(const CartesianStateVariable& state_variable_type) const {
  return CartesianState::norms(state_variable_type);
}

inline CartesianWrench CartesianWrench::normalized(const CartesianStateVariable& state_variable_type) const {
  return CartesianState::normalized(state_variable_type);
}
}// namespace state_representation
