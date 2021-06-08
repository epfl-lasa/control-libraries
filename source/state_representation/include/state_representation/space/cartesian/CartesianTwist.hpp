#pragma once

#include "state_representation/space/cartesian/CartesianPose.hpp"
#include "state_representation/space/cartesian/CartesianState.hpp"

namespace state_representation {
class CartesianPose;

/**
 * @class CartesianTwist
 * @brief Class to define twist in cartesian space as 3D linear and angular velocity vectors
 */
class CartesianTwist : public CartesianState {
private:
  using CartesianState::clamp_state_variable;

public:
  // delete inaccessible getter and setters
  const Eigen::Vector3d& get_position() const = delete;
  const Eigen::Quaterniond& get_orientation() const = delete;
  Eigen::Vector4d get_orientation_coefficients() const = delete;
  Eigen::Matrix<double, 7, 1> get_pose() const = delete;
  Eigen::Matrix4d get_transformation_matrix() const = delete;
  const Eigen::Vector3d& get_linear_acceleration() const = delete;
  const Eigen::Vector3d& get_angular_acceleration() const = delete;
  Eigen::Matrix<double, 6, 1> get_accelerations() const = delete;
  const Eigen::Vector3d& get_force() const = delete;
  const Eigen::Vector3d& get_torque() const = delete;
  Eigen::Matrix<double, 6, 1> get_wrench() const = delete;
  void set_position(const Eigen::Vector3d& position) = delete;
  void set_position(const std::vector<double>& position) = delete;
  void set_position(const double& x, const double& y, const double& z) = delete;
  void set_orientation(const Eigen::Quaterniond& orientation) = delete;
  void set_orientation(const Eigen::Vector4d& orientation) = delete;
  void set_orientation(const std::vector<double>& orientation) = delete;
  void set_pose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) = delete;
  void set_pose(const Eigen::Matrix<double, 7, 1>& pose) = delete;
  void set_pose(const std::vector<double>& pose) = delete;
  void set_linear_acceleration(const Eigen::Vector3d& linear_acceleration) = delete;
  void set_angular_acceleration(const Eigen::Vector3d& angular_acceleration) = delete;
  void set_accelerations(const Eigen::Matrix<double, 6, 1>& accelerations) = delete;
  void set_force(const Eigen::Vector3d& force) = delete;
  void set_torque(const Eigen::Vector3d& torque) = delete;
  void set_wrench(const Eigen::Matrix<double, 6, 1>& wrench) = delete;

  /**
   * @brief Empty constructor
   */
  explicit CartesianTwist() = default;

  /**
   * @brief Constructor with name and reference frame provided
   * @param name the name of the state
   * @param reference the name of the reference frame
   */
  explicit CartesianTwist(const std::string& name, const std::string& reference = "world");

  /**
   * @brief Copy constructor
   */
  CartesianTwist(const CartesianTwist& twist);

  /**
   * @brief Copy constructor from a CartesianState
   */
  CartesianTwist(const CartesianState& state);

  /**
   * @brief Copy constructor from a CartesianPose by considering that it is equivalent to dividing the pose by 1 second
   */
  CartesianTwist(const CartesianPose& pose);

  /**
   * @brief Construct a CartesianTwist from a linear velocity given as a vector.
   */
  explicit CartesianTwist(const std::string& name,
                          const Eigen::Vector3d& linear_velocity,
                          const std::string& reference = "world");

  /**
   * @brief Construct a CartesianTwist from a linear velocity and angular velocity given as vectors.
   */
  explicit CartesianTwist(const std::string& name,
                          const Eigen::Vector3d& linear_velocity,
                          const Eigen::Vector3d& angular_velocity,
                          const std::string& reference = "world");

  /**
   * @brief Construct a CartesianTwist from a single 6d twist vector
   */
  explicit CartesianTwist(const std::string& name,
                          const Eigen::Matrix<double, 6, 1>& twist,
                          const std::string& reference = "world");

  /**
   * @brief Constructor for the zero twist
   * @param name the name of the state
   * @param reference the name of the reference frame
   * @return CartesianTwist with zero values
   */
  static CartesianTwist Zero(const std::string& name, const std::string& reference = "world");

  /**
   * @brief Constructor for a random twist
   * @param name the name of the state
   * @param reference the name of the reference frame
   * @return CartesianTwist random twist
   */
  static CartesianTwist Random(const std::string& name, const std::string& reference = "world");

  /**
   * @brief Copy assignment operator that have to be defined to the custom assignment operator
   * @param twist the twist with value to assign
   * @return reference to the current twist with new values
   */
  CartesianTwist& operator=(const CartesianTwist& twist) = default;

  /**
   * @brief Overload the *= operator
   * @param twist CartesianTwist to multiply with
   * @return the current CartesianTwist multiplied by the CartesianTwist given in argument
   */
  CartesianTwist& operator*=(const CartesianTwist& twist);

  /**
   * @brief Overload the * operator with a twist
   * @param twist CartesianTwist to multiply with
   * @return the current CartesianTwist multiplied by the CartesianTwist given in argument
   */
  CartesianTwist operator*(const CartesianTwist& twist) const;

  /**
   * @brief Overload the += operator
   * @param twist CartesianTwist to add to
   * @return the current CartesianTwist added the CartesianTwist given in argument
   */
  CartesianTwist& operator+=(const CartesianTwist& twist);

  /**
   * @brief Overload the + operator with a twist
   * @param twist CartesianTwist to add to
   * @return the current CartesianTwist added the CartesianTwist given in argument
   */
  CartesianTwist operator+(const CartesianTwist& twist) const;

  /**
   * @brief Overload the -= operator
   * @param twist CartesianTwist to subtract
   * @return the current CartesianTwist minus the CartesianTwist given in argument
   */
  CartesianTwist& operator-=(const CartesianTwist& twist);

  /**
   * @brief Overload the - operator with a twist
   * @param twist CartesianTwist to subtract
   * @return the current CartesianTwist minus the CartesianTwist given in argument
   */
  CartesianTwist operator-(const CartesianTwist& twist) const;

  /**
   * @brief Overload the *= operator with a scalar
   * @param lambda the scalar to multiply with
   * @return the CartesianTwist multiplied by lambda
   */
  CartesianTwist& operator*=(double lambda);

  /**
   * @brief Overload the * operator with a scalar
   * @param lambda the scalar to multiply with
   * @return the CartesianState multiplied by lambda
   */
  CartesianTwist operator*(double lambda) const;

  /**
   * @brief Overload the *= operator with a gain matrix
   * @param lambda the matrix to multiply with
   * @return the CartesianTwist multiplied by lambda
   */
  CartesianTwist& operator*=(const Eigen::Matrix<double, 6, 6>& lambda);

  /**
   * @brief Overload the * operator with a time period
   * @param dt the time period to multiply with
   * @return the CartesianPose corresponding to the displacement over the time period
   */
  CartesianPose operator*(const std::chrono::nanoseconds& dt) const;

  /**
   * @brief Clamp inplace the magnitude of the twist to the values in argument
   * @param max_linear the maximum magnitude of the linear velocity
   * @param max_angular the maximum magnitude of the angular velocity
   * @param linear_noise_ratio if provided, this value will be used to apply a deadzone under which
   * the linear velocity will be set to 0
   * @param angular_noise_ratio if provided, this value will be used to apply a deadzone under which
   * the angular velocity will be set to 0
   */
  void clamp(double max_linear, double max_angular, double linear_noise_ratio = 0, double angular_noise_ratio = 0);

  /**
   * @brief Return the clamped twist
   * @param max_linear the maximum magnitude of the linear velocity
   * @param max_angular the maximum magnitude of the angular velocity
   * @param noise_ratio if provided, this value will be used to apply a deadzone under which
   * the linear velocity will be set to 0
   * @param angular_noise_ratio if provided, this value will be used to apply a deadzone under which
   * the angular velocity will be set to 0
   * @return the clamped twist
   */
  CartesianTwist clamped(double max_linear,
                         double max_angular,
                         double noise_ratio = 0,
                         double angular_noise_ratio = 0) const;

  /**
   * @brief Return a copy of the CartesianTwist
   * @return the copy
   */
  CartesianTwist copy() const;

  /**
   * @brief Returns the twist data as an Eigen vector
   * @return the twist data vector
   */
  Eigen::VectorXd data() const override;

  /**
   * @brief Compute the norms of the state variable specified by the input type (default is full twist)
   * @param state_variable_type the type of state variable to compute the norms on
   * @return the norms of the state variables as a vector
   */
  std::vector<double> norms(const CartesianStateVariable& state_variable_type = CartesianStateVariable::TWIST) const override;

  /**
   * @brief Compute the normalized twist at the state variable given in argument (default is full twist)
   * @param state_variable_type the type of state variable to compute the norms on
   * @return the normalized twist
   */
  CartesianTwist normalized(const CartesianStateVariable& state_variable_type = CartesianStateVariable::TWIST) const;

  /**
   * @brief Overload the ostream operator for printing
   * @param os the ostream to append the string representing the CartesianTwist to
   * @param CartesianTwist the CartesianTwist to print
   * @return the appended ostream
   */
  friend std::ostream& operator<<(std::ostream& os, const CartesianTwist& twist);

  /**
   * @brief Overload the * operator with a scalar
   * @param lambda the scalar to multiply with
   * @return the CartesianTwist provided multiplied by lambda
   */
  friend CartesianTwist operator*(double lambda, const CartesianTwist& twist);

  /**
   * @brief Overload the * operator with a gain matrix
   * @param lambda the matrix to multiply with
   * @return the CartesianTwist provided multiplied by lambda
   */
  friend CartesianTwist operator*(const Eigen::Matrix<double, 6, 6>& lambda, const CartesianTwist& twist);

  /**
   * @brief Overload the * operator with a time period
   * @param dt the time period to multiply with
   * @return the CartesianPose corresponding to the displacement over the time period
   */
  friend CartesianPose operator*(const std::chrono::nanoseconds& dt, const CartesianTwist& twist);
};

inline std::vector<double> CartesianTwist::norms(const CartesianStateVariable& state_variable_type) const {
  return CartesianState::norms(state_variable_type);
}

inline CartesianTwist CartesianTwist::normalized(const CartesianStateVariable& state_variable_type) const {
  return CartesianState::normalized(state_variable_type);
}
}// namespace state_representation
