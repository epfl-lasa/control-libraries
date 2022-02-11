#pragma once

#include "state_representation/space/cartesian/CartesianState.hpp"
#include "state_representation/space/cartesian/CartesianTwist.hpp"

namespace state_representation {
class CartesianTwist;

/**
 * @class CartesianAcceleration
 * @brief Class to define acceleration in cartesian space as 3D linear and angular acceleration vectors
 */
class CartesianAcceleration : public CartesianState {
private:
  using CartesianState::clamp_state_variable;

public:
  // delete inaccessible getter and setters
  const Eigen::Vector3d& get_position() const = delete;
  const Eigen::Quaterniond& get_orientation() const = delete;
  Eigen::Vector4d get_orientation_coefficients() const = delete;
  Eigen::Matrix<double, 7, 1> get_pose() const = delete;
  Eigen::Matrix4d get_transformation_matrix() const = delete;
  const Eigen::Vector3d& get_linear_velocity() const = delete;
  const Eigen::Vector3d& get_angular_velocity() const = delete;
  Eigen::Matrix<double, 6, 1> get_twist() const = delete;
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
  void set_linear_velocity(const Eigen::Vector3d& linear_acceleration) = delete;
  void set_angular_velocity(const Eigen::Vector3d& angular_acceleration) = delete;
  void set_twist(const Eigen::Matrix<double, 6, 1>& acceleration) = delete;
  void set_force(const Eigen::Vector3d& force) = delete;
  void set_torque(const Eigen::Vector3d& torque) = delete;
  void set_wrench(const Eigen::Matrix<double, 6, 1>& wrench) = delete;
  CartesianState operator*=(const CartesianState& state) = delete;
  CartesianState operator*(const CartesianState& state) = delete;
  friend CartesianState operator*=(const CartesianState& state, const CartesianAcceleration& acceleration) = delete;

  /**
   * @brief Empty constructor
   */
  explicit CartesianAcceleration() = default;

  /**
   * @brief Constructor with name and reference frame provided
   * @param name the name of the state
   * @param reference the name of the reference frame
   */
  explicit CartesianAcceleration(const std::string& name, const std::string& reference = "world");

  /**
   * @brief Copy constructor
   */
  CartesianAcceleration(const CartesianAcceleration& acceleration);

  /**
   * @brief Copy constructor from a CartesianState
   */
  CartesianAcceleration(const CartesianState& state);

  /**
   * @brief Copy constructor from a CartesianTwist by considering that it is equivalent to dividing the twist by 1 second
   */
  CartesianAcceleration(const CartesianTwist& pose);

  /**
   * @brief Construct a CartesianAcceleration from a linear acceleration given as a vector.
   */
  explicit CartesianAcceleration(
      const std::string& name, const Eigen::Vector3d& linear_acceleration, const std::string& reference = "world"
  );

  /**
   * @brief Construct a CartesianAcceleration from a linear acceleration and angular acceleration given as vectors.
   */
  explicit CartesianAcceleration(
      const std::string& name, const Eigen::Vector3d& linear_acceleration, const Eigen::Vector3d& angular_acceleration,
      const std::string& reference = "world"
  );

  /**
   * @brief Construct a CartesianAcceleration from a single 6d acceleration vector
   */
  explicit CartesianAcceleration(
      const std::string& name, const Eigen::Matrix<double, 6, 1>& acceleration, const std::string& reference = "world"
  );

  /**
   * @brief Constructor for the zero acceleration
   * @param name the name of the state
   * @param reference the name of the reference frame
   * @return CartesianAcceleration with zero values
   */
  static CartesianAcceleration Zero(const std::string& name, const std::string& reference = "world");

  /**
   * @brief Constructor for a random acceleration
   * @param name the name of the state
   * @param reference the name of the reference frame
   * @return CartesianAcceleration random acceleration
   */
  static CartesianAcceleration Random(const std::string& name, const std::string& reference = "world");

  /**
   * @brief Copy assignment operator that have to be defined to the custom assignment operator
   * @param acceleration the acceleration with value to assign
   * @return reference to the current acceleration with new values
   */
  CartesianAcceleration& operator=(const CartesianAcceleration& acceleration) = default;

  /**
   * @brief Overload the += operator
   * @param acceleration CartesianAcceleration to add to
   * @return the current CartesianAcceleration added the CartesianAcceleration given in argument
   */
  CartesianAcceleration& operator+=(const CartesianAcceleration& acceleration);

  /**
   * @brief Overload the + operator with an acceleration
   * @param acceleration CartesianAcceleration to add to
   * @return the current CartesianAcceleration added the CartesianAcceleration given in argument
   */
  CartesianAcceleration operator+(const CartesianAcceleration& acceleration) const;

  /**
   * @brief Overload the -= operator
   * @param acceleration CartesianAcceleration to subtract
   * @return the current CartesianAcceleration minus the CartesianAcceleration given in argument
   */
  CartesianAcceleration& operator-=(const CartesianAcceleration& acceleration);

  /**
   * @brief Overload the - operator with an acceleration
   * @param acceleration CartesianAcceleration to subtract
   * @return the current CartesianAcceleration minus the CartesianAcceleration given in argument
   */
  CartesianAcceleration operator-(const CartesianAcceleration& acceleration) const;

  /**
   * @brief Overload the *= operator with a scalar
   * @param lambda the scalar to multiply with
   * @return the CartesianAcceleration multiplied by lambda
   */
  CartesianAcceleration& operator*=(double lambda);

  /**
   * @brief Overload the * operator with a scalar
   * @param lambda the scalar to multiply with
   * @return the CartesianAcceleration multiplied by lambda
   */
  CartesianAcceleration operator*(double lambda) const;

  /**
   * @brief Overload the /= operator with a scalar
   * @param lambda the scalar to divide with
   * @return the CartesianAcceleration divided by lambda
   */
  CartesianAcceleration& operator/=(double lambda);

  /**
   * @brief Overload the / operator with a scalar
   * @param lambda the scalar to divide with
   * @return the CartesianAcceleration divided by lambda
   */
  CartesianAcceleration operator/(double lambda) const;

  /**
   * @brief Overload the *= operator with a gain matrix
   * @param lambda the matrix to multiply with
   * @return the CartesianAcceleration multiplied by lambda
   */
  CartesianAcceleration& operator*=(const Eigen::Matrix<double, 6, 6>& lambda);

  /**
   * @brief Overload the * operator with a time period
   * @param dt the time period to multiply with
   * @return the CartesianTwist corresponding to the twist over the time period
   */
  CartesianTwist operator*(const std::chrono::nanoseconds& dt) const;

  /**
   * @brief Clamp inplace the magnitude of the acceleration to the values in argument
   * @param max_linear the maximum magnitude of the linear acceleration
   * @param max_angular the maximum magnitude of the angular acceleration
   * @param linear_noise_ratio if provided, this value will be used to apply a deadzone under which
   * the linear acceleration will be set to 0
   * @param angular_noise_ratio if provided, this value will be used to apply a deadzone under which
   * the angular acceleration will be set to 0
   */
  void clamp(double max_linear, double max_angular, double linear_noise_ratio = 0, double angular_noise_ratio = 0);

  /**
   * @brief Return the clamped twist
   * @param max_linear the maximum magnitude of the linear acceleration
   * @param max_angular the maximum magnitude of the angular acceleration
   * @param noise_ratio if provided, this value will be used to apply a deadzone under which
   * the linear acceleration will be set to 0
   * @param angular_noise_ratio if provided, this value will be used to apply a deadzone under which
   * the angular acceleration will be set to 0
   * @return the clamped acceleration
   */
  CartesianAcceleration clamped(
      double max_linear, double max_angular, double noise_ratio = 0, double angular_noise_ratio = 0
  ) const;

  /**
   * @brief Return a copy of the CartesianAcceleration
   * @return the copy
   */
  CartesianAcceleration copy() const;

  /**
   * @brief Returns the acceleration data as an Eigen vector
   * @return the acceleration data vector
   */
  Eigen::VectorXd data() const override;

  /**
   * @brief Set the acceleration data from an Eigen vector
   * @param the acceleration data vector
   */
  void set_data(const Eigen::VectorXd& data) override;

  /**
   * @brief Set the acceleration data from a std vector
   * @param the acceleration data vector
   */
  void set_data(const std::vector<double>& data) override;

  /**
   * @brief Compute the inverse of the current CartesianAcceleration
   * @return the inverse corresponding to b_S_f (assuming this is f_S_b)
   */
  CartesianAcceleration inverse() const;

  /**
   * @brief Compute the norms of the state variable specified by the input type (default is full twist)
   * @param state_variable_type the type of state variable to compute the norms on
   * @return the norms of the state variables as a vector
   */
  std::vector<double>
  norms(const CartesianStateVariable& state_variable_type = CartesianStateVariable::ACCELERATION) const override;

  /**
   * @brief Compute the normalized acceleration at the state variable given in argument (default is full acceleration)
   * @param state_variable_type the type of state variable to compute the norms on
   * @return the normalized acceleration
   */
  CartesianAcceleration
  normalized(const CartesianStateVariable& state_variable_type = CartesianStateVariable::ACCELERATION) const;

  /**
   * @brief Overload the ostream operator for printing
   * @param os the ostream to append the string representing the CartesianAcceleration to
   * @param CartesianAcceleration the CartesianAcceleration to print
   * @return the appended ostream
   */
  friend std::ostream& operator<<(std::ostream& os, const CartesianAcceleration& acceleration);

  /**
   * @brief Overload the * operator with a CartesianState
   * @param state the state to multiply with
   * @return the CartesianAcceleration provided multiplied by the state
   */
  friend CartesianAcceleration operator*(const CartesianState& state, const CartesianAcceleration& acceleration);

  /**
   * @brief Overload the * operator with a scalar
   * @param lambda the scalar to multiply with
   * @return the CartesianAcceleration provided multiplied by lambda
   */
  friend CartesianAcceleration operator*(double lambda, const CartesianAcceleration& acceleration);

  /**
   * @brief Overload the * operator with a gain matrix
   * @param lambda the matrix to multiply with
   * @return the CartesianAcceleration provided multiplied by lambda
   */
  friend CartesianAcceleration
  operator*(const Eigen::Matrix<double, 6, 6>& lambda, const CartesianAcceleration& acceleration);

  /**
   * @brief Overload the * operator with a time period
   * @param dt the time period to multiply with
   * @return the CartesianTwist corresponding to the velocity over the time period
   */
  friend CartesianTwist operator*(const std::chrono::nanoseconds& dt, const CartesianAcceleration& acceleration);
};

inline std::vector<double> CartesianAcceleration::norms(const CartesianStateVariable& state_variable_type) const {
  return CartesianState::norms(state_variable_type);
}

inline CartesianAcceleration CartesianAcceleration::normalized(const CartesianStateVariable& state_variable_type) const {
  return CartesianState::normalized(state_variable_type);
}
}// namespace state_representation
