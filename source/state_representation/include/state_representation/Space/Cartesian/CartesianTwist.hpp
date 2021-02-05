/**
 * @author Baptiste Busch
 * @date 2019/06/07
 */

#pragma once

#include "state_representation/Space/Cartesian/CartesianPose.hpp"
#include "state_representation/Space/Cartesian/CartesianState.hpp"

namespace StateRepresentation {
class CartesianPose;

/**
 * @class CartesianTwist
 * @brief Class to define twist in cartesian space as 3D linear and angular velocity vectors
 */
class CartesianTwist : public CartesianState {
public:
  /**
   * Empty constructor
   */
  explicit CartesianTwist();

  /**
   * @brief Empty constructor for a CartesianTwist
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
   * @brief Construct a CartesianTwist from a linear_velocity given as a vector of coordinates for the linear velocity.
   */
  explicit CartesianTwist(const std::string& name, const Eigen::Vector3d& linear_velocity, const std::string& reference = "world");

  /**
   * @brief Construct a CartesianTwist from a linear_velocity given as a vector of coordinates and a quaternion.
   */
  explicit CartesianTwist(const std::string& name, const Eigen::Vector3d& linear_velocity, const Eigen::Vector3d& angular_velocity, const std::string& reference = "world");

  /**
   * @brief Construct a CartesianTwist from a single 6d twist vector
   */
  explicit CartesianTwist(const std::string& name, const Eigen::Matrix<double, 6, 1>& twist, const std::string& reference = "world");

  /**
   * @brief Copy assignement operator that have to be defined to the custom assignement operator
   * @param twist the twist with value to assign
   * @return reference to the current twist with new values
   */
  CartesianTwist& operator=(const CartesianTwist& twist);

  /**
   * @brief Overload the = operator from a CartesianState
   * @param state CartesianState to get velocity from
   */
  CartesianTwist& operator=(const CartesianState& state);

  /**
   * @brief Set the values of the 6D twist from a 6D Eigen Vector
   * @param twist the twist as an Eigen Vector
   */
  CartesianTwist& operator=(const Eigen::Matrix<double, 6, 1>& twist);

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
  const CartesianTwist operator*(const CartesianTwist& twist) const;

  /**
   * @brief Overload the * operator with a state
   * @param state CartesianState to multiply with
   * @return the current CartesianTwist multiplied by the CartesianState given in argument
   */
  const CartesianState operator*(const CartesianState& state) const;

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
  const CartesianTwist operator*(double lambda) const;

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
  const CartesianTwist operator+(const CartesianTwist& twist) const;

  /**
   * @brief Overload the + operator with a state
   * @param state CartesianState to add
   * @return the current CartesianTwist added the CartesianState given in argument
   */
  const CartesianState operator+(const CartesianState& state) const;

  /**
   * @brief Overload the -= operator
   * @param twist CartesianTwist to substract
   * @return the current CartesianTwist minus the CartesianTwist given in argument
   */
  CartesianTwist& operator-=(const CartesianTwist& twist);

  /**
   * @brief Overload the - operator with a twist
   * @param twist CartesianTwist to substract
   * @return the current CartesianTwist minus the CartesianTwist given in argument
   */
  const CartesianTwist operator-(const CartesianTwist& twist) const;

  /**
   * @brief Overload the - operator with a state
   * @param state CartesianState to substract
   * @return the current CartesianTwist minus the CartesianState given in argument
   */
  const CartesianState operator-(const CartesianState& state) const;

  /**
   * @brief Overload the += operator with a 6D Eigen Vector
   * @param vector Eigen Vector to add
   * @return the CartesianTwist added the vector given in argument
   */
  CartesianTwist& operator+=(const Eigen::Matrix<double, 6, 1>& vector);

  /**
   * @brief Overload the + operator with a 6D Eigen Vector
   * @param vector Eigen Vector to add
   * @return the CartesianTwist added the vector given in argument
   */
  const CartesianTwist operator+(const Eigen::Matrix<double, 6, 1>& vector) const;

  /**
   * @brief Overload the -= operator with a 6D Eigen Vector
   * @param vector Eigen Vector to substract
   * @return the CartesianTwist substracted the vector given in argument
   */
  CartesianTwist& operator-=(const Eigen::Matrix<double, 6, 1>& vector);

  /**
   * @brief Overload the - operator with a 6D Eigen Vector
   * @param vector Eigen Vector to substract
   * @return the CartesianTwist substracted the vector given in argument
   */
  const CartesianTwist operator-(const Eigen::Matrix<double, 6, 1>& vector) const;

  /**
   * @brief Overload the *= operator with a matrix
   * @param lambda the matrix to multiply with
   * @return the CartesianTwist multiplied by lambda
   */
  CartesianTwist& operator*=(const Eigen::Matrix<double, 6, 6>& lambda);

  /**
   * @brief Clamp inplace the magnitude of the velocity to the values in argument
   * @param max_linear the maximum magnitude of the linear velocity
   * @param max_angular the maximum magnitude of the angular velocity
   * @param linear_noise_ratio if provided, this value will be used to apply a deadzone under which
   * the velocity will be set to 0
   * @param angular_noise_ratio if provided, this value will be used to apply a deadzone under which
   * the velocity will be set to 0
   */
  void clamp(double max_linear, double max_angular, double linear_noise_ratio = 0, double angular_noise_ratio = 0);

  /**
   * @brief Return the clamped velocity
   * @param max_linear the maximum magnitude of the linear velocity
   * @param max_angular the maximum magnitude of the angular velocity
   * @param noise_ratio if provided, this value will be used to apply a deadzone under which
   * the velocity will be set to 0
   * @param angular_noise_ratio if provided, this value will be used to apply a deadzone under which
   * the velocity will be set to 0
   * @return the clamped velocity
   */
  const CartesianTwist clamped(double max_linear, double max_angular, double noise_ratio = 0, double angular_noise_ratio = 0) const;

  /**
   * @brief Return a copy of the CartesianTwist
   * @return the copy
   */
  const CartesianTwist copy() const;

  /**
   * @brief Return the value of the 6D twist as Eigen array
   * @retrun the Eigen array representing the twist
   */
  const Eigen::Array<double, 6, 1> array() const;

  /**
   * @brief Overload the ostream operator for printing
   * @param os the ostream to happend the string representing the CartesianTwist to
   * @param CartesianTwist the CartesianTwist to print
   * @return the appended ostream
   */
  friend std::ostream& operator<<(std::ostream& os, const CartesianTwist& twist);

  /**
   * @brief Overload the + operator with a 6D Eigen Vector
   * @param vector Eigen Vector to add
   * @param twist CartesianTwist to add
   * @return the Eigen Vector plus the CartesianTwist represented as a CartesianTwist
   */
  friend const CartesianTwist operator+(const Eigen::Matrix<double, 6, 1>& vector, const CartesianTwist& twist);

  /**
   * @brief Overload the - operator with a 6D Eigen Vector
   * @param vector Eigen Vector
   * @param twist CartesianTwist to substract
   * @return the Eigen Vector minus the CartesianTwist represented as a CartesianTwist
   */
  friend const CartesianTwist operator-(const Eigen::Matrix<double, 6, 1>& vector, const CartesianTwist& twist);

  /**
   * @brief Overload the * operator with a scalar
   * @param lambda the scalar to multiply with
   * @return the CartesianTwist provided multiplied by lambda
   */
  friend const CartesianTwist operator*(double lambda, const CartesianTwist& twist);

  /**
   * @brief Overload the * operator with a matrix
   * @param lambda the matrix to multiply with
   * @return the CartesianTwist provided multiplied by lambda
   */
  friend const CartesianTwist operator*(const Eigen::Matrix<double, 6, 6>& lambda, const CartesianTwist& twist);

  /**
   * @brief Overload the * operator with a time period
   * @param dt the time period to multiply with
   * @return the CartesianPose corresponding to the displacement over the time period
   */
  friend const CartesianPose operator*(const std::chrono::nanoseconds& dt, const CartesianTwist& twist);

  /**
   * @brief Overload the * operator with a time period
   * @param dt the time period to multiply with
   * @return the CartesianPose corresponding to the displacement over the time period
   */
  friend const CartesianPose operator*(const CartesianTwist& twist, const std::chrono::nanoseconds& dt);
};

inline CartesianTwist& CartesianTwist::operator=(const CartesianTwist& twist) {
  CartesianState::operator=(twist);
  return (*this);
}
}// namespace StateRepresentation
