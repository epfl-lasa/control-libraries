/**
 * @author Baptiste Busch
 * @date 2019/06/07
 */

#pragma once

#include "state_representation/space/cartesian/CartesianState.hpp"
#include "state_representation/space/cartesian/CartesianTwist.hpp"

namespace state_representation {
class CartesianTwist;

/**
 * @class CartesianPose
 * @brief Class to define CartesianPose in cartesian space as 3D position and quaternion based orientation
 */
class CartesianPose : public CartesianState {
public:
  /**
   * Empty constructor
   */
  explicit CartesianPose();

  /**
   * @brief Constructor with name and reference frame provided
   * @param name the name of the state
   * @param reference the name of the reference frame
   */
  explicit CartesianPose(const std::string& name, const std::string& reference = "world");

  /**
   * @brief Copy constructor
   */
  CartesianPose(const CartesianPose& pose);

  /**
   * @brief Copy constructor from a CartesianState
   */
  CartesianPose(const CartesianState& state);

  /**
   * @brief Copy constructor from a CartesianTwist by considering that it is a displacement over 1 second
   */
  CartesianPose(const CartesianTwist& twist);

  /**
   * @brief Construct a CartesianPose from a position given as a vector of coordinates.
   */
  explicit CartesianPose(const std::string& name, const Eigen::Vector3d& position, const std::string& reference = "world");

  /**
   * @brief Construct a CartesianPose from a position given as three scalar coordinates.
   */
  explicit CartesianPose(const std::string& name, const double& x, const double& y, const double& z, const std::string& reference = "world");

  /**
   * @brief Construct a CartesianPose from a position given as a vector of coordinates and a quaternion.
   */
  explicit CartesianPose(const std::string& name, const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation, const std::string& reference = "world");

  /**
   * @brief Constructor for the identity pose
   * @param name the name of the state
   * @param reference the name of the reference frame
   * @return CartesianPose identity pose
   */
  static CartesianPose Identity(const std::string& name, const std::string& reference = "world");

  /**
   * @brief Constructor for a random pose
   * @param name the name of the state
   * @param reference the name of the reference frame
   * @return CartesianPose random pose
   */
  static CartesianPose Random(const std::string& name, const std::string& reference = "world");

  /**
   * @brief Copy assignement operator that have to be defined to the custom assignement operator
   * @param pose the pose with value to assign
   * @return reference to the current pose with new values
   */
  CartesianPose& operator=(const CartesianPose& pose);

  /**
   * @brief Copy assignement operator from a state
   * @param state the state with value to assign
   * @return reference to the current pose with new values
   */
  CartesianPose& operator=(const CartesianState& state);

  /**
   * @brief Overload the * operator for a vector input
   * @param vector vector to multiply with, representing either a position, velocity or acceleration
   * @return the vector multiplied by the current CartesianPose
   */
  Eigen::Vector3d operator*(const Eigen::Vector3d& vector) const;

  /**
   * @brief Overload the *= operator
   * @param pose CartesianPose to multiply with
   * @return the current CartesianPose multiplied by the CartesianPose given in argument
   */
  CartesianPose& operator*=(const CartesianPose& pose);

  /**
   * @brief Overload the * operator
   * @param pose CartesianPose to multiply with
   * @return the current CartesianPose multiplied by the CartesianPose given in argument
   */
  CartesianPose operator*(const CartesianPose& pose) const;

  /**
   * @brief Overload the *= operator with a scalar
   * @param lambda the scalar to multiply with
   * @return the CartesianPose multiplied by lambda
   */
  CartesianPose& operator*=(double lambda);

  /**
   * @brief Overload the * operator with a scalar
   * @param lambda the scalar to multiply with
   * @return the CartesianState multiplied by lambda
   */
  CartesianPose operator*(double lambda) const;

  /**
   * @brief Overload the += operator
   * @param pose CartesianPose to add to
   * @return the current CartesianPose added the CartesianPose given in argument
   */
  CartesianPose& operator+=(const CartesianPose& pose);

  /**
   * @brief Overload the + operator with a pose
   * @param pose CartesianPose to add to
   * @return the current CartesianPose added the CartesianPose given in argument
   */
  CartesianPose operator+(const CartesianPose& pose) const;

  /**
   * @brief Overload the + operator with a state
   * @param state CartesianState to add
   * @return the current CartesianPose added the CartesianState given in argument
   */
  CartesianState operator+(const CartesianState& state) const;

  /**
   * @brief Overload the -= operator
   * @param pose CartesianPose to substract
   * @return the current CartesianPose minus the CartesianPose given in argument
   */
  CartesianPose& operator-=(const CartesianPose& pose);

  /**
   * @brief Overload the - operator with a pose
   * @param pose CartesianPose to substract
   * @return the current CartesianPose minus the CartesianPose given in argument
   */
  CartesianPose operator-(const CartesianPose& pose) const;

  /**
   * @brief Overload the - operator with a state
   * @param state CartesianState to substract
   * @return the current CartesianPose minus the CartesianState given in argument
   */
  CartesianState operator-(const CartesianState& state) const;

  /**
   * @brief Overload the / operator with a time period
   * @param dt the time period to divise by
   * @return the corresponding CartesianTwist
   */
  CartesianTwist operator/(const std::chrono::nanoseconds& dt) const;

  /**
   * @brief Return a copy of the CartesianPose
   * @return the copy
   */
  CartesianPose copy() const;

  /**
   * @brief Returns the pose data as an Eigen vector
   * @return the pose data vector
   */
  Eigen::VectorXd data() const;

  /**
   * @brief Overload the ostream operator for printing
   * @param os the ostream to happend the string representing the CartesianPose to
   * @param CartesianPose the CartesianPose to print
   * @return the appended ostream
   */
  friend std::ostream& operator<<(std::ostream& os, const CartesianPose& pose);

  /**
   * @brief Overload the * operator with a scalar
   * @param lambda the scalar to multiply with
   * @return the CartesianPose provided multiply by lambda
   */
  friend CartesianPose operator*(double lambda, const CartesianPose& pose);

  /**
   * @brief Set the value from a std vector
   * @param value the value as a std vector
   */
  void from_std_vector(const std::vector<double>& value);
};

inline CartesianPose& CartesianPose::operator=(const CartesianPose& pose) {
  CartesianState::operator=(pose);
  return (*this);
}
}// namespace state_representation
