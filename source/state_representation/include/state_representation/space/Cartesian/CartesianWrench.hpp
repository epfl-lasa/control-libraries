/**
 * @author Baptiste Busch
 * @date 2019/09/13
 */

#pragma once

#include "state_representation/space/Cartesian/CartesianState.hpp"

namespace state_representation {
/**
 * @class CartesianWrench
 * @brief Class to define wrench in cartesian space as 3D force and torque vectors
 */
class CartesianWrench : public CartesianState {
public:
  /**
   * Empty constructor
   */
  explicit CartesianWrench();

  /**
   * @brief Empty constructor for a CartesianWrench
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
   * @brief Construct a CartesianWrench from a force given as a vector of coordinates.
   */
  explicit CartesianWrench(const std::string& name, const Eigen::Vector3d& force, const std::string& reference = "world");

  /**
   * @brief Construct a CartesianWrench from a force given as a vector of coordinates and a quaternion.
   */
  explicit CartesianWrench(const std::string& name, const Eigen::Vector3d& force, const Eigen::Vector3d& torque, const std::string& reference = "world");

  /**
   * @brief Construct a CartesianWrench from a single 6d wrench vector
   */
  explicit CartesianWrench(const std::string& name, const Eigen::Matrix<double, 6, 1>& wrench, const std::string& reference = "world");

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
   * @brief Copy assignement operator that have to be defined to the custom assignement operator
   * @param pose the pose with value to assign
   * @return reference to the current pose with new values
   */
  CartesianWrench& operator=(const CartesianWrench& pose);

  /**
   * @brief Overload the = operator from a CartesianState
   * @param state CartesianState to get the wrench from
   */
  CartesianWrench& operator=(const CartesianState& state);

  /**
   * @brief Overload the *= operator
   * @param wrench CartesianWrench to multiply with
   * @return the current CartesianWrench multiplied by the CartesianWrench given in argument
   */
  CartesianWrench& operator*=(const CartesianWrench& twist);

  /**
   * @brief Overload the * operator with a wrench
   * @param wrench CartesianWrench to multiply with
   * @return the current CartesianWrench multiplied by the CartesianWrench given in argument
   */
  CartesianWrench operator*(const CartesianWrench& twist) const;

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
   * @param wrench CartesianWrench to substract
   * @return the current CartesianWrench minus the CartesianWrench given in argument
   */
  CartesianWrench& operator-=(const CartesianWrench& wrench);

  /**
   * @brief Overload the - operator
   * @param wrench CartesianWrench to substract
   * @return the current CartesianWrench minus the CartesianWrench given in argument
   */
  CartesianWrench operator-(const CartesianWrench& wrench) const;

  /**
   * @brief Overload the *= operator with a scalar
   * @param lambda the scalar to multiply with
   * @return the CartesianWrench multiply by lambda
   */
  CartesianWrench& operator*=(double lambda);

  /**
   * @brief Overload the * operator with a scalar
   * @param lambda the scalar to multiply with
   * @return the CartesianWrench multiply by lambda
   */
  CartesianWrench operator*(double lambda) const;

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
  CartesianWrench clamped(double max_force, double max_torque, double force_noise_ratio = 0, double torque_noise_ratio = 0) const;

  /**
   * @brief Return a copy of the CartesianWrench
   * @return the copy
   */
  CartesianWrench copy() const;

  /**
   * @brief Returns the wrench data as an Eigen vector
   * @return the wrench data vector
   */
  Eigen::VectorXd data() const;

  /**
   * @brief Return the value of the 6D wrench as Eigen array
   * @retrun the Eigen array representing the wrench
   */
  Eigen::Array<double, 6, 1> array() const;

  /**
   * @brief Overload the ostream operator for printing
   * @param os the ostream to happend the string representing the CartesianWrench to
   * @param CartesianWrench the CartesianWrench to print
   * @return the appended ostream
   */
  friend std::ostream& operator<<(std::ostream& os, const CartesianWrench& wrench);

  /**
   * @brief Overload the * operator with a scalar
   * @param lambda the scalar to multiply with
   * @return the CartesianWrench provided multiply by lambda
   */
  friend CartesianWrench operator*(double lambda, const CartesianWrench& wrench);
};

inline CartesianWrench& CartesianWrench::operator=(const CartesianWrench& wrench) {
  CartesianState::operator=(wrench);
  return (*this);
}
}// namespace state_representation
