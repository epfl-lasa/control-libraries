/**
 * @author Baptiste Busch
 * @date 2019/06/07
 */

#pragma once

#include "state_representation/space/cartesian/CartesianState.hpp"
#include "state_representation/space/cartesian/CartesianTwist.hpp"
#include "state_representation/space/cartesian/CartesianWrench.hpp"

namespace state_representation {
class CartesianTwist;

/**
 * @class CartesianPose
 * @brief Class to define CartesianPose in cartesian space as 3D position and quaternion based orientation
 */
class CartesianPose : public CartesianState {
private:
  using CartesianState::clamp_state_variable;

public:
  // delete inaccessible getter and setters
  const Eigen::Vector3d& get_linear_velocity() const = delete;
  const Eigen::Vector3d& get_angular_velocity() const = delete;
  Eigen::Matrix<double, 6, 1> get_twist() const = delete;
  const Eigen::Vector3d& get_linear_acceleration() const = delete;
  const Eigen::Vector3d& get_angular_acceleration() const = delete;
  Eigen::Matrix<double, 6, 1> get_accelerations() const = delete;
  const Eigen::Vector3d& get_force() const = delete;
  const Eigen::Vector3d& get_torque() const = delete;
  Eigen::Matrix<double, 6, 1> get_wrench() const = delete;
  void set_linear_velocity(const Eigen::Vector3d& linear_velocity) = delete;
  void set_angular_velocity(const Eigen::Vector3d& angular_velocity) = delete;
  void set_twist(const Eigen::Matrix<double, 6, 1>& twist) = delete;
  void set_linear_acceleration(const Eigen::Vector3d& linear_acceleration) = delete;
  void set_angular_acceleration(const Eigen::Vector3d& angular_acceleration) = delete;
  void set_accelerations(const Eigen::Matrix<double, 6, 1>& accelerations) = delete;
  void set_force(const Eigen::Vector3d& force) = delete;
  void set_torque(const Eigen::Vector3d& torque) = delete;
  void set_wrench(const Eigen::Matrix<double, 6, 1>& wrench) = delete;

  /**
   * @brief Empty constructor
   */
  explicit CartesianPose() = default;

  /**
   * @brief Constructor with name and reference frame provided
   * @param name the name of the state
   * @param reference the name of the reference frame (default is "world")
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
   * @brief Constructor of a CartesianPose from a position given as a vector of coordinates
   * @param name the name of the state
   * @param position the position data given as Eigen vector
   * @param reference the name of the reference frame (default is "world")
   */
  explicit CartesianPose(const std::string& name,
                         const Eigen::Vector3d& position,
                         const std::string& reference = "world");

  /**
   * @brief Constructor of a CartesianPose from a position given as three scalar coordinates
   * @param name the name of the state
   * @param x the x coordinate of the position
   * @param y the y coordinate of the position
   * @param z the z coordinate of the position
   * @param reference the name of the reference frame (default is "world")
   */
  explicit CartesianPose(const std::string& name,
                         double x,
                         double y,
                         double z,
                         const std::string& reference = "world");

  /**
   * @brief Constructor of a CartesianPose from a quaternion
   * @param name the name of the state
   * @param orientation the orientation given as Eigen quaternion
   * @param reference the name of the reference frame (default is "world")
   */
  explicit CartesianPose(const std::string& name,
                         const Eigen::Quaterniond& orientation,
                         const std::string& reference = "world");

  /**
   * @brief Constructor of a CartesianPose from a position given as a vector of coordinates and a quaternion
   * @param name the name of the state
   * @param position the position data given as Eigen vector
   * @param orientation the orientation given as Eigen quaternion
   * @param reference the name of the reference frame (default is "world")
   */
  explicit CartesianPose(const std::string& name,
                         const Eigen::Vector3d& position,
                         const Eigen::Quaterniond& orientation,
                         const std::string& reference = "world");

  /**
   * @brief Constructor for the identity pose
   * @param name the name of the state
   * @param reference the name of the reference frame (default is "world")
   * @return CartesianPose identity pose
   */
  static CartesianPose Identity(const std::string& name, const std::string& reference = "world");

  /**
   * @brief Constructor for a random pose
   * @param name the name of the state
   * @param reference the name of the reference frame (default is "world")
   * @return CartesianPose random pose
   */
  static CartesianPose Random(const std::string& name, const std::string& reference = "world");

  /**
   * @brief Copy assignment operator that have to be defined to the custom assignment operator
   * @param pose the pose with value to assign
   * @return reference to the current pose with new values
   */
  CartesianPose& operator=(const CartesianPose& pose) = default;

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
   * @brief Overload the * operator
   * @param state CartesianState to multiply with
   * @return the current CartesianPose multiplied by the CartesianState given in argument
   */
  CartesianState operator*(const CartesianState& state) const;

  /**
   * @brief Overload the * operator
   * @param twist CartesianTwist to multiply with
   * @return the current CartesianPose multiplied by the CartesianTwist given in argument
   */
  CartesianTwist operator*(const CartesianTwist& twist) const;

  /**
   * @brief Overload the * operator
   * @param wrench CartesianWrench to multiply with
   * @return the current CartesianPose multiplied by the CartesianWrench given in argument
   */
  CartesianWrench operator*(const CartesianWrench& wrench) const;

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
   * @brief Overload the -= operator
   * @param pose CartesianPose to subtract
   * @return the current CartesianPose minus the CartesianPose given in argument
   */
  CartesianPose& operator-=(const CartesianPose& pose);

  /**
   * @brief Overload the - operator with a pose
   * @param pose CartesianPose to subtract
   * @return the current CartesianPose minus the CartesianPose given in argument
   */
  CartesianPose operator-(const CartesianPose& pose) const;

  /**
   * @brief Overload the / operator with a time period
   * @param dt the time period to divide by
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
  Eigen::VectorXd data() const override;

  /**
   * @brief Compute the norms of the state variable specified by the input type (default is full pose)
   * @param state_variable_type the type of state variable to compute the norms on
   * @return the norms of the state variables as a vector
   */
  std::vector<double> norms(const CartesianStateVariable& state_variable_type = CartesianStateVariable::POSE) const override;

  /**
   * @brief Compute the inverse of the current CartesianPose
   * @return the inverse corresponding to b_S_f (assuming this is f_S_b)
   */
  CartesianPose inverse() const;

  /**
   * @brief Compute the normalized pose at the state variable given in argument (default is full pose)
   * @param state_variable_type the type of state variable to compute the norms on
   * @return the normalized pose
   */
  CartesianPose normalized(const CartesianStateVariable& state_variable_type = CartesianStateVariable::POSE) const;

  /**
   * @brief Overload the ostream operator for printing
   * @param os the ostream to append the string representing the CartesianPose to
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
  void from_std_vector(const std::vector<double>& value) override;
};

inline std::vector<double> CartesianPose::norms(const CartesianStateVariable& state_variable_type) const {
  return CartesianState::norms(state_variable_type);
}

inline CartesianPose CartesianPose::normalized(const CartesianStateVariable& state_variable_type) const {
  return CartesianState::normalized(state_variable_type);
}
}// namespace state_representation
