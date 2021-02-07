/**
 * @author Baptiste Busch
 * @date 2019/04/16
 */

#pragma once

#include "state_representation/Exceptions/IncompatibleSizeException.hpp"
#include "state_representation/Space/SpatialState.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

namespace StateRepresentation {
class CartesianState;

/**
 * @enum CartesianStateFields
 * @brief Enum representing all the fields (position, orientation, angular_velocity, ...)
 * of the CartesianState
 */
enum class CartesianStateFields {
  POSITION,
  ORIENTATION,
  POSE,
  LINEAR_VELOCITY,
  ANGULAR_VELOCITY,
  TWIST,
  LINEAR_ACCELERATION,
  ANGULAR_ACCELERATION,
  ACCELERATIONS,
  FORCE,
  TORQUE,
  WRENCH,
  ALL
};

/**
 * @brief compute the distance between two CartesianStates
 * @param s1 the first CartesianState
 * @param s2 the second CartesianState
 * @param field_name name of the field from the CartesianStateFields structure to apply
 * the distance on. Default ALL for full distance across all dimensions
 * @return the distance beteen the two states
 */
double dist(const CartesianState& s1, const CartesianState& s2, const CartesianStateFields& field_name = CartesianStateFields::ALL);

/**
 * @class CartesianState
 * @brief Class to represent a state in Cartesian space
 */
class CartesianState : public SpatialState {
private:
  Eigen::Vector3d position;            ///< position of the point
  Eigen::Quaterniond orientation;      ///< orientation of the point
  Eigen::Vector3d linear_velocity;     ///< linear_velocity of the point
  Eigen::Vector3d angular_velocity;    ///< angular_velocity of the point
  Eigen::Vector3d linear_acceleration; ///< linear_acceleration of the point
  Eigen::Vector3d angular_acceleration;///< angular_acceleration of the point
  Eigen::Vector3d force;               ///< force applied at the point
  Eigen::Vector3d torque;              ///< torque applied at the point

  /**
   * @brief Set new_value in the provided state_variable (positions, velocities, accelerations or torques)
   * @param state_variable the state variable to fill
   * @param new_value the new value of the state variable
   */
  void set_state_variable(Eigen::Vector3d& state_variable, const Eigen::Vector3d& new_value);

  /**
   * @brief Set new_value in the provided state_variable (positions, velocities, accelerations or torques)
   * @param state_variable the state variable to fill
   * @param new_value the new value of the state variable
   */
  void set_state_variable(Eigen::Vector3d& state_variable, const std::vector<double>& new_value);

  /**
   * @brief Set new_value in the provided state_variable (twist, accelerations or wrench)
   * @param linear_state_variable the linear part of the state variable to fill
   * @param angular_state_variable the angular part of the state variable to fill
   * @param new_value the new value of the state variable
   */
  void set_state_variable(Eigen::Vector3d& linear_state_variable, Eigen::Vector3d& angular_state_variable, const Eigen::Matrix<double, 6, 1>& new_value);

  /**
   * @brief Set new_value in the provided state_variable (twist, accelerations or wrench)
   * @param linear_state_variable the linear part of the state variable to fill
   * @param angular_state_variable the angular part of the state variable to fill
   * @param new_value the new value of the state variable
   */
  void set_state_variable(Eigen::Vector3d& linear_state_variable, Eigen::Vector3d& angular_state_variable, const std::vector<double>& new_value);

protected:
  const Eigen::VectorXd get_field(const CartesianStateFields& field_name) const;

  void set_field(const Eigen::VectorXd& field_value, const CartesianStateFields& field_name);

public:
  /**
   * @brief Empty constructor
   */
  explicit CartesianState();

  /**
   * @brief Constructor with name and reference frame provided
   * @brief name the name of the state
   * @brief reference the name of the reference frame
   */
  explicit CartesianState(const std::string& name, const std::string& reference = "world");

  /**
   * @brief Copy constructor of a CartesianState
   */
  CartesianState(const CartesianState& state);

  /**
   * @brief Copy assignement operator that have to be defined to the custom assignement operator
   * @param state the state with value to assign
   * @return reference to the current state with new values
   */
  CartesianState& operator=(const CartesianState& state);

  /**
   * @brief Getter of the posistion attribute
   */
  const Eigen::Vector3d& get_position() const;

  /**
   * @brief Getter of the orientation attribute
   */
  const Eigen::Quaterniond& get_orientation() const;

  /**
   * @brief Getter of the orientation attribute as Vector4d of coefficients.
   * Beware, quaternion coefficients are returned using the (w, x, y, z) convention
   */
  const Eigen::Vector4d get_orientation_coefficients() const;

  /**
   * @brief Getter of a pose from position and orientation attributes
   * @return the pose as a 7d vector. Beware, quaternion coefficients are
   * returned using the (w, x, y, z) convention
   */
  const Eigen::Matrix<double, 7, 1> get_pose() const;

  /**
   * @brief Getter of a pose from position and orientation attributes
   * @return the pose as a 4x4 transformation matrix
   */
  const Eigen::Matrix4d get_transformation_matrix() const;

  /**
   * @brief Getter of the linear velocity attribute 
   */
  const Eigen::Vector3d& get_linear_velocity() const;

  /**
   * @brief Getter of the angular velocity attribute 
   */
  const Eigen::Vector3d& get_angular_velocity() const;

  /**
   * @brief Getter of a twist from linear and angular velocities attribute
   */
  const Eigen::Matrix<double, 6, 1> get_twist() const;

  /**
   * @brief Getter of the linear acceleration attribute 
   */
  const Eigen::Vector3d& get_linear_acceleration() const;

  /**
   * @brief Getter of the angular acceleration attribute 
   */
  const Eigen::Vector3d& get_angular_acceleration() const;

  /**
   * @brief Getter of accelerations from linear and angular accelerations attribute
   */
  const Eigen::Matrix<double, 6, 1> get_accelerations() const;

  /**
   * @brief Getter of the force attribute
   */
  const Eigen::Vector3d& get_force() const;

  /**
   * @brief Getter of the torque attribute
   */
  const Eigen::Vector3d& get_torque() const;

  /**
   * @brief Getter of a wrench from force and torque attributes
   */
  const Eigen::Matrix<double, 6, 1> get_wrench() const;

  /**
   * @brief Setter of the position
   */
  void set_position(const Eigen::Vector3d& position);

  /**
   * @brief Setter of the position from a std vector
   */
  void set_position(const std::vector<double>& position);

  /**
   * @brief Setter of the position from three scalar coordinates
   */
  void set_position(const double& x, const double& y, const double& z);

  /**
   * @brief Setter of the orientation
   */
  void set_orientation(const Eigen::Quaterniond& orientation);

  /**
   * @brief Setter of the orientation from a 4d vector
   * @param the orientation coefficients as a 4d vector. Beware, quaternion coefficients 
   * uses the (w, x, y, z) convention
   */
  void set_orientation(const Eigen::Vector4d& orientation);

  /**
   * @brief Setter of the orientation from a std vector
   * @param the orientation coefficients as a 4d vector. Beware, quaternion coefficients 
   * uses the (w, x, y, z) convention
   */
  void set_orientation(const std::vector<double>& orientation);

  /**
   * @brief Setter of the pose from both position and orientation
   * @param position the position
   * @param orientation the orientation
   */
  void set_pose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation);

  /**
   * @brief Setter of the pose from both position and orientation as Eigen 7D vector
   * @param pose the pose as a 7d vector. Beware, quaternion coefficients 
   * uses the (w, x, y, z) convention
   */
  void set_pose(const Eigen::Matrix<double, 7, 1>& pose);

  /**
   * @brief Setter of the pose from both position and orientation as std vector
   * @param pose the pose as a 7d vectorz. Beware, quaternion coefficients 
   * uses the (w, x, y, z) convention
   */
  void set_pose(const std::vector<double>& pose);

  /**
   * @brief Setter of the linear velocity attribute
   */
  void set_linear_velocity(const Eigen::Vector3d& linear_velocity);

  /**
   * @brief Setter of the angular velocity attribute
   */
  void set_angular_velocity(const Eigen::Vector3d& angular_velocity);

  /**
   * @brief Setter of the linear and angular velocities from a single twist vector
   */
  void set_twist(const Eigen::Matrix<double, 6, 1>& twist);

  /**
   * @brief Setter of the linear accelration attribute
   */
  void set_linear_acceleration(const Eigen::Vector3d& linear_acceleration);

  /**
   * @brief Setter of the angular velocity attribute
   */
  void set_angular_acceleration(const Eigen::Vector3d& angular_acceleration);

  /**
   * @brief Setter of the linear and angular accelerations from a single acceleration vector
   */
  void set_accelerations(const Eigen::Matrix<double, 6, 1>& accelerations);

  /**
   * @brief Setter of the force attribute
   */
  void set_force(const Eigen::Vector3d& force);

  /**
   * @brief Setter of the force attribute
   */
  void set_torque(const Eigen::Vector3d& torque);

  /**
   * @brief Setter of the force and torque from a single wrench vector
   */
  void set_wrench(const Eigen::Matrix<double, 6, 1>& wrench);

  /**
   * @brief Initialize the CartesianState to a zero value
   */
  void initialize();

  /**
   * @brief Set the State to a zero value
   */
  void set_zero();

  /**
   * @brief Clamp inplace the magnitude of the a specific field (velocity, acceleration or force)
   * @param max_value the maximum absolute magnitude of the field
   * @param field_name name of the field from the CartesianStateFields structure to clamp
   * @param noise_ratio if provided, this value will be used to apply a deadzone under which
   * the velocity will be set to 0
   */
  void clamp_field(double max_value, const CartesianStateFields& field_name, double noise_ratio = 0);

  /**
   * @brief Return a copy of the CartesianState
   * @return the copy
   */
  const CartesianState copy() const;

  /**
   * @brief Overload the *= operator with another state by deriving the equations of motions
   * @param state the state to compose with corresponding to b_S_c
   * @return the CartesianState corresponding f_S_c = f_S_b * b_S_c (assuming this is f_S_b)
   */
  CartesianState& operator*=(const CartesianState& state);

  /**
   * @brief Overload the * operator with another state by deriving the equations of motions
   * @param state the state to compose with corresponding to b_S_c
   * @return the CartesianState corresponding f_S_c = f_S_b * b_S_c (assuming this is f_S_b)
   */
  const CartesianState operator*(const CartesianState& state) const;

  /**
   * @brief Overload the *= operator with a scalar
   * @param lambda the scalar to multiply with
   * @return the CartesianState multiplied by lambda
   */
  CartesianState& operator*=(double lambda);

  /**
   * @brief Overload the * operator with a scalar
   * @param lambda the scalar to multiply with
   * @return the CartesianState multiplied by lambda
   */
  const CartesianState operator*(double lambda) const;

  /**
   * @brief Overload the += operator
   * @param state CartesianState to add
   * @return the current CartesianState added the CartesianState given in argument
   */
  CartesianState& operator+=(const CartesianState& state);

  /**
   * @brief Overload the + operator
   * @param state CartesianState to add
   * @return the current CartesianState added the CartesianState given in argument
   */
  const CartesianState operator+(const CartesianState& state) const;

  /**
   * @brief Overload the -= operator
   * @param state CartesianState to substract
   * @return the current CartesianState minus the CartesianState given in argument
   */
  CartesianState& operator-=(const CartesianState& state);

  /**
   * @brief Overload the - operator
   * @param state CartesianState to substract
   * @return the current CartesianState minus the CartesianState given in argument
   */
  const CartesianState operator-(const CartesianState& state) const;

  /**
   * @brief compute the inverse of the current CartesianState
   * @return the inverse corresponding to b_S_f (assuming this is f_S_b) 
   */
  const CartesianState inverse() const;

  /**
   * @brief Compute the distance between two states as the sum of distances between each features
   * @param state the second state
   * @param field_name name of the field from the CartesianStateFields structure to apply
   * the distance on. Default ALL for full distance across all dimensions
   * @return dist the distance value as a double
   */
  double dist(const CartesianState& state, const CartesianStateFields& field_name = CartesianStateFields::ALL) const;

  /**
   * @brief Overload the ostream operator for printing
   * @param os the ostream to happend the string representing the state to
   * @param state the state to print
   * @return the appended ostream
   */
  friend std::ostream& operator<<(std::ostream& os, const CartesianState& state);

  /**
   * @brief Overload the * operator with a scalar
   * @param lambda the scalar to multiply with
   * @return the CartesianState provided multiply by lambda
   */
  friend const CartesianState operator*(double lambda, const CartesianState& state);

  /**
   * @brief compute the distance between two CartesianStates
   * @param s1 the first CartesianState 
   * @param s2 the second CartesianState
   * @param type of the distance between position, orientation, linear_velocity, etc...
   * default all for full distance across all dimensions
   * @return the distance beteen the two states
   */
  friend double dist(const CartesianState& s1, const CartesianState& s2, const std::string& distance_type);

  /**
   * @brief Return the state as a std vector of floats
   * @return std::vector<float> the state vector as a std vector
   */
  virtual const std::vector<double> to_std_vector() const;

  /**
   * @brief Set the value from a std vector
   * @param value the value as a std vector
   */
  virtual void from_std_vector(const std::vector<double>& value);
};

inline CartesianState& CartesianState::operator=(const CartesianState& state) {
  SpatialState::operator=(state);
  this->set_pose(state.get_pose());
  this->set_twist(state.get_twist());
  this->set_accelerations(state.get_accelerations());
  this->set_wrench(state.get_wrench());
  return (*this);
}

inline const Eigen::Vector3d& CartesianState::get_position() const {
  return this->position;
}

inline const Eigen::Quaterniond& CartesianState::get_orientation() const {
  return this->orientation;
}

inline const Eigen::Vector4d CartesianState::get_orientation_coefficients() const {
  return Eigen::Vector4d(this->get_orientation().w(), this->get_orientation().x(), this->get_orientation().y(), this->get_orientation().z());
}

inline const Eigen::Matrix<double, 7, 1> CartesianState::get_pose() const {
  Eigen::Matrix<double, 7, 1> pose;
  pose << this->get_position(), this->get_orientation_coefficients();
  return pose;
}

inline const Eigen::Matrix4d CartesianState::get_transformation_matrix() const {
  Eigen::Matrix4d pose;
  pose << this->orientation.toRotationMatrix(), this->position, 0., 0., 0., 1;
  return pose;
}

inline const Eigen::Vector3d& CartesianState::get_linear_velocity() const {
  return this->linear_velocity;
}

inline const Eigen::Vector3d& CartesianState::get_angular_velocity() const {
  return this->angular_velocity;
}

inline const Eigen::Matrix<double, 6, 1> CartesianState::get_twist() const {
  Eigen::Matrix<double, 6, 1> twist;
  twist << this->get_linear_velocity(), this->get_angular_velocity();
  return twist;
}

inline const Eigen::Vector3d& CartesianState::get_linear_acceleration() const {
  return this->linear_acceleration;
}

inline const Eigen::Vector3d& CartesianState::get_angular_acceleration() const {
  return this->angular_acceleration;
}

inline const Eigen::Matrix<double, 6, 1> CartesianState::get_accelerations() const {
  Eigen::Matrix<double, 6, 1> accelerations;
  accelerations << this->get_linear_acceleration(), this->get_angular_acceleration();
  return accelerations;
}

inline const Eigen::Vector3d& CartesianState::get_force() const {
  return this->force;
}

inline const Eigen::Vector3d& CartesianState::get_torque() const {
  return this->torque;
}

inline const Eigen::Matrix<double, 6, 1> CartesianState::get_wrench() const {
  Eigen::Matrix<double, 6, 1> wrench;
  wrench << this->get_force(), this->get_torque();
  return wrench;
}

inline const Eigen::VectorXd CartesianState::get_field(const CartesianStateFields& field_name) const {
  switch (field_name) {
    case CartesianStateFields::POSITION:
      return this->get_position();

    case CartesianStateFields::ORIENTATION:
      return this->get_orientation_coefficients();

    case CartesianStateFields::POSE:
      return this->get_pose();

    case CartesianStateFields::LINEAR_VELOCITY:
      return this->get_linear_velocity();

    case CartesianStateFields::ANGULAR_VELOCITY:
      return this->get_angular_velocity();

    case CartesianStateFields::TWIST:
      return this->get_twist();

    case CartesianStateFields::LINEAR_ACCELERATION:
      return this->get_linear_acceleration();

    case CartesianStateFields::ANGULAR_ACCELERATION:
      return this->get_angular_acceleration();

    case CartesianStateFields::ACCELERATIONS:
      return this->get_accelerations();

    case CartesianStateFields::FORCE:
      return this->get_force();

    case CartesianStateFields::TORQUE:
      return this->get_torque();

    case CartesianStateFields::WRENCH:
      return this->get_wrench();

    case CartesianStateFields::ALL:
      Eigen::Matrix<double, 25, 1> all_fields;
      all_fields << this->get_pose(), this->get_twist(), this->get_accelerations(), this->get_wrench();
      return all_fields;
  }
  // this never goes here but is compulsory to avoid a warning
  return Eigen::Vector3d::Zero();
}

inline void CartesianState::set_state_variable(Eigen::Vector3d& state_variable, const Eigen::Vector3d& new_value) {
  this->set_filled();
  state_variable = new_value;
}

inline void CartesianState::set_state_variable(Eigen::Vector3d& state_variable, const std::vector<double>& new_value) {
  this->set_state_variable(state_variable, Eigen::Vector3d::Map(new_value.data(), new_value.size()));
}

inline void CartesianState::set_state_variable(Eigen::Vector3d& linear_state_variable, Eigen::Vector3d& angular_state_variable, const Eigen::Matrix<double, 6, 1>& new_value) {
  this->set_state_variable(linear_state_variable, new_value.head(3));
  this->set_state_variable(angular_state_variable, new_value.tail(3));
}

inline void CartesianState::set_state_variable(Eigen::Vector3d& linear_state_variable, Eigen::Vector3d& angular_state_variable, const std::vector<double>& new_value) {
  this->set_state_variable(linear_state_variable, std::vector<double>(new_value.begin(), new_value.begin() + 3));
  this->set_state_variable(angular_state_variable, std::vector<double>(new_value.begin() + 4, new_value.end()));
}

inline void CartesianState::set_position(const Eigen::Vector3d& position) {
  this->set_state_variable(this->position, position);
}

inline void CartesianState::set_position(const std::vector<double>& position) {
  this->set_state_variable(this->position, position);
}

inline void CartesianState::set_position(const double& x, const double& y, const double& z) {
  this->set_position(Eigen::Vector3d(x, y, z));
}

inline void CartesianState::set_orientation(const Eigen::Quaterniond& orientation) {
  this->set_filled();
  this->orientation = orientation.normalized();
}

inline void CartesianState::set_orientation(const Eigen::Vector4d& orientation) {
  this->set_orientation(Eigen::Quaterniond(orientation(0), orientation(1), orientation(2), orientation(3)));
}

inline void CartesianState::set_orientation(const std::vector<double>& orientation) {
  if (orientation.size() != 4) throw Exceptions::IncompatibleSizeException("The input vector is not of size 4 required for orientation");
  this->set_orientation(Eigen::Quaterniond(orientation[0], orientation[1], orientation[2], orientation[3]));
}

inline void CartesianState::set_pose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) {
  this->set_position(position);
  this->set_orientation(orientation);
}

inline void CartesianState::set_pose(const Eigen::Matrix<double, 7, 1>& pose) {
  this->set_position(pose.head(3));
  this->set_orientation(pose.tail(4));
}

inline void CartesianState::set_pose(const std::vector<double>& pose) {
  if (pose.size() != 7) throw Exceptions::IncompatibleSizeException("The input vector is not of size 7 required for pose");
  this->set_position(std::vector<double>(pose.begin(), pose.begin() + 3));
  this->set_orientation(std::vector<double>(pose.begin() + 4, pose.end()));
}

inline void CartesianState::set_linear_velocity(const Eigen::Vector3d& linear_velocity) {
  this->set_state_variable(this->linear_velocity, linear_velocity);
}

inline void CartesianState::set_angular_velocity(const Eigen::Vector3d& angular_velocity) {
  this->set_state_variable(this->angular_velocity, angular_velocity);
}

inline void CartesianState::set_twist(const Eigen::Matrix<double, 6, 1>& twist) {
  this->set_state_variable(this->linear_velocity, this->angular_velocity, twist);
}

inline void CartesianState::set_linear_acceleration(const Eigen::Vector3d& linear_acceleration) {
  this->set_state_variable(this->linear_acceleration, linear_acceleration);
}

inline void CartesianState::set_angular_acceleration(const Eigen::Vector3d& angular_acceleration) {
  this->set_state_variable(this->angular_acceleration, angular_acceleration);
}

inline void CartesianState::set_accelerations(const Eigen::Matrix<double, 6, 1>& accelerations) {
  this->set_state_variable(this->linear_acceleration, this->angular_acceleration, accelerations);
}

inline void CartesianState::set_force(const Eigen::Vector3d& force) {
  this->set_state_variable(this->force, force);
}

inline void CartesianState::set_torque(const Eigen::Vector3d& torque) {
  this->set_state_variable(this->torque, torque);
}

inline void CartesianState::set_wrench(const Eigen::Matrix<double, 6, 1>& wrench) {
  this->set_state_variable(this->force, this->torque, wrench);
}

inline void CartesianState::set_field(const Eigen::VectorXd& field_value, const CartesianStateFields& field_name) {
  switch (field_name) {
    case CartesianStateFields::POSITION:
      this->set_position(field_value);
      break;

    case CartesianStateFields::ORIENTATION:
      this->set_orientation(field_value);
      break;

    case CartesianStateFields::POSE:
      this->set_pose(field_value);
      break;

    case CartesianStateFields::LINEAR_VELOCITY:
      this->set_linear_velocity(field_value);
      break;

    case CartesianStateFields::ANGULAR_VELOCITY:
      this->set_angular_velocity(field_value);
      break;

    case CartesianStateFields::TWIST:
      this->set_twist(field_value);
      break;

    case CartesianStateFields::LINEAR_ACCELERATION:
      this->set_linear_acceleration(field_value);
      break;

    case CartesianStateFields::ANGULAR_ACCELERATION:
      this->set_angular_acceleration(field_value);
      break;

    case CartesianStateFields::ACCELERATIONS:
      this->set_accelerations(field_value);
      break;

    case CartesianStateFields::FORCE:
      this->set_force(field_value);
      break;

    case CartesianStateFields::TORQUE:
      this->set_torque(field_value);
      break;

    case CartesianStateFields::WRENCH:
      this->set_wrench(field_value);
      break;

    case CartesianStateFields::ALL:
      this->set_pose(field_value.segment(0, 7));
      this->set_twist(field_value.segment(7, 6));
      this->set_accelerations(field_value.segment(13, 6));
      this->set_wrench(field_value.segment(19, 6));
      break;
  }
}
}// namespace StateRepresentation