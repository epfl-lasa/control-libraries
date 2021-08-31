#pragma once

#include "state_representation/space/SpatialState.hpp"
#include "state_representation/exceptions/IncompatibleSizeException.hpp"

namespace state_representation {
class CartesianState;

/**
 * @enum CartesianStateAttribute
 * @brief Enum representing the attributes (position, orientation, angular_velocity, ...)
 * of the CartesianState
 */
enum class CartesianStateVariable {
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
 * @param state_variable_type name of the state variable from the CartesianStateVariable structure to apply
 * the distance on. Default ALL for full distance across all dimensions
 * @return the distance between the two states
 */
double dist(
    const CartesianState& s1, const CartesianState& s2,
    const CartesianStateVariable& state_variable_type = CartesianStateVariable::ALL
);

/**
 * @class CartesianState
 * @brief Class to represent a state in Cartesian space
 */
class CartesianState : public SpatialState {
private:
  Eigen::Vector3d position_;            ///< position of the point
  Eigen::Quaterniond orientation_;      ///< orientation of the point
  Eigen::Vector3d linear_velocity_;     ///< linear velocity of the point
  Eigen::Vector3d angular_velocity_;    ///< angular velocity of the point
  Eigen::Vector3d linear_acceleration_; ///< linear acceleration of the point
  Eigen::Vector3d angular_acceleration_;///< angular acceleration of the point
  Eigen::Vector3d force_;               ///< force applied at the point
  Eigen::Vector3d torque_;              ///< torque applied at the point

  /**
   * @brief Set new_values in all the state variables (pose, twist, wrench)
   * @param new_values the new values of the state variables
   */
  void set_all_state_variables(const Eigen::VectorXd& new_values);

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
  void set_state_variable(
      Eigen::Vector3d& linear_state_variable, Eigen::Vector3d& angular_state_variable,
      const Eigen::Matrix<double, 6, 1>& new_value
  );

  /**
   * @brief Set new_value in the provided state_variable (twist, accelerations or wrench)
   * @param linear_state_variable the linear part of the state variable to fill
   * @param angular_state_variable the angular part of the state variable to fill
   * @param new_value the new value of the state variable
   */
  void set_state_variable(
      Eigen::Vector3d& linear_state_variable, Eigen::Vector3d& angular_state_variable,
      const std::vector<double>& new_value
  );

protected:
  /**
   * @brief Getter of the variable value corresponding to the input
   * @param state_variable_type the type of variable to get
   * @return the value of the variable as a vector
   */
  Eigen::VectorXd get_state_variable(const CartesianStateVariable& state_variable_type) const;

  /**
   * @brief Setter of the variable value corresponding to the input
   * @param new_value the new value of the variable
   * @param state_variable_type the type of variable to get
   */
  void set_state_variable(const Eigen::VectorXd& new_value, const CartesianStateVariable& state_variable_type);

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
  CartesianState(const CartesianState& state) = default;

  /**
   * @brief Constructor for the identity CartesianState (identity pose and 0 for the rest)
   * @param name the name of the state
   * @param reference the name of the reference frame
   * @return CartesianState identity state
   */
  static CartesianState Identity(const std::string& name, const std::string& reference = "world");

  /**
   * @brief Constructor for a random state
   * @param name the name of the state
   * @param reference the name of the reference frame
   * @return CartesianState random state
   */
  static CartesianState Random(const std::string& name, const std::string& reference = "world");

  /**
   * @brief Swap the values of the two CartesianState
   * @param state1 CartesianState to be swapped with 2
   * @param state2 CartesianState to be swapped with 1
   */
  friend void swap(CartesianState& state1, CartesianState& state2);

  /**
   * @brief Copy assignment operator that have to be defined to the custom assignment operator
   * @param state the state with value to assign
   * @return reference to the current state with new values
   */
  CartesianState& operator=(const CartesianState& state);

  /**
   * @brief Getter of the position attribute
   */
  const Eigen::Vector3d& get_position() const;

  /**
   * @brief Getter of the orientation attribute
   */
  const Eigen::Quaterniond& get_orientation() const;

  /**
   * @brief Getter of the orientation attribute as Vector4d of coefficients
   * Beware, quaternion coefficients are returned using the (w, x, y, z) convention
   */
  Eigen::Vector4d get_orientation_coefficients() const;

  /**
   * @brief Getter of a pose from position and orientation attributes
   * @return the pose as a 7d vector. Beware, quaternion coefficients are
   * returned using the (w, x, y, z) convention
   */
  Eigen::Matrix<double, 7, 1> get_pose() const;

  /**
   * @brief Getter of a pose from position and orientation attributes
   * @return the pose as a 4x4 transformation matrix
   */
  Eigen::Matrix4d get_transformation_matrix() const;

  /**
   * @brief Getter of the linear velocity attribute 
   */
  const Eigen::Vector3d& get_linear_velocity() const;

  /**
   * @brief Getter of the angular velocity attribute 
   */
  const Eigen::Vector3d& get_angular_velocity() const;

  /**
   * @brief Getter of the 6d twist from linear and angular velocity attributes
   */
  Eigen::Matrix<double, 6, 1> get_twist() const;

  /**
   * @brief Getter of the linear acceleration attribute 
   */
  const Eigen::Vector3d& get_linear_acceleration() const;

  /**
   * @brief Getter of the angular acceleration attribute 
   */
  const Eigen::Vector3d& get_angular_acceleration() const;

  /**
   * @brief Getter of the 6d acceleration from linear and angular acceleration attributes
   */
  Eigen::Matrix<double, 6, 1> get_accelerations() const;

  /**
   * @brief Getter of the force attribute
   */
  const Eigen::Vector3d& get_force() const;

  /**
   * @brief Getter of the torque attribute
   */
  const Eigen::Vector3d& get_torque() const;

  /**
   * @brief Getter of the 6d wrench from force and torque attributes
   */
  Eigen::Matrix<double, 6, 1> get_wrench() const;

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
   * @brief Setter of the pose from both position and orientation as Eigen 7d vector
   * @param pose the pose as a 7d vector. Beware, quaternion coefficients 
   * uses the (w, x, y, z) convention
   */
  void set_pose(const Eigen::Matrix<double, 7, 1>& pose);

  /**
   * @brief Setter of the pose from both position and orientation as std vector
   * @param pose the pose as a 7d vector. Beware, quaternion coefficients
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
   * @brief Setter of the linear and angular velocities from a 6d twist vector
   */
  void set_twist(const Eigen::Matrix<double, 6, 1>& twist);

  /**
   * @brief Setter of the linear acceleration attribute
   */
  void set_linear_acceleration(const Eigen::Vector3d& linear_acceleration);

  /**
   * @brief Setter of the angular velocity attribute
   */
  void set_angular_acceleration(const Eigen::Vector3d& angular_acceleration);

  /**
   * @brief Setter of the linear and angular accelerations from a 6d acceleration vector
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
   * @brief Setter of the force and torque from a 6d wrench vector
   */
  void set_wrench(const Eigen::Matrix<double, 6, 1>& wrench);

  /**
   * @brief Initialize the CartesianState to a zero value
   */
  void initialize() override;

  /**
   * @brief Set the State to a zero value
   */
  void set_zero();

  /**
   * @brief Clamp inplace the norm of the a specific state variable
   * @param max_norm the maximum norm of the state variable
   * @param state_variable_type name of the variable from the CartesianStateVariable structure to clamp
   * @param noise_ratio if provided, this value will be used to apply a dead zone under which
   * the norm of the state variable will be set to 0
   */
  void clamp_state_variable(double max_norm, const CartesianStateVariable& state_variable_type, double noise_ratio = 0);

  /**
   * @brief Return a copy of the CartesianState
   * @return the copy
   */
  CartesianState copy() const;

  /**
   * @brief Return the data as the concatenation of
   * all the state variables in a single vector
   * @return the concatenated data vector
   */
  virtual Eigen::VectorXd data() const;

  /**
   * @brief Return the data vector as an Eigen Array
   * @return the concatenated data array
   */
  Eigen::ArrayXd array() const;

  /**
   * @brief Set the data of the state from
   * all the state variables in a single Eigen vector
   * @param the concatenated data vector
   */
  virtual void set_data(const Eigen::VectorXd& data) override;

  /**
   * @brief Set the data of the state from
   * all the state variables in a single std vector
   * @param the concatenated data vector
   */
  virtual void set_data(const std::vector<double>& data) override;

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
  CartesianState operator*(const CartesianState& state) const;

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
  CartesianState operator*(double lambda) const;

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
  CartesianState operator+(const CartesianState& state) const;

  /**
   * @brief Overload the -= operator
   * @param state CartesianState to subtract
   * @return the current CartesianState minus the CartesianState given in argument
   */
  CartesianState& operator-=(const CartesianState& state);

  /**
   * @brief Overload the - operator
   * @param state CartesianState to subtract
   * @return the current CartesianState minus the CartesianState given in argument
   */
  CartesianState operator-(const CartesianState& state) const;

  /**
   * @brief Compute the inverse of the current CartesianState
   * @return the inverse corresponding to b_S_f (assuming this is f_S_b) 
   */
  CartesianState inverse() const;

  /**
   * @brief Compute the distance to another state as the sum of distances between each features
   * @param state the second state
   * @param state_variable_type name of the variable from the CartesianStateVariable structure to apply
   * the distance on. Default ALL for full distance across all dimensions
   * @return dist the distance value as a double
   */
  double dist(
      const CartesianState& state, const CartesianStateVariable& state_variable_type = CartesianStateVariable::ALL
  ) const;

  /**
   * @brief Compute the norms of the state variable specified by the input type (default is full state)
   * @param state_variable_type the type of state variable to compute the norms on
   * @return the norms of the state variables as a vector
   */
  virtual std::vector<double>
  norms(const CartesianStateVariable& state_variable_type = CartesianStateVariable::ALL) const;

  /**
   * @brief Normalize inplace the state at the state variable given in argument (default is full state)
   * @param state_variable_type the type of state variable to compute the norms on
   */
  void normalize(const CartesianStateVariable& state_variable_type = CartesianStateVariable::ALL);

  /**
   * @brief Compute the normalized state at the state variable given in argument (default is full state)
   * @param state_variable_type the type of state variable to compute the norms on
   * @return the normalized state
   */
  CartesianState normalized(const CartesianStateVariable& state_variable_type = CartesianStateVariable::ALL) const;

  /**
   * @brief Overload the ostream operator for printing
   * @param os the ostream to append the string representing the state to
   * @param state the state to print
   * @return the appended ostream
   */
  friend std::ostream& operator<<(std::ostream& os, const CartesianState& state);

  /**
   * @brief Overload the * operator with a scalar
   * @param lambda the scalar to multiply with
   * @return the CartesianState provided multiply by lambda
   */
  friend CartesianState operator*(double lambda, const CartesianState& state);

  /**
   * @brief compute the distance between two CartesianStates
   * @param s1 the first CartesianState 
   * @param s2 the second CartesianState
   * @param type of the distance between position, orientation, linear_velocity, etc...
   * default all for full distance across all dimensions
   * @return the distance between the two states
   */
  friend double dist(
      const CartesianState& s1, const CartesianState& s2, const CartesianStateVariable& state_variable_type
  );

  /**
   * @brief Return the state as a std vector
   * @return std::vector<double> the state vector as a std vector
   */
  std::vector<double> to_std_vector() const;
};

inline void swap(CartesianState& state1, CartesianState& state2) {
  swap(static_cast<SpatialState&>(state1), static_cast<SpatialState&>(state2));
  std::swap(state1.position_, state2.position_);
  std::swap(state1.orientation_, state2.orientation_);
  std::swap(state1.linear_velocity_, state2.linear_velocity_);
  std::swap(state1.angular_velocity_, state2.angular_velocity_);
  std::swap(state1.linear_acceleration_, state2.linear_acceleration_);
  std::swap(state1.angular_acceleration_, state2.angular_acceleration_);
  std::swap(state1.force_, state2.force_);
  std::swap(state1.torque_, state2.torque_);
}

inline CartesianState& CartesianState::operator=(const CartesianState& state) {
  CartesianState tmp(state);
  swap(*this, tmp);
  return *this;
}

inline const Eigen::Vector3d& CartesianState::get_position() const {
  return this->position_;
}

inline const Eigen::Quaterniond& CartesianState::get_orientation() const {
  return this->orientation_;
}

inline Eigen::Vector4d CartesianState::get_orientation_coefficients() const {
  return Eigen::Vector4d(
      this->get_orientation().w(), this->get_orientation().x(), this->get_orientation().y(),
      this->get_orientation().z()
  );
}

inline Eigen::Matrix<double, 7, 1> CartesianState::get_pose() const {
  Eigen::Matrix<double, 7, 1> pose;
  pose << this->get_position(), this->get_orientation_coefficients();
  return pose;
}

inline Eigen::Matrix4d CartesianState::get_transformation_matrix() const {
  Eigen::Matrix4d pose;
  pose << this->orientation_.toRotationMatrix(), this->position_, 0., 0., 0., 1;
  return pose;
}

inline const Eigen::Vector3d& CartesianState::get_linear_velocity() const {
  return this->linear_velocity_;
}

inline const Eigen::Vector3d& CartesianState::get_angular_velocity() const {
  return this->angular_velocity_;
}

inline Eigen::Matrix<double, 6, 1> CartesianState::get_twist() const {
  Eigen::Matrix<double, 6, 1> twist;
  twist << this->get_linear_velocity(), this->get_angular_velocity();
  return twist;
}

inline const Eigen::Vector3d& CartesianState::get_linear_acceleration() const {
  return this->linear_acceleration_;
}

inline const Eigen::Vector3d& CartesianState::get_angular_acceleration() const {
  return this->angular_acceleration_;
}

inline Eigen::Matrix<double, 6, 1> CartesianState::get_accelerations() const {
  Eigen::Matrix<double, 6, 1> accelerations;
  accelerations << this->get_linear_acceleration(), this->get_angular_acceleration();
  return accelerations;
}

inline const Eigen::Vector3d& CartesianState::get_force() const {
  return this->force_;
}

inline const Eigen::Vector3d& CartesianState::get_torque() const {
  return this->torque_;
}

inline Eigen::Matrix<double, 6, 1> CartesianState::get_wrench() const {
  Eigen::Matrix<double, 6, 1> wrench;
  wrench << this->get_force(), this->get_torque();
  return wrench;
}

inline Eigen::VectorXd CartesianState::get_state_variable(const CartesianStateVariable& state_variable_type) const {
  switch (state_variable_type) {
    case CartesianStateVariable::POSITION:
      return this->get_position();

    case CartesianStateVariable::ORIENTATION:
      return this->get_orientation_coefficients();

    case CartesianStateVariable::POSE:
      return this->get_pose();

    case CartesianStateVariable::LINEAR_VELOCITY:
      return this->get_linear_velocity();

    case CartesianStateVariable::ANGULAR_VELOCITY:
      return this->get_angular_velocity();

    case CartesianStateVariable::TWIST:
      return this->get_twist();

    case CartesianStateVariable::LINEAR_ACCELERATION:
      return this->get_linear_acceleration();

    case CartesianStateVariable::ANGULAR_ACCELERATION:
      return this->get_angular_acceleration();

    case CartesianStateVariable::ACCELERATIONS:
      return this->get_accelerations();

    case CartesianStateVariable::FORCE:
      return this->get_force();

    case CartesianStateVariable::TORQUE:
      return this->get_torque();

    case CartesianStateVariable::WRENCH:
      return this->get_wrench();

    case CartesianStateVariable::ALL:
      Eigen::VectorXd all_fields(25);
      all_fields << this->get_pose(), this->get_twist(), this->get_accelerations(), this->get_wrench();
      return all_fields;
  }
  // this never goes here but is compulsory to avoid a warning
  return Eigen::Vector3d::Zero();
}

inline void CartesianState::set_all_state_variables(const Eigen::VectorXd& new_values) {
  if (new_values.size() != 25) {
    throw state_representation::exceptions::IncompatibleSizeException(
        "Input is of incorrect size: expected 25, given " + std::to_string(new_values.size()));
  }
  this->set_pose(new_values.segment(0, 7));
  this->set_twist(new_values.segment(7, 6));
  this->set_accelerations(new_values.segment(13, 6));
  this->set_wrench(new_values.segment(19, 6));
}

inline void CartesianState::set_state_variable(Eigen::Vector3d& state_variable, const Eigen::Vector3d& new_value) {
  this->set_filled();
  state_variable = new_value;
}

inline void CartesianState::set_state_variable(Eigen::Vector3d& state_variable, const std::vector<double>& new_value) {
  this->set_state_variable(state_variable, Eigen::Vector3d::Map(new_value.data(), new_value.size()));
}

inline void CartesianState::set_state_variable(
    Eigen::Vector3d& linear_state_variable, Eigen::Vector3d& angular_state_variable,
    const Eigen::Matrix<double, 6, 1>& new_value
) {
  this->set_state_variable(linear_state_variable, new_value.head(3));
  this->set_state_variable(angular_state_variable, new_value.tail(3));
}

inline void CartesianState::set_state_variable(
    Eigen::Vector3d& linear_state_variable, Eigen::Vector3d& angular_state_variable,
    const std::vector<double>& new_value
) {
  this->set_state_variable(linear_state_variable, std::vector<double>(new_value.begin(), new_value.begin() + 3));
  this->set_state_variable(angular_state_variable, std::vector<double>(new_value.begin() + 3, new_value.end()));
}

inline void CartesianState::set_position(const Eigen::Vector3d& position) {
  this->set_state_variable(this->position_, position);
}

inline void CartesianState::set_position(const std::vector<double>& position) {
  this->set_state_variable(this->position_, position);
}

inline void CartesianState::set_position(const double& x, const double& y, const double& z) {
  this->set_position(Eigen::Vector3d(x, y, z));
}

inline void CartesianState::set_orientation(const Eigen::Quaterniond& orientation) {
  this->set_filled();
  this->orientation_ = orientation.normalized();
}

inline void CartesianState::set_orientation(const Eigen::Vector4d& orientation) {
  this->set_orientation(Eigen::Quaterniond(orientation(0), orientation(1), orientation(2), orientation(3)));
}

inline void CartesianState::set_orientation(const std::vector<double>& orientation) {
  if (orientation.size() != 4) {
    throw exceptions::IncompatibleSizeException("The input vector is not of size 4 required for orientation");
  }
  this->set_orientation(Eigen::Vector4d::Map(orientation.data(), orientation.size()));
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
  if (pose.size() != 7) {
    throw exceptions::IncompatibleSizeException("The input vector is not of size 7 required for pose");
  }
  this->set_position(std::vector<double>(pose.begin(), pose.begin() + 3));
  this->set_orientation(std::vector<double>(pose.begin() + 3, pose.end()));
}

inline void CartesianState::set_linear_velocity(const Eigen::Vector3d& linear_velocity) {
  this->set_state_variable(this->linear_velocity_, linear_velocity);
}

inline void CartesianState::set_angular_velocity(const Eigen::Vector3d& angular_velocity) {
  this->set_state_variable(this->angular_velocity_, angular_velocity);
}

inline void CartesianState::set_twist(const Eigen::Matrix<double, 6, 1>& twist) {
  this->set_state_variable(this->linear_velocity_, this->angular_velocity_, twist);
}

inline void CartesianState::set_linear_acceleration(const Eigen::Vector3d& linear_acceleration) {
  this->set_state_variable(this->linear_acceleration_, linear_acceleration);
}

inline void CartesianState::set_angular_acceleration(const Eigen::Vector3d& angular_acceleration) {
  this->set_state_variable(this->angular_acceleration_, angular_acceleration);
}

inline void CartesianState::set_accelerations(const Eigen::Matrix<double, 6, 1>& accelerations) {
  this->set_state_variable(this->linear_acceleration_, this->angular_acceleration_, accelerations);
}

inline void CartesianState::set_force(const Eigen::Vector3d& force) {
  this->set_state_variable(this->force_, force);
}

inline void CartesianState::set_torque(const Eigen::Vector3d& torque) {
  this->set_state_variable(this->torque_, torque);
}

inline void CartesianState::set_wrench(const Eigen::Matrix<double, 6, 1>& wrench) {
  this->set_state_variable(this->force_, this->torque_, wrench);
}

inline void CartesianState::set_state_variable(
    const Eigen::VectorXd& new_value, const CartesianStateVariable& state_variable_type
) {
  switch (state_variable_type) {
    case CartesianStateVariable::POSITION:
      this->set_position(new_value);
      break;

    case CartesianStateVariable::ORIENTATION:
      this->set_orientation(new_value);
      break;

    case CartesianStateVariable::POSE:
      this->set_pose(new_value);
      break;

    case CartesianStateVariable::LINEAR_VELOCITY:
      this->set_linear_velocity(new_value);
      break;

    case CartesianStateVariable::ANGULAR_VELOCITY:
      this->set_angular_velocity(new_value);
      break;

    case CartesianStateVariable::TWIST:
      this->set_twist(new_value);
      break;

    case CartesianStateVariable::LINEAR_ACCELERATION:
      this->set_linear_acceleration(new_value);
      break;

    case CartesianStateVariable::ANGULAR_ACCELERATION:
      this->set_angular_acceleration(new_value);
      break;

    case CartesianStateVariable::ACCELERATIONS:
      this->set_accelerations(new_value);
      break;

    case CartesianStateVariable::FORCE:
      this->set_force(new_value);
      break;

    case CartesianStateVariable::TORQUE:
      this->set_torque(new_value);
      break;

    case CartesianStateVariable::WRENCH:
      this->set_wrench(new_value);
      break;

    case CartesianStateVariable::ALL:
      this->set_pose(new_value.segment(0, 7));
      this->set_twist(new_value.segment(7, 6));
      this->set_accelerations(new_value.segment(13, 6));
      this->set_wrench(new_value.segment(19, 6));
      break;
  }
}

inline std::vector<double> CartesianState::to_std_vector() const {
  Eigen::VectorXd data = this->data();
  return std::vector<double>(data.data(), data.data() + data.size());
}
}// namespace state_representation