/**
 * @author Baptiste Busch
 * @date 2019/09/09
 */

#pragma once

#include "state_representation/exceptions/IncompatibleSizeException.hpp"
#include "state_representation/robot/JointTorques.hpp"
#include "state_representation/robot/JointVelocities.hpp"
#include "state_representation/space/cartesian/CartesianTwist.hpp"
#include "state_representation/space/cartesian/CartesianWrench.hpp"
#include "state_representation/State.hpp"
#include <eigen3/Eigen/Core>

namespace state_representation {
class CartesianTwist;
class CartesianWrench;
class JointVelocities;
class JointTorques;

/**
 * @class Jacobian
 * @brief Class to define a robot Jacobian matrix
 */
class Jacobian : public State {
private:
  std::vector<std::string> joint_names_;///< names of the joints
  std::string frame_;                   ///< name of the frame at which the jacobian is computed
  std::string reference_frame_;         /// name of the reference frame in which the jacobian is expressed
  unsigned int rows_;                   ///< number of rows
  unsigned int cols_;                   ///< number of columns
  Eigen::MatrixXd data_;                ///< internal storage of the jacobian matrix

public:
  /**
   * @brief Constructor with names and number of joints provided
   * @param robot_name the name of the robot associated to
   * @param nb_joints the number of joints
   * @param frame the name of the frame at which the jacobian is computed
   * @param reference_frame the name of the reference frame in which the jacobian is expressed (default "world")
   */
  explicit Jacobian(const std::string& robot_name,
                    unsigned int nb_joints,
                    const std::string& frame,
                    const std::string& reference_frame = "world");

  /**
   * @brief Constructor with names and list of joint names provided
   * @param joint_names the vector of joint names
   * @param robot_name the name of the robot associated to
   * @param frame the name of the frame at which the jacobian is computed
   * @param reference_frame the name of the reference frame in which the jacobian is expressed (default "world")
   */
  explicit Jacobian(const std::string& robot_name,
                    const std::vector<std::string>& joint_names,
                    const std::string& frame,
                    const std::string& reference_frame = "world");

  /**
   * @brief Constructor with names and jacobian matrix as eigen matrix
   * @param frame the name of the frame at which the jacobian is computed
   * @param data the values of the jacobian matrix
   * @param reference_frame the name of the reference frame in which the jacobian is expressed (default "world")
   */
  explicit Jacobian(const std::string& robot_name,
                    const std::string& frame,
                    const Eigen::MatrixXd& data,
                    const std::string& reference_frame = "world");

  /**
   * @brief Constructor with names, vector of joint names, and jacobian matrix as eigen matrix
   * @param joint_names the vector of joint names
   * @param frame the name of the frame at which the jacobian is computed
   * @param data the values of the jacobian matrix
   * @param reference_frame the name of the reference frame in which the jacobian is expressed (default "world")
   */
  explicit Jacobian(const std::string& robot_name,
                    const std::vector<std::string>& joint_names,
                    const std::string& frame,
                    const Eigen::MatrixXd& data,
                    const std::string& reference_frame = "world");

  /**
   * @brief Copy constructor of a Jacobian
   */
  Jacobian(const Jacobian& jacobian);

  /**
   * @brief Constructor for a random Jacobian
   * @param robot_name the name of the associated robot
   * @param nb_joints the number of joints for initialization
   * @param frame the name of the frame at which the jacobian is computed
   * @param reference_frame the name of the reference frame in which the jacobian is expressed (default "world")
   * @return Jacobian with random data values
   */
  static Jacobian Random(const std::string& robot_name,
                         unsigned int nb_joints,
                         const std::string& frame,
                         const std::string& reference_frame = "world");

  /**
   * @brief Constructor for a random Jacobian
   * @param robot_name the name of the associated robot
   * @param joint_names list of joint names
   * @param frame the name of the frame at which the jacobian is computed
   * @param reference_frame the name of the reference frame in which the jacobian is expressed (default "world")
   * @return Jacobian with random data values
   */
  static Jacobian Random(const std::string& robot_name,
                         const std::vector<std::string>& joint_names,
                         const std::string& frame,
                         const std::string& reference_frame = "world");

  /**
   * @brief Swap the values of the two Jacobian
   * @param jacobian1 Jacobian to be swapped with 2
   * @param jacobian2 Jacobian to be swapped with 1
   */
  friend void swap(Jacobian& jacobian1, Jacobian& jacobian2);

  /**
   * @brief Copy assignment operator that have to be defined to the custom assignment operator
   * @param jacobian the Jacobian with value to assign
   * @return reference to the current Jacobian with new values
   */
  Jacobian& operator=(const Jacobian& jacobian);

  /**
   * @brief Getter of the number of rows attribute
   * @return the number of rows
   */
  unsigned int rows() const;

  /**
   * @brief Setter of the number of rows
   * @param rows the number of rows
   */
  void set_rows(unsigned int rows);

  /**
   * @brief Accessor of the row data at given index
   * @param index the desired index of the row
   * @return the row vector at index
   */
  Eigen::VectorXd row(unsigned int index) const;

  /**
   * @brief Getter of the number of columns attribute
   * @return the number of cols
   */
  unsigned int cols() const;

  /**
   * @brief Setter of the number of columns
   * @param cols the number of columns
   */
  void set_cols(unsigned int cols);

  /**
   * @brief Accessor of the column data at given index
   * @param index the desired index of the column
   * @return the column vector at index
   */
  Eigen::VectorXd col(unsigned int index) const;

  /**
   * @brief Getter of the joint_names attribute
   */
  const std::vector<std::string>& get_joint_names() const;

  /**
   * @brief Setter of the joint_names attribute from the number of joints
   */
  void set_joint_names(unsigned int nb_joints);

  /**
   * @brief Setter of the joint_names attribute from a vector of joint names
   */
  void set_joint_names(const std::vector<std::string>& joint_names);

  /**
   * @brief Getter of the frame_name attribute
   */
  const std::string& get_frame() const;

  /**
   * @brief Getter of the frame_name attribute
   */
  const std::string& get_reference_frame() const;

  /**
   * @brief Setter of the reference_frame attribute from a CartesianPose
   * Update the value of the data matrix accordingly by changing the reference frame of each columns.
   * This means that the computation needs to be compatible, i.e. the previous reference frame should
   * match the name of the new reference frame as input.
   * @param reference_frame the reference frame as a CartesianPose
   */
  void set_reference_frame(const CartesianPose& reference_frame);

  /**
   * @brief Getter of the data attribute
   */
  const Eigen::MatrixXd& data() const;

  /**
   * @brief Setter of the data attribute
   */
  void set_data(const Eigen::MatrixXd& data);

  /**
   * @brief Check if the jacobian matrix is compatible for operations with the state given as argument
   * @param state the state to check compatibility with
   */
  bool is_compatible(const State& state) const override;

  /**
   * @brief Initialize the matrix to a zero value
   */
  void initialize() override;

  /**
    * @brief Return the transpose of the jacobian matrix
    * @return the Jacobian transposed
    */
  Jacobian transpose() const;

  /**
   * @brief Return the inverse of the Jacobian matrix
   * If the matrix is not invertible, an error is thrown advising to use the
   * pseudoinverse function instead
   * @return the inverse of the Jacobian
   */
  Jacobian inverse() const;

  /**
   * @brief Return the pseudoinverse of the Jacobian matrix
   * @return the pseudoinverse of the Jacobian
   */
  Jacobian pseudoinverse() const;

  /**
   * @brief Overload the * operator with an non specific matrix
   * @param matrix the vector to multiply with
   * @return the matrix multiply by the jacobian matrix
   */
  Eigen::MatrixXd operator*(const Eigen::MatrixXd& matrix) const;

  /**
   * @brief Overload the * operator with a JointVelocities
   * @param dq the joint velocity to multiply with
   * @return this result into the CartesianTwist of the end effector
   * the name of the output CartesianTwist will be "robot"_end_effector and
   * the reference frame will be "robot"_base 
   */
  CartesianTwist operator*(const JointVelocities& dq) const;

  /**
   * @brief Overload the * operator with a CartesianTwist.
   * @param twist the cartesian velocity to multiply with
   * @return this result into a JointVelocities
   */
  JointVelocities operator*(const CartesianTwist& twist) const;

  /**
   * @brief Overload the * operator with a CartesianWrench.
   * @param wrench the cartesian wrench to multiply with
   * @return this result into a JointTorques
   */
  JointTorques operator*(const CartesianWrench& wrench) const;

  /**
   * @brief Solve the system X = inv(J)*M to obtain X which is more efficient than multiplying with the pseudo-inverse
   * @param matrix the matrix to solve the system with
   * @return result of X = J.solve(M) from Eigen decomposition
   */
  Eigen::MatrixXd solve(const Eigen::MatrixXd& matrix) const;

  /**
   * @brief Solve the system dX = J*dq to obtain dq which is more efficient than multiplying with the pseudo-inverse
   * @param dX the cartesian velocity to multiply with
   * @return this result into a JointVelocities
   */
  JointVelocities solve(const CartesianTwist& twist) const;

  /**
   * @brief Overload the () operator in a non const fashion to modify the value at given (row, col)
   * @param row the index of the row
   * @param the index of the column
   * @return the reference to the value at the given row and column
   */
  double& operator()(unsigned int row, unsigned int col);

  /**
   * @brief Overload the () operator const fashion to access the value at given (row, col)
   * @param row the index of the row
   * @param the index of the column
   * @return the const reference to the value at the given row and column
   */
  const double& operator()(unsigned int row, unsigned int col) const;

  /**
   * @brief Return a copy of the JointPositions
   * @return the copy
   */
  Jacobian copy() const;

  /**
   * @brief Overload the ostream operator for printing
   * @param os the ostream to append the string representing the matrix to
   * @param jacobian the Jacobian to print
   * @return the appended ostream
   */
  friend std::ostream& operator<<(std::ostream& os, const Jacobian& jacobian);

  /**
   * @brief Overload the * operator with a CartesianPose on left side. This is
   * equivalent to a changing of reference frame of the Jacobian
   * @param pose the CartesianPose to multiply with
   * @param jacobian the Jacobian to be multiplied with the CartesianPose
   * @return the Jacobian expressed in the new reference frame
   */
  friend Jacobian operator*(const CartesianPose& pose, const Jacobian& jacobian);
};

inline void swap(Jacobian& jacobian1, Jacobian& jacobian2) {
  swap(static_cast<State&>(jacobian1), static_cast<State&>(jacobian2));
  std::swap(jacobian1.joint_names_, jacobian2.joint_names_);
  std::swap(jacobian1.frame_, jacobian2.frame_);
  std::swap(jacobian1.reference_frame_, jacobian2.reference_frame_);
  std::swap(jacobian1.cols_, jacobian2.cols_);
  std::swap(jacobian1.rows_, jacobian2.rows_);
  std::swap(jacobian1.data_, jacobian2.data_);
}

inline Jacobian& Jacobian::operator=(const Jacobian& jacobian) {
  Jacobian tmp(jacobian);
  swap(*this, tmp);
  return *this;
}

inline unsigned int Jacobian::rows() const {
  return this->rows_;
}

inline unsigned int Jacobian::cols() const {
  return this->cols_;
}

inline void Jacobian::set_rows(unsigned int rows) {
  this->rows_ = rows;
}

inline void Jacobian::set_cols(unsigned int cols) {
  this->cols_ = cols;
}

inline Eigen::VectorXd Jacobian::row(unsigned int index) const {
  return this->data_.row(index);
}

inline Eigen::VectorXd Jacobian::col(unsigned int index) const {
  return this->data_.col(index);
}

inline const std::vector<std::string>& Jacobian::get_joint_names() const {
  return this->joint_names_;
}

inline void Jacobian::set_joint_names(unsigned int nb_joints) {
  if (this->joint_names_.size() != nb_joints) {
    throw exceptions::IncompatibleSizeException("Input number of joints is of incorrect size, expected "
                                                    + std::to_string(this->joint_names_.size())
                                                    + " got " + std::to_string(nb_joints));
  }
  for (unsigned int i = 0; i < nb_joints; ++i) {
    this->joint_names_[i] = "joint" + std::to_string(i);
  }
}

inline void Jacobian::set_joint_names(const std::vector<std::string>& joint_names) {
  if (this->joint_names_.size() != joint_names.size()) {
    throw exceptions::IncompatibleSizeException("Input vector of joint names is of incorrect size, expected "
                                                    + std::to_string(this->joint_names_.size())
                                                    + " got " + std::to_string(joint_names.size()));
  }
  this->joint_names_ = joint_names;
}

inline const std::string& Jacobian::get_frame() const {
  return this->frame_;
}

inline const std::string& Jacobian::get_reference_frame() const {
  return this->reference_frame_;
}

inline const Eigen::MatrixXd& Jacobian::data() const {
  return this->data_;
}

inline void Jacobian::set_data(const Eigen::MatrixXd& data) {
  if (this->rows() != data.rows() || this->cols() != data.cols()) {
    throw exceptions::IncompatibleSizeException("Input matrix is of incorrect size, expected "
                                                    + std::to_string(this->rows_) + "x" + std::to_string(this->cols_)
                                                    + " got " + std::to_string(data.rows()) + "x"
                                                    + std::to_string(data.cols()));
  }
  this->set_filled();
  this->data_ = data;
}

inline double& Jacobian::operator()(unsigned int row, unsigned int col) {
  if (row > this->rows_) {
    throw std::out_of_range("Given row is out of range: number of rows = " + std::to_string(this->rows_));
  }
  if (col > this->cols_) {
    throw std::out_of_range("Given column is out of range: number of columns = " + std::to_string(this->cols_));
  }
  return this->data_(row, col);
}

inline const double& Jacobian::operator()(unsigned int row, unsigned int col) const {
  if (row > this->rows_) {
    throw std::out_of_range("Given row is out of range: number of rows = " + std::to_string(this->rows_));
  }
  if (col > this->cols_) {
    throw std::out_of_range("Given column is out of range: number of columns = " + std::to_string(this->cols_));
  }
  return this->data_(row, col);
}
}// namespace state_representation
