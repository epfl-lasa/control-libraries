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

using namespace state_representation::Exceptions;

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
  unsigned int nb_rows_;                ///< number of rows
  unsigned int nb_cols_;                ///< number of columns
  Eigen::MatrixXd data_matrix_;         ///< internal storage of the jacobian matrix

public:
  /**
   * @brief Empty constructor for a Jacobian
   */
  explicit Jacobian();

  /**
   * @brief Constructor with name and number of joints provided
   * @brief name the name of the robot associated to
   * @brief nb_joints the number of joints
   */
  explicit Jacobian(const std::string& robot_name, unsigned int nb_joints = 0);

  /**
   * @brief Constructor with name and list of joint names provided
   * @brief name the name of the robot associated to
   * @brief joint_names the list of joint names
   */
  explicit Jacobian(const std::string& robot_name, const std::vector<std::string>& joint_names);

  /**
   * @brief Constructor with name and jacobian matrix as eigen matrix
   * @brief name the name of the robot associated to
   * @brief data the value of the jacobian matrix
   */
  explicit Jacobian(const std::string& robot_name, const Eigen::MatrixXd& data);

  /**
   * @brief Constructor with name, list of joint names and jacobian matrix as eigen matrix
   * @brief name the name of the robot associated to
   * @brief joint_names the list of joint names
   * @brief data the value of the jacobian matrix
   */
  explicit Jacobian(const std::string& robot_name, const std::vector<std::string>& joint_names,
                    const Eigen::MatrixXd& data);

  /**
   * @brief Copy constructor of a Jacobian
   */
  Jacobian(const Jacobian& jacobian);

  /**
   * @brief Copy assignment operator that have to be defined to the custom assignment operator
   * @param matrix the matrix with value to assign
   * @return reference to the current matrix with new values
   */
  Jacobian& operator=(const Jacobian& matrix);

  /**
   * @brief Getter of the nb_rows from the attributes
   */
  unsigned int get_nb_rows() const;

  /**
   * @brief Setter of the nb_rows
   */
  void set_nb_rows(unsigned int nb_rows);

  /**
   * @brief Setter of the nb_cols
   */
  void set_nb_cols(unsigned int nb_cols);

  /**
   * @brief Getter of the nb_cols from the attributes
   */
  unsigned int get_nb_cols() const;

  /**
   * @brief Getter of the names attribute
   */
  const std::vector<std::string>& get_joint_names() const;

  /**
   * @brief Setter of the names attribute from the number of joints
   */
  void set_joint_names(unsigned int nb_joints);

  /**
   * @brief Setter of the names attribute
   */
  void set_joint_names(const std::vector<std::string>& names);

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
  bool is_compatible(const State& state) const;

  /**
   * @brief Initialize the matrix to a zero value
   */
  void initialize();

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
   * @param matrix the matrix to print
   * @return the appended ostream
   */
  friend std::ostream& operator<<(std::ostream& os, const Jacobian& matrix);
};

inline Jacobian& Jacobian::operator=(const Jacobian& matrix) {
  State::operator=(matrix);
  this->joint_names_ = matrix.joint_names_;
  this->nb_cols_ = matrix.nb_cols_;
  this->nb_rows_ = matrix.nb_rows_;
  this->data_matrix_ = matrix.data_matrix_;
  return (*this);
}

inline unsigned int Jacobian::get_nb_rows() const {
  return this->nb_rows_;
}

inline unsigned int Jacobian::get_nb_cols() const {
  return this->nb_cols_;
}

inline void Jacobian::set_nb_rows(unsigned int nb_rows) {
  this->nb_rows_ = nb_rows;
}

inline void Jacobian::set_nb_cols(unsigned int nb_cols) {
  this->nb_cols_ = nb_cols;
}

inline const std::vector<std::string>& Jacobian::get_joint_names() const {
  return this->joint_names_;
}

inline void Jacobian::set_joint_names(unsigned int nb_joints) {
  this->joint_names_.resize(nb_joints);
  this->nb_cols_ = nb_joints;
  for (unsigned int i = 0; i < nb_joints; ++i) {
    this->joint_names_[i] = "joint" + std::to_string(i);
  }
  this->initialize();
}

inline void Jacobian::set_joint_names(const std::vector<std::string>& joint_names) {
  this->joint_names_ = joint_names;
  this->nb_cols_ = joint_names.size();
  this->initialize();
}

inline const Eigen::MatrixXd& Jacobian::data() const {
  return this->data_matrix_;
}

inline void Jacobian::set_data(const Eigen::MatrixXd& data) {
  if (this->get_nb_rows() != data.rows() || this->get_nb_cols() != data.cols()) {
    throw IncompatibleSizeException("Input matrix is of incorrect size");
  }
  this->set_filled();
  this->data_matrix_ = data;
}

inline bool Jacobian::is_compatible(const State& state) const {
  bool compatible = false;
  if (state.get_type() == StateType::JOINTSTATE) {
    compatible = (this->get_name() == state.get_name())
        && (this->get_nb_cols() == static_cast<const JointState&>(state).get_size());
    if (compatible) {
      for (unsigned int i = 0; i < this->get_nb_cols(); ++i) {
        compatible = (compatible && this->joint_names_[i] == static_cast<const JointState&>(state).get_names()[i]);
      }
    }
  }
    // there is no possibilities to check that a correct frame associated to the robot is sent
  else if (state.get_type() == StateType::CARTESIANSTATE) {
    compatible = true;
  }
  return compatible;
}

inline double& Jacobian::operator()(unsigned int row, unsigned int col) {
  if (row > this->get_nb_rows()) {
    throw std::out_of_range("Given row is out of range: number of rows = " + this->get_nb_rows());
  }
  if (col > this->get_nb_cols()) {
    throw std::out_of_range("Given column is out of range: number of columns = " + this->get_nb_cols());
  }
  return this->data_matrix_(row, col);
}

inline const double& Jacobian::operator()(unsigned int row, unsigned int col) const {
  if (row > this->get_nb_rows()) {
    throw std::out_of_range("Given row is out of range: number of rows = " + this->get_nb_rows());
  }
  if (col > this->get_nb_cols()) {
    throw std::out_of_range("Given column is out of range: number of columns = " + this->get_nb_cols());
  }
  return this->data_matrix_(row, col);
}
}// namespace state_representation
