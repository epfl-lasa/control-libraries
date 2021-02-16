/**
 * @author Baptiste Busch
 * @date 2021/02
 *
 */

#pragma once

#include "controllers/impedance/Impedance.hpp"
#include "state_representation/Parameters/Parameter.hpp"
#include "state_representation/Space/Cartesian/CartesianState.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

namespace controllers {
namespace impedance {
/**
 * @enum ComputationalSpaceType
 * @brief Selector of the space in which the controller should be computed
 * LINEAR and ANGULAR only compute the command in linear and angular space
 * respectively, TWIST applies it on the full twist vector, and
 * DECOUPLED_TWIST (default) computes the damping matrix for the linear
 * and angular part separatly
 */
enum class ComputationalSpaceType {
  LINEAR,
  ANGULAR,
  TWIST,
  DECOUPLED_TWIST
};

/**
 * @class Dissipative
 * @brief Definition of a dissipative impedance controller (PassiveDS) in task space
 */
class Dissipative : public Impedance<StateRepresentation::CartesianState> {
private:
  ComputationalSpaceType computational_space_;                                          ///< the space in which to compute the command vector
  Eigen::Matrix<double, 6, 6> basis_;                                                   ///< basis matrix used to compute the damping matrix
  std::shared_ptr<StateRepresentation::Parameter<Eigen::VectorXd>> damping_eigenvalues_;///< coefficient of eigenvalues used in the damping matrix computation

public:
  /**
   * @brief Constructor with specified computational space. Default is DECOUPLED_TWIST
   */
  explicit Dissipative(const ComputationalSpaceType& computational_space = ComputationalSpaceType::DECOUPLED_TWIST);

  /**
   * @brief Compute the orthonormal basis based on the main eigenvector in parameter
   * @param basis the basis matrix to orthonormalize
   * @param main_eigenvector the main eigenvector used to compute the basis.
   * Other eigenvectora are orthogonal and selected randomly
   */
  Eigen::MatrixXd compute_orthonormal_basis(const Eigen::MatrixXd& basis, const Eigen::VectorXd& main_eigenvector) const;

  /**
   * @brief Compute the damping matrix as the orthonormal basis
   * to the direction of the desired velocity
   * @param desired_velocity the velocity from which the direction
   * of motion is extracted to compute the basis
   */
  void compute_damping(const Eigen::VectorXd& desired_velocity);

  /**
   * @brief Getter of the damping eigenvalues parameter
   * @return the eigenvalues as a Vetor6D
   */
  Eigen::Matrix<double, 6, 1> get_damping_eigenvalues() const;

  /**
   * @brief Getter of the damping eigenvalue corresponding to the specified index
   * @param index the index of the eigenvalue (number between 0 and 5)
   * @return the eigenvalue at desired index
   */
  double get_damping_eigenvalue(unsigned int index) const;

  /**
   * @brief Setter of the damping eigenvalues parameter
   * @param damping_eigenvalues the new values of the eigenvalue as a Vector6D
   */
  void set_damping_eigenvalues(const Eigen::Matrix<double, 6, 1>& damping_eigenvalues);

  /**
   * @brief Setter of the damping eigenvalue corresponding to the specified index
   * @param damping_eigenvalue the new value of the eigenvalue
   * @param index the index of the eigenvalue (number between 0 and 5)
   */
  void set_damping_eigenvalue(double damping_eigenvalue, unsigned int index);

  /**
   * @brief Get the eigenvalues in a diagonal matrix form
   */
  Eigen::DiagonalMatrix<double, 6, 6> get_diagonal_eigenvalues() const;

  /**
   * @brief Compute the force (task space) or torque (joint space) command based on the input state
   * of the system as the error between the desired state and the real state.
   * @param desired_state the desired state to reach
   * @param feedback_state the real state of the system as read from feedback loop
   * @return the output command at the input state
   */
  StateRepresentation::CartesianState compute_command(const StateRepresentation::CartesianState& desired_state,
                                                      const StateRepresentation::CartesianState& feedback_state);

  /**
   * @brief Return a list of all the parameters of the controller
   * @return the list of parameters
   */
  std::list<std::shared_ptr<StateRepresentation::ParameterInterface>> get_parameters() const;
};

inline Eigen::Matrix<double, 6, 1> Dissipative::get_damping_eigenvalues() const {
  return this->damping_eigenvalues_->get_value();
}

inline double Dissipative::get_damping_eigenvalue(unsigned int index) const {
  Eigen::Matrix<double, 6, 1> eigenvalues = this->get_damping_eigenvalues();
  return eigenvalues(index);
}

inline void Dissipative::set_damping_eigenvalues(const Eigen::Matrix<double, 6, 1>& damping_eigenvalues) {
  this->damping_eigenvalues_->set_value(damping_eigenvalues);
}

inline void Dissipative::set_damping_eigenvalue(double damping_eigenvalue, unsigned int index) {
  Eigen::Matrix<double, 6, 1> eigenvalues = this->get_damping_eigenvalues();
  eigenvalues(index) = damping_eigenvalue;
  this->set_damping_eigenvalues(eigenvalues);
}

inline Eigen::DiagonalMatrix<double, 6, 6> Dissipative::get_diagonal_eigenvalues() const {
  return this->get_damping_eigenvalues().asDiagonal();
}

inline std::list<std::shared_ptr<StateRepresentation::ParameterInterface>> Dissipative::get_parameters() const {
  std::list<std::shared_ptr<StateRepresentation::ParameterInterface>> param_list;
  param_list.push_back(this->damping_eigenvalues_);
  return param_list;
}
}// namespace impedance
}// namespace controllers
