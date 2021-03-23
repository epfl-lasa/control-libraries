#pragma once

#include "controllers/impedance/Impedance.hpp"
#include "state_representation/parameters/Parameter.hpp"
#include "state_representation/space/cartesian/CartesianState.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

namespace controllers::impedance {
/**
 * @enum ComputationalSpaceType
 * @brief Selector of the space in which the controller should be computed
 * LINEAR and ANGULAR only compute the command in linear and angular space
 * respectively, TWIST applies it on the full twist vector, and
 * DECOUPLED_TWIST (default) computes the damping matrix for the linear
 * and angular part separately
 */
enum class ComputationalSpaceType {
  LINEAR,
  ANGULAR,
  DECOUPLED_TWIST,
  FULL
};

/**
 * @class Dissipative
 * @brief Definition of a dissipative impedance controller (PassiveDS) in task space
 */
template<class S>
class Dissipative : public Impedance<S> {
private:
  ComputationalSpaceType computational_space_;///< the space in which to compute the command vector
  unsigned int nb_dimensions_;///< the number of dimensions of the input space
  Eigen::MatrixXd basis_;///< basis matrix used to compute the damping matrix
  std::shared_ptr<state_representation::Parameter<Eigen::VectorXd>>
      damping_eigenvalues_;///< coefficient of eigenvalues used in the damping matrix computation

  /**
   * @brief Constructor with specified computational space and number of dimensions. Both parameters are
   * mutually exclusive and therefore initialized independently in other constructors
   */
  explicit Dissipative(const ComputationalSpaceType& computational_space, unsigned int nb_dimensions);

public:
  /**
   * @brief Constructor with specified computational space. Default is DECOUPLED_TWIST
   * @param computational_space the space in which to compute the command in LINEAR, ANGULAR, DECOUPLED_TWIST or FULL.
   * Default is DECOUPLED_TWIST
   */
  explicit Dissipative(const ComputationalSpaceType& computational_space = ComputationalSpaceType::DECOUPLED_TWIST);

  /**
   * @brief Constructor with specified number of dimensions. Automatically use the full damping matrix.
   * @param nb_dimensions the number of dimensions of the damping matrix
   */
  explicit Dissipative(unsigned int nb_dimensions);

  /**
   * @brief Copy constructor
   * @param other the controller to copy
   */
  Dissipative(const Dissipative<S>& other);

  /**
   * @brief Swap the values of the two controllers
   * @tparam U space of the controller
   * @param controller1 controller to be swapped with 2
   * @param controller2 controller to be swapped with 1
   */
  template<class U>
  friend void swap(Dissipative<U>& controller1, Dissipative<U>& controller2);

  /**
   * @param Assignment operator
   * @param other the controller to copy
   * @return reference to the controller with values from other
   */
  Dissipative<S>& operator=(const Dissipative<S>& other);

  /**
   * @brief Getter of the damping eigenvalues parameter
   * @return the eigenvalues as a Vetor6D
   */
  Eigen::VectorXd get_damping_eigenvalues() const;

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
  void set_damping_eigenvalues(const Eigen::VectorXd& damping_eigenvalues);

  /**
   * @brief Setter of the damping eigenvalue corresponding to the specified index
   * @param damping_eigenvalue the new value of the eigenvalue
   * @param index the index of the eigenvalue (number between 0 and 5)
   */
  void set_damping_eigenvalue(double damping_eigenvalue, unsigned int index);

  /**
   * @brief Get the eigenvalues in a diagonal matrix form
   */
  Eigen::MatrixXd get_diagonal_eigenvalues() const;

  /**
   * @brief Orthornormalize the basis matrix given in input wrt the main engenvector
   * @param basis the basis matrix to orthonormalize
   * @param main_eigenvector the main eigenvector used to compute the basis.
   * Other eigenvectors are orthogonal and selected randomly
   * @return the orthonormalized basis
   */
  static Eigen::MatrixXd orthonormalize_basis(const Eigen::MatrixXd& basis, const Eigen::VectorXd& main_eigenvector);

  /**
   * @brief Compute the orthonormal basis based on the desired velocity input
   * @param desired_velocity the desired velocity used as main eigenvector used to compute the basis.
   * Other eigenvectors are orthogonal and selected randomly
   * @return the orthonormalized basis
   */
  Eigen::MatrixXd compute_orthonormal_basis(const S& desired_velocity);

  /**
   * @brief Compute the damping matrix as the orthonormal basis
   * to the direction of the desired velocity
   * @param desired_velocity the velocity from which the direction
   * of motion is extracted to compute the basis
   */
  void compute_damping(const S& desired_velocity);

  /**
   * @brief Compute the force (task space) or torque (joint space) command based on the input state
   * of the system as the error between the desired state and the real state.
   * @param desired_state the desired state to reach
   * @param feedback_state the real state of the system as read from feedback loop
   * @return the output command at the input state
   */
  S compute_command(const S& desired_state, const S& feedback_state) override;

  /**
   * @brief Compute the command based on the desired state and a feedback state
   * To be redefined based on the actual controller implementation.
   * @param desired_state the desired state of the system.
   * @param feedback_state the real state of the system as read from feedback loop
   * @param jacobian the Jacobian matrix of the robot to convert from one state to the other
   * @return the output command at the input state
   */
  state_representation::JointState compute_command(const S& desired_state,
                                                   const S& feedback_state,
                                                   const state_representation::Jacobian& jacobian) override;

  /**
   * @brief Return a list of all the parameters of the controller
   * @return the list of parameters
   */
  std::list<std::shared_ptr<state_representation::ParameterInterface>> get_parameters() const;
};

template<class S>
Dissipative<S>::Dissipative(const ComputationalSpaceType& computational_space, unsigned int nb_dimensions) :
    Impedance<S>(Eigen::MatrixXd::Zero(nb_dimensions, nb_dimensions),
                 Eigen::MatrixXd::Identity(nb_dimensions, nb_dimensions),
                 Eigen::MatrixXd::Zero(nb_dimensions, nb_dimensions)),
    computational_space_(computational_space),
    nb_dimensions_(nb_dimensions),
    basis_(Eigen::MatrixXd::Random(nb_dimensions, nb_dimensions)),
    damping_eigenvalues_(std::make_shared<state_representation::Parameter<Eigen::VectorXd>>("damping_eigenvalues",
                                                                                            Eigen::ArrayXd::Ones(nb_dimensions))) {}

template<class S>
Dissipative<S>::Dissipative(const Dissipative<S>& other):
    Impedance<S>(other),
    computational_space_(other.computational_space_),
    nb_dimensions_(other.nb_dimensions_),
    basis_(other.basis_),
    damping_eigenvalues_(std::make_shared<state_representation::Parameter<Eigen::VectorXd>>("damping_eigenvalues",
                                                                                            other.get_damping_eigenvalues())) {}

template<class U>
inline void swap(Dissipative<U>& controller1, Dissipative<U>& controller2) {
  swap(static_cast<Impedance<U>&>(controller1), static_cast<Impedance<U>&>(controller2));
  std::swap(controller1.computational_space_, controller2.computational_space_);
  std::swap(controller1.nb_dimensions_, controller2.nb_dimensions_);
  std::swap(controller1.basis_, controller2.basis_);
  std::swap(controller1.damping_eigenvalues_, controller2.damping_eigenvalues_);
}

template<class S>
Dissipative<S>& Dissipative<S>::operator=(const Dissipative<S>& other) {
  Dissipative<S> tmp(other);
  swap(*this, tmp);
  return *this;
}

template<class S>
inline Eigen::VectorXd Dissipative<S>::get_damping_eigenvalues() const {
  return this->damping_eigenvalues_->get_value();
}

template<class S>
inline double Dissipative<S>::get_damping_eigenvalue(unsigned int index) const {
  Eigen::VectorXd eigenvalues = this->get_damping_eigenvalues();
  return eigenvalues(index);
}

template<class S>
inline void Dissipative<S>::set_damping_eigenvalues(const Eigen::VectorXd& damping_eigenvalues) {
  this->damping_eigenvalues_->set_value(damping_eigenvalues);
}

template<class S>
inline void Dissipative<S>::set_damping_eigenvalue(double damping_eigenvalue, unsigned int index) {
  Eigen::VectorXd eigenvalues = this->get_damping_eigenvalues();
  eigenvalues(index) = damping_eigenvalue;
  this->set_damping_eigenvalues(eigenvalues);
}

template<class S>
inline Eigen::MatrixXd Dissipative<S>::get_diagonal_eigenvalues() const {
  return this->get_damping_eigenvalues().asDiagonal();
}

template<class S>
inline std::list<std::shared_ptr<state_representation::ParameterInterface>>
Dissipative<S>::get_parameters() const {
  std::list<std::shared_ptr<state_representation::ParameterInterface>> param_list;
  param_list.push_back(this->damping_eigenvalues_);
  return param_list;
}

template<class S>
Eigen::MatrixXd Dissipative<S>::orthonormalize_basis(const Eigen::MatrixXd& basis,
                                                     const Eigen::VectorXd& main_eigenvector) {
  Eigen::MatrixXd orthonormal_basis = basis;
  uint dim = basis.rows();
  orthonormal_basis.col(0) = main_eigenvector.normalized();
  for (uint i = 1; i < dim; i++) {
    for (uint j = 0; j < i; j++) {
      orthonormal_basis.col(i) -= orthonormal_basis.col(j).dot(orthonormal_basis.col(i)) * orthonormal_basis.col(j);
    }
    orthonormal_basis.col(i).normalize();
  }
  return orthonormal_basis;
}

template<class S>
S Dissipative<S>::compute_command(const S& desired_state,
                                  const S& feedback_state) {
  // compute the damping matrix out of the desired_state twist
  this->compute_damping(desired_state);
  // apply the impedance control law
  return this->Impedance<S>::compute_command(desired_state, feedback_state);
}

template<class S>
void Dissipative<S>::compute_damping(const S& desired_velocity) {
  this->basis_ = this->compute_orthonormal_basis(desired_velocity);
  this->set_damping(this->basis_ * this->get_diagonal_eigenvalues() * this->basis_.transpose());
}

template<class S>
state_representation::JointState Dissipative<S>::compute_command(const S& desired_state,
                                                                 const S& feedback_state,
                                                                 const state_representation::Jacobian& jacobian) {
  return this->Impedance<S>::compute_command(desired_state, feedback_state, jacobian);
}
}// namespace controllers
