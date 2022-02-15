#pragma once

#include "controllers/impedance/NewImpedance.hpp"
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
  LINEAR, ANGULAR, DECOUPLED_TWIST, FULL
};

/**
 * @class NewDissipative
 * @brief Definition of a dissipative impedance controller (PassiveDS) in task space
 */
template<class S>
class NewDissipative : public NewImpedance<S> {

public:

  /**
   * @brief Base constructor.
   * @param computational_space The computational space type
   * @param dimensions The number of dimensions
   */
  explicit NewDissipative(const ComputationalSpaceType& computational_space, unsigned int dimensions = 6);

  /**
   * @brief Constructor from an initial parameter list.
   * @param parameters A parameter list containing the initial parameters
   * @param computational_space The computational space type
   * @param dimensions The number of dimensions
   */
  explicit NewDissipative(
      const std::list<std::shared_ptr<state_representation::ParameterInterface>>& parameters,
      const ComputationalSpaceType& computational_space, unsigned int dimensions = 6
  );

  /**
   * @brief Compute the force (task space) or torque (joint space) command based on the input state
   * of the system as the error between the desired state and the real state.
   * @param command_state the desired state to reach
   * @param feedback_state the real state of the system as read from feedback loop
   * @return the output command at the input state
   */
  S compute_command(const S& command_state, const S& feedback_state) override;

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

protected:

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

  std::shared_ptr<state_representation::Parameter<Eigen::VectorXd>>
      damping_eigenvalues_; ///< coefficient of eigenvalues used in the damping matrix computation

  const ComputationalSpaceType computational_space_; ///< the space in which to compute the command vector

  Eigen::MatrixXd basis_; ///< basis matrix used to compute the damping matrix

};

template<class S>
NewDissipative<S>::NewDissipative(const ComputationalSpaceType& computational_space, unsigned int dimensions) :
    NewImpedance<S>(dimensions),
    damping_eigenvalues_(
        state_representation::make_shared_parameter<Eigen::VectorXd>(
            "damping_eigenvalues", Eigen::ArrayXd::Ones(dimensions))),
    computational_space_(computational_space),
    basis_(Eigen::MatrixXd::Random(dimensions, dimensions)) {
  this->parameters_.erase("damping");
  this->damping_->set_value(Eigen::MatrixXd::Zero(dimensions, dimensions));
  this->parameters_.erase("inertia");
  this->inertia_->set_value(Eigen::MatrixXd::Zero(dimensions, dimensions));
}

template<class S>
NewDissipative<S>::NewDissipative(
    const std::list<std::shared_ptr<state_representation::ParameterInterface>>& parameters,
    const ComputationalSpaceType& computational_space, unsigned int dimensions
) :
    NewDissipative<S>(computational_space, dimensions) {
  this->set_parameters(parameters);
}

template<class S>
inline Eigen::VectorXd NewDissipative<S>::get_damping_eigenvalues() const {
  return this->damping_eigenvalues_->get_value();
}

template<class S>
inline double NewDissipative<S>::get_damping_eigenvalue(unsigned int index) const {
  Eigen::VectorXd eigenvalues = this->get_damping_eigenvalues();
  return eigenvalues(index);
}

template<class S>
inline void NewDissipative<S>::set_damping_eigenvalues(const Eigen::VectorXd& damping_eigenvalues) {
  this->damping_eigenvalues_->set_value(damping_eigenvalues);
}

template<class S>
inline void NewDissipative<S>::set_damping_eigenvalue(double damping_eigenvalue, unsigned int index) {
  Eigen::VectorXd eigenvalues = this->get_damping_eigenvalues();
  eigenvalues(index) = damping_eigenvalue;
  this->set_damping_eigenvalues(eigenvalues);
}

template<class S>
inline Eigen::MatrixXd NewDissipative<S>::get_diagonal_eigenvalues() const {
  return this->get_damping_eigenvalues().asDiagonal();
}

template<class S>
Eigen::MatrixXd NewDissipative<S>::orthonormalize_basis(
    const Eigen::MatrixXd& basis, const Eigen::VectorXd& main_eigenvector
) {
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
S NewDissipative<S>::compute_command(
    const S& command_state, const S& feedback_state
) {
  // compute the damping matrix out of the command_state twist
  this->compute_damping(command_state);
  // apply the impedance control law
  return this->NewImpedance<S>::compute_command(command_state, feedback_state);
}

template<class S>
void NewDissipative<S>::compute_damping(const S& desired_velocity) {
  this->basis_ = this->compute_orthonormal_basis(desired_velocity);
  this->damping_->set_value(this->basis_ * this->get_diagonal_eigenvalues() * this->basis_.transpose());
}

}// namespace controllers
