#pragma once

#include "controllers/impedance/Impedance.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

namespace controllers::impedance {
/**
 * @class VelocityImpedance
 * @brief A velocity impedance is a normal impedance controller
 * that only take a vlocity input and integrate it for position error
 * @tparam S the space of the controller either CartesianState or JointState
 */
template<class S>
class VelocityImpedance : public Impedance<S> {
public:
  /**
   * @brief Constructor initializing all the matrices
   * @param stiffness the stiffness matrix
   * @param damping the damping matrix
   */
  explicit VelocityImpedance(const Eigen::MatrixXd& stiffness, const Eigen::MatrixXd& damping);

  /**
   * @brief Copy constructor
   * @param other the controller to copy
   */
  VelocityImpedance(const VelocityImpedance<S>& other);

  /**
   * @brief Swap the values of the two controllers
   * @tparam U space of the controller
   * @param controller1 controller to be swapped with 2
   * @param controller2 controller to be swapped with 1
   */
  template<class U>
  friend void swap(VelocityImpedance<U>& controller1, VelocityImpedance<U>& controller2);

  /**
   * @param Assignment operator
   * @param other the controller to copy
   * @return reference to the controller with values from other
   */
  VelocityImpedance<S>& operator=(const VelocityImpedance<S>& other);

  /**
   * @brief Compute the force (task space) or torque (joint space) command based on the input state 
   * of the system as the error between the desired state and the real state.
   * @param desired_state the desired state to reach
   * @param feedback_state the real state of the system as read from feedback loop
   * @return the output command at the input state
   */
  S compute_command(const S& desired_state, const S& feedback_state) override;

  /**
   * @brief Compute the command based on the desired state and a feedback state in a non const fashion
   * To be redefined based on the actual controller implementation.
   * @param desired_state the desired state of the system.
   * @param feedback_state the real state of the system as read from feedback loop
   * @param jacobian the Jacobian matrix of the robot to convert from one state to the other
   * @return the output command at the input state
   */
  StateRepresentation::JointState compute_command(const S& desired_state,
                                                  const S& feedback_state,
                                                  const StateRepresentation::Jacobian& jacobian) override;
};

template<class S>
VelocityImpedance<S>::VelocityImpedance(const VelocityImpedance<S>& other):
    Impedance<S>(other) {}

template<class U>
inline void swap(VelocityImpedance<U>& controller1, VelocityImpedance<U>& controller2) {
  swap(static_cast<Impedance<U>&>(controller1), static_cast<Impedance<U>&>(controller2));
}

template<class S>
inline VelocityImpedance<S>& VelocityImpedance<S>::operator=(const VelocityImpedance<S>& other) {
  VelocityImpedance<S> tmp(other);
  swap(*this, tmp);
  return *this;
}

template<class S>
StateRepresentation::JointState VelocityImpedance<S>::compute_command(const S& desired_state,
                                                                      const S& feedback_state,
                                                                      const StateRepresentation::Jacobian& jacobian) {
  return this->Impedance<S>::compute_command(desired_state, feedback_state, jacobian);
}
}// namespace controllers
