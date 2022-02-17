#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "controllers/impedance/NewImpedance.hpp"

namespace controllers::impedance {
/**
 * @class VelocityImpedance
 * @brief A velocity impedance is a normal impedance controller
 * that only uses velocity input and integrates it for position error
 * @tparam S the space of the controller (either CartesianState or JointState)
 */
template<class S>
class NewVelocityImpedance : public NewImpedance<S> {
public:

  /**
   * @brief Base constructor.
   * @details This initializes all gain matrices to the identity matrix of the corresponding dimensionality.
   * @param dimensions The number of dimensions associated with the controller
   */
  explicit NewVelocityImpedance(unsigned int dimensions = 6);

  /**
   * @brief Constructor from an initial parameter list
   * @param parameters A parameter list containing initial gain values
   * @param dimensions The number of dimensions associated with the controller
   */
  explicit NewVelocityImpedance(
      const std::list<std::shared_ptr<state_representation::ParameterInterface>>& parameters,
      unsigned int dimensions = 6
  );

  /**
   * @brief Compute the force (task space) or torque (joint space) command based on the input state 
   * of the system as the error between the desired state and the real state.
   * @param desired_state the desired state to reach
   * @param feedback_state the real state of the system as read from feedback loop
   * @return the output command at the input state
   */
  S compute_command(const S& desired_state, const S& feedback_state) override;
};

template<class S>
NewVelocityImpedance<S>::NewVelocityImpedance(unsigned int dimensions) : NewImpedance<S>(dimensions) {
  this->parameters_.erase("inertia");
  this->inertia_->set_value(Eigen::MatrixXd::Zero(dimensions, dimensions));
}

template<class S>
NewVelocityImpedance<S>::NewVelocityImpedance(
    const std::list<std::shared_ptr<state_representation::ParameterInterface>>& parameters, unsigned int dimensions
) :
    NewVelocityImpedance<S>(dimensions) {
  this->set_parameters(parameters);
}

}// namespace controllers
