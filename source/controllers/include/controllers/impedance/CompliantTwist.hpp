#pragma once

#include "controllers/IController.hpp"
#include "controllers/impedance/NewDissipative.hpp"
#include "controllers/impedance/NewVelocityImpedance.hpp"
#include "state_representation/parameters/Parameter.hpp"

namespace controllers::impedance {
/**
 * @class NewCartesianTwistController
 * @brief A concrete controller class specifically for controlling
 * 6 degree of freedom Cartesian twist with a combination of impedance
 * controllers. The linear velocity is controlled with the Dissipative
 * controller class, while the angular velocity is controlled
 * with the VelocityImpedance class.
 */
class CompliantTwist : public IController<state_representation::CartesianState> {

public:

  /**
   * @brief Constructor from an initial parameter list
   * @param parameters A parameter list containing initial gain values
   */
  explicit CompliantTwist(
      const std::list<std::shared_ptr<state_representation::ParameterInterface>>& parameters
  );

  /**
   * @brief Constructor taking gain parameters as arguments
   * @param linear_principle_damping damping along principle eigenvector of linear velocity error
   * @param linear_orthogonal_damping damping along secondary eigenvectors of linear velocity error
   * @param angular_stiffness stiffness of angular displacement
   * @param angular_damping damping of angular velocity error
   */
  CompliantTwist(
      double linear_principle_damping, double linear_orthogonal_damping, double angular_stiffness,
      double angular_damping
  );

  /**
   * @brief Compute the force (task space) or torque (joint space) command based on the input state 
   * of the system as the error between the desired state and the real state.
   * @param desired_state the desired state to reach
   * @param feedback_state the real state of the system as read from feedback loop
   * @return the output command at the input state
   */
  state_representation::CartesianState compute_command(
      const state_representation::CartesianState& desired_state,
      const state_representation::CartesianState& feedback_state
  ) override;

protected:

  /**
   * @brief Validate and set parameters for controller gains.
   * @param parameter A parameter interface pointer
   */
  void validate_and_set_parameter(const std::shared_ptr<state_representation::ParameterInterface>& parameter) override;

  /**
   * @brief Setter of the linear principle damping
   * @param the new principle damping value
   */
  void set_linear_principle_damping(double linear_principle_damping);

  /**
   * @brief Setter of the linear orthogonal damping
   * @param the new orthogonal damping value
   */
  void set_linear_orthogonal_damping(double linear_orthogonal_damping);

  /**
   * @brief Setter of the linear damping values
   * @param linear_principle_damping the new principle damping value
   * @param linear_orthogonal_damping the new orthogonal damping value
   */
  void set_linear_gains(double linear_principle_damping, double linear_orthogonal_damping);

  /**
   * @brief Setter of the angular stiffness
   * @param the new stiffness value
   */
  void set_angular_stiffness(double angular_stiffness);

  /**
   * @brief Setter of the angular damping
   * @param the new damping value
   */
  void set_angular_damping(double angular_damping);

  /**
   * @brief Setter of the angular damping
   * @param angular_stiffness the new angular stiffness value
   * @param angular_damping the new angular damping value
   */
  void set_angular_gains(double angular_stiffness, double angular_damping);

  std::shared_ptr<state_representation::Parameter<double>>
      linear_principle_damping_; ///< damping along principle eigenvector of linear velocity error
  std::shared_ptr<state_representation::Parameter<double>>
      linear_orthogonal_damping_; ///< damping along secondary eigenvectors of linear velocity error
  std::shared_ptr<state_representation::Parameter<double>> angular_stiffness_; ///< stiffness of angular displacement
  std::shared_ptr<state_representation::Parameter<double>> angular_damping_; ///< damping of angular velocity error

  NewDissipative<state_representation::CartesianState> dissipative_ctrl_; ///< controller for linear space
  NewVelocityImpedance<state_representation::CartesianState> velocity_impedance_ctrl_; ///< controller for angular space
};

}// namespace controllers
