#pragma once

#include "controllers/Controller.hpp"
#include "controllers/impedance/Dissipative.hpp"
#include "controllers/impedance/VelocityImpedance.hpp"
#include "state_representation/parameters/Parameter.hpp"

namespace controllers::impedance {
/**
 * @class CartesianTwistController
 * @brief A concrete controller class specifically for controlling
 * 6 degree of freedom Cartesian twist with a combination of impedance
 * controllers. The linear velocity is controlled with the Dissipative
 * controller class, while the angular velocity is controlled
 * with the VelocityImpedance class.
 */
class CartesianTwistController : public Controller<state_representation::CartesianState> {
private:
  std::shared_ptr<state_representation::Parameter<double>>
      linear_principle_damping_; ///< damping along principle eigenvector of linear velocity error
  std::shared_ptr<state_representation::Parameter<double>>
      linear_orthogonal_damping_; ///< damping along secondary eigenvectors of linear velocity error
  std::shared_ptr<state_representation::Parameter<double>> angular_stiffness_; ///< stiffness of angular displacement
  std::shared_ptr<state_representation::Parameter<double>> angular_damping_; ///< damping of angular velocity error

  Dissipative<state_representation::CartesianState> dissipative_ctrl_;
  VelocityImpedance<state_representation::CartesianState> velocity_impedance_ctrl_;
public:
  /**
   * @brief Constructor for the CartesianTwistController
   * @param linear_principle_damping damping along principle eigenvector of linear velocity error
   * @param linear_orthogonal_damping damping along secondary eigenvectors of linear velocity error
   * @param angular_stiffness stiffness of angular displacement
   * @param angular_damping damping of angular velocity error
   */
  CartesianTwistController(double linear_principle_damping,
                           double linear_orthogonal_damping,
                           double angular_stiffness,
                           double angular_damping);

  /**
   * @brief Constructor for the CartesianTwistController with vector form gains
   * @param gains Eigen 4d vector of gains in the order linear_principle_damping,
   * linear_orthogonal_damping, angular_stiffness, angular_damping
   */
  explicit CartesianTwistController(const Eigen::Vector4d& gains);

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

  /**
   * @brief Setter of the controller gains
   * @param the new gains as a vector of linear principle damping,
   * linear orthogonal, angular stiffness and angular damping values
   */
  void set_gains(const Eigen::Vector4d& gains);

  /**
   * @brief Getter of the controller gains
   * @return the new gains as a vector of linear principle damping,
   * linear orthogonal, angular stiffness and angular damping values
   */
  Eigen::Vector4d get_gains() const;

  /**
   * @brief Return a list of all the parameters of the controller
   * @return the list of parameters
   */
  std::list<std::shared_ptr<state_representation::ParameterInterface>> get_parameters() const override;

  /**
   * @brief Compute the force (task space) or torque (joint space) command based on the input state 
   * of the system as the error between the desired state and the real state.
   * @param desired_state the desired state to reach
   * @param feedback_state the real state of the system as read from feedback loop
   * @return the output command at the input state
   */
  state_representation::CartesianState compute_command(const state_representation::CartesianState& desired_state,
                                                       const state_representation::CartesianState& feedback_state) override;

  /**
   * @brief Compute the command based on the desired state and a feedback state in a non const fashion
   * To be redefined based on the actual controller implementation.
   * @param desired_state the desired state of the system.
   * @param feedback_state the real state of the system as read from feedback loop
   * @param jacobian the Jacobian matrix of the robot to convert from one state to the other
   * @return the output command at the input state
   */
  state_representation::JointState compute_command(const state_representation::CartesianState& desired_state,
                                                   const state_representation::CartesianState& feedback_state,
                                                   const state_representation::Jacobian& jacobian) override;
};

}// namespace controllers
