#include "controllers/impedance/VelocityImpedance.hpp"
#include "state_representation/robot/JointState.hpp"
#include "state_representation/robot/JointPositions.hpp"
#include "state_representation/robot/JointVelocities.hpp"
#include "state_representation/space/cartesian/CartesianState.hpp"
#include "state_representation/space/cartesian/CartesianPose.hpp"
#include "state_representation/space/cartesian/CartesianTwist.hpp"

using namespace state_representation;

namespace controllers::impedance {
template<class S>
VelocityImpedance<S>::VelocityImpedance(const Eigen::MatrixXd& stiffness,
                                        const Eigen::MatrixXd& damping) :
    Impedance<S>(stiffness, damping, Eigen::MatrixXd::Zero(stiffness.rows(), stiffness.cols())) {}

template VelocityImpedance<CartesianState>::VelocityImpedance(const Eigen::MatrixXd&,
                                                              const Eigen::MatrixXd&);
template VelocityImpedance<JointState>::VelocityImpedance(const Eigen::MatrixXd&,
                                                          const Eigen::MatrixXd&);

template<class S>
S VelocityImpedance<S>::compute_command(const S&, const S&) {
  throw exceptions::NotImplementedException(
      "compute_command(desired_state, feedback_state) not implemented for this input class");
}

template<>
CartesianState VelocityImpedance<CartesianState>::compute_command(const CartesianState& desired_state,
                                                                  const CartesianState& feedback_state) {
  using namespace std::chrono_literals;
  // compute the displacement by multiplying the desired twist by the unit time period and add it to the current pose
  CartesianPose desired_pose = 1s * static_cast<CartesianTwist>(desired_state);
  // set this as the new desired_state keeping the rest of the state values
  CartesianState integrated_desired_state = desired_state;
  integrated_desired_state.set_pose(desired_pose.data());
  // only keep velocity feedback
  CartesianTwist feedback_twist(feedback_state);
  // compute the impedance control law normally
  return this->Impedance<CartesianState>::compute_command(integrated_desired_state, feedback_twist);
}

template<>
JointState VelocityImpedance<JointState>::compute_command(const JointState& desired_state,
                                                          const JointState& feedback_state) {
  
  using namespace std::chrono_literals;
  // compute the displacement by multiplying the desired velocities by the unit time period and add it to the current positions
  JointPositions desired_positions = 1s * static_cast<JointVelocities>(desired_state);
  // set this displacement as the positions of the desired_state
  JointState integrated_desired_state = desired_state;
  integrated_desired_state.set_positions(desired_positions.data());
  // only keep velocity feedback
  JointVelocities feedback_velocities(feedback_state);
  // compute the impedance control law normally
  return this->Impedance<JointState>::compute_command(integrated_desired_state, feedback_velocities);
}
}// namespace controllers