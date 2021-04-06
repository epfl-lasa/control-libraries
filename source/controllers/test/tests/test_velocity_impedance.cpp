#include "controllers/impedance/VelocityImpedance.hpp"
#include "state_representation/robot/JointState.hpp"
#include "state_representation/robot/JointTorques.hpp"
#include "state_representation/space/cartesian/CartesianState.hpp"
#include "state_representation/space/cartesian/CartesianWrench.hpp"
#include <eigen3/Eigen/Core>
#include <gtest/gtest.h>

using namespace controllers::impedance;
using namespace state_representation;

TEST(VelocityImpedanceControllerTest, TestCopyConstructor) {
  VelocityImpedance<CartesianState> impedance_controller(Eigen::MatrixXd::Random(6, 6),
                                                         Eigen::MatrixXd::Random(6, 6));
  VelocityImpedance<CartesianState> copy(impedance_controller);
  EXPECT_TRUE(impedance_controller.get_damping().isApprox(copy.get_damping()));
  EXPECT_TRUE(impedance_controller.get_stiffness().isApprox(copy.get_stiffness()));
  EXPECT_TRUE(impedance_controller.get_inertia().isApprox(copy.get_inertia()));
}

TEST(VelocityImpedanceControllerTest, TestAssignmentOperator) {
  VelocityImpedance<CartesianState> impedance_controller(Eigen::MatrixXd::Random(6, 6),
                                                         Eigen::MatrixXd::Random(6, 6));
  VelocityImpedance<CartesianState> copy = impedance_controller;
  EXPECT_TRUE(impedance_controller.get_damping().isApprox(copy.get_damping()));
  EXPECT_TRUE(impedance_controller.get_stiffness().isApprox(copy.get_stiffness()));
  EXPECT_TRUE(impedance_controller.get_inertia().isApprox(copy.get_inertia()));
}

TEST(VelocityImpedanceControllerTest, TestCartesianImpedance) {
  VelocityImpedance<CartesianState> impedance_controller(Eigen::MatrixXd::Identity(6, 6),
                                                         Eigen::MatrixXd::Identity(6, 6));
  // set up a desired state6d
  CartesianState desired_state("test");
  desired_state.set_linear_velocity(Eigen::Vector3d(1, 0, 0));
  // set up a real state
  CartesianState feedback_state("test");
  feedback_state.set_linear_velocity(Eigen::Vector3d(0.5, 0, 0));
  // check command
  CartesianWrench command = impedance_controller.compute_command(desired_state, feedback_state);
  // expect some non null data
  EXPECT_GT(command.data().norm(), 0.);
  // as opposed to the normal impedance controller, even if the two velocities are the same
  // we still expect a command as the desired pose is non null
  feedback_state.set_linear_velocity(desired_state.get_linear_velocity());
  command = impedance_controller.compute_command(desired_state, feedback_state);
  EXPECT_GT(command.data().norm(),0.);
}

TEST(VelocityImpedanceControllerTest, TestJointImpedance) {
  int nb_joints = 3;
  VelocityImpedance<JointState> impedance_controller(Eigen::MatrixXd::Identity(nb_joints, nb_joints),
                                                     Eigen::MatrixXd::Identity(nb_joints, nb_joints));
  // set up a desired state6d
  JointState desired_state("test", nb_joints);
  desired_state.set_velocities(Eigen::Vector3d(1, 0, 0));
  // set up a real state
  JointState feedback_state("test", nb_joints);
  feedback_state.set_velocities(Eigen::Vector3d(0.5, 0, 0));
  // check command
  JointTorques command = impedance_controller.compute_command(desired_state, feedback_state);
  // expect some non null data
  EXPECT_GT(command.data().norm(), 0.);
  // as opposed to the normal impedance controller, even if the two velocities are the same
  // we still expect a command as the desired positions are non null
  feedback_state.set_velocities(desired_state.get_velocities());
  command = impedance_controller.compute_command(desired_state, feedback_state);
  EXPECT_GT(command.data().norm(), 0.);
}

TEST(VelocityImpedanceControllerTest, TestCartesianToJointImpedance) {
  VelocityImpedance<CartesianState> impedance_controller(Eigen::MatrixXd::Identity(6, 6),
                                                         Eigen::MatrixXd::Identity(6, 6));
  // set up a desired state6d
  CartesianState desired_state("test");
  desired_state.set_linear_velocity(Eigen::Vector3d(1, 0, 0));
  // set up a real state
  CartesianState feedback_state("test");
  feedback_state.set_linear_velocity(Eigen::Vector3d(0.5, 0, 0));
  // set a Jacobian matrix
  Jacobian jacobian = Jacobian::Random("test_robot", 3, "test");
  // check command
  JointTorques command = impedance_controller.compute_command(desired_state, feedback_state, jacobian);
  // expect some non null data
  EXPECT_GT(command.data().norm(), 0.);
  // as opposed to the normal impedance controller, even if the two velocities are the same
  // we still expect a command as the desired pose is non null
  feedback_state.set_linear_velocity(desired_state.get_linear_velocity());
  command = impedance_controller.compute_command(desired_state, feedback_state, jacobian);
  EXPECT_GT(command.data().norm(), 0.);
}
