#include "controllers/impedance/VelocityImpedance.hpp"
#include "state_representation/Robot/JointState.hpp"
#include "state_representation/Robot/JointPositions.hpp"
#include "state_representation/Robot/JointVelocities.hpp"
#include "state_representation/Robot/JointTorques.hpp"
#include "state_representation/Space/Cartesian/CartesianState.hpp"
#include "state_representation/Space/Cartesian/CartesianPose.hpp"
#include "state_representation/Space/Cartesian/CartesianTwist.hpp"
#include "state_representation/Space/Cartesian/CartesianWrench.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <gtest/gtest.h>

using namespace controllers::impedance;
using namespace StateRepresentation;

TEST(TestCopyConstructor, PositiveNos) {
  VelocityImpedance<CartesianState> impedance_controller(Eigen::MatrixXd::Random(6, 6),
                                                         Eigen::MatrixXd::Random(6, 6));
  VelocityImpedance<CartesianState> copy(impedance_controller);
  double tolerance = 1e-4;
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(impedance_controller.get_damping().col(i).norm(), copy.get_damping().col(i).norm(), tolerance);
    EXPECT_NEAR(impedance_controller.get_stiffness().col(i).norm(), copy.get_stiffness().col(i).norm(), tolerance);
    EXPECT_NEAR(impedance_controller.get_inertia().col(i).norm(), copy.get_inertia().col(i).norm(), tolerance);
  }
}

TEST(TestAssignmentOperator, PositiveNos) {
  VelocityImpedance<CartesianState> impedance_controller(Eigen::MatrixXd::Random(6, 6),
                                                         Eigen::MatrixXd::Random(6, 6));
  VelocityImpedance<CartesianState> copy = impedance_controller;
  double tolerance = 1e-4;
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(impedance_controller.get_damping().col(i).norm(), copy.get_damping().col(i).norm(), tolerance);
    EXPECT_NEAR(impedance_controller.get_stiffness().col(i).norm(), copy.get_stiffness().col(i).norm(), tolerance);
    EXPECT_NEAR(impedance_controller.get_inertia().col(i).norm(), copy.get_inertia().col(i).norm(), tolerance);
  }
}

TEST(TestCartesianImpedance, PositiveNos) {
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
  EXPECT_TRUE(command.data().norm() > 0.);
  // as opposed to the normal impedance controller, even if the two velocities are the same
  // we still expect a command as the desired velocity is non null
  feedback_state.set_linear_velocity(desired_state.get_linear_velocity());
  command = impedance_controller.compute_command(desired_state, feedback_state);
  EXPECT_TRUE(command.data().norm() > 0.);
}

TEST(TestJointImpedance, PositiveNos) {
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
  EXPECT_TRUE(command.data().norm() > 0.);
  // as opposed to the normal impedance controller, even if the two velocities are the same
  // we still expect a command as the desired velocity is non null
  feedback_state.set_velocities(desired_state.get_velocities());
  command = impedance_controller.compute_command(desired_state, feedback_state);
  EXPECT_TRUE(command.data().norm() > 0.);
}

TEST(TestCartesianToJointImpedance, PositiveNos) {
  VelocityImpedance<CartesianState> impedance_controller(Eigen::MatrixXd::Identity(6, 6),
                                                         Eigen::MatrixXd::Identity(6, 6));
  // set up a desired state6d
  CartesianState desired_state("test");
  desired_state.set_linear_velocity(Eigen::Vector3d(1, 0, 0));
  // set up a real state
  CartesianState feedback_state("test");
  feedback_state.set_linear_velocity(Eigen::Vector3d(0.5, 0, 0));
  // set a Jacobian matrix
  Jacobian jacobian("test_robot", 3);
  jacobian.set_data(Eigen::MatrixXd::Random(6, 3));
  // check command
  JointTorques command = impedance_controller.compute_command(desired_state, feedback_state, jacobian);
  // expect some non null data
  EXPECT_TRUE(command.data().norm() > 0.);
  // as opposed to the normal impedance controller, even if the two velocities are the same
  // we still expect a command as the desired velocity is non null
  feedback_state.set_linear_velocity(desired_state.get_linear_velocity());
  command = impedance_controller.compute_command(desired_state, feedback_state, jacobian);
  EXPECT_TRUE(command.data().norm() > 0.);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
