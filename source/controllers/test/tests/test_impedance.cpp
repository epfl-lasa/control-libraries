#include "controllers/impedance/Impedance.hpp"
#include "state_representation/robot/JointState.hpp"
#include "state_representation/robot/JointTorques.hpp"
#include "state_representation/space/cartesian/CartesianState.hpp"
#include "state_representation/space/cartesian/CartesianWrench.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <gtest/gtest.h>

using namespace controllers::impedance;
using namespace state_representation;

TEST(ImpedanceControllerTest, TestCopyConstructor) {
  Impedance<CartesianState> impedance_controller(Eigen::MatrixXd::Random(6, 6),
                                                 Eigen::MatrixXd::Random(6, 6),
                                                 Eigen::MatrixXd::Random(6, 6));
  Impedance<CartesianState> copy(impedance_controller);
  double tolerance = 1e-4;
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(impedance_controller.get_damping().col(i).norm(), copy.get_damping().col(i).norm(), tolerance);
    EXPECT_NEAR(impedance_controller.get_stiffness().col(i).norm(), copy.get_stiffness().col(i).norm(), tolerance);
    EXPECT_NEAR(impedance_controller.get_inertia().col(i).norm(), copy.get_inertia().col(i).norm(), tolerance);
  }
}

TEST(ImpedanceControllerTest, TestAssignmentOperator) {
  Impedance<CartesianState> impedance_controller(Eigen::MatrixXd::Random(6, 6),
                                                 Eigen::MatrixXd::Random(6, 6),
                                                 Eigen::MatrixXd::Random(6, 6));
  Impedance<CartesianState> copy = impedance_controller;
  double tolerance = 1e-4;
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(impedance_controller.get_damping().col(i).norm(), copy.get_damping().col(i).norm(), tolerance);
    EXPECT_NEAR(impedance_controller.get_stiffness().col(i).norm(), copy.get_stiffness().col(i).norm(), tolerance);
    EXPECT_NEAR(impedance_controller.get_inertia().col(i).norm(), copy.get_inertia().col(i).norm(), tolerance);
  }
}

TEST(ImpedanceControllerTest, TestCartesianImpedance) {
  Impedance<CartesianState> impedance_controller(Eigen::MatrixXd::Identity(6, 6),
                                                 Eigen::MatrixXd::Identity(6, 6),
                                                 Eigen::MatrixXd::Identity(6, 6));
  // set up a desired state6d
  CartesianState desired_state("test");
  desired_state.set_linear_velocity(Eigen::Vector3d(1, 0, 0));
  // set up a real state
  CartesianState feedback_state("test");
  feedback_state.set_orientation(Eigen::Quaterniond(0, 0, 1, 1));
  feedback_state.set_linear_velocity(Eigen::Vector3d(0.5, 0, 0));
  // check command
  CartesianWrench command = impedance_controller.compute_command(desired_state, feedback_state);
  // expect some non null data
  EXPECT_TRUE(command.data().norm() > 0.);
}

TEST(ImpedanceControllerTest, TestJointImpedance) {
  int nb_joints = 3;
  Impedance<JointState> impedance_controller(Eigen::MatrixXd::Identity(nb_joints, nb_joints),
                                             Eigen::MatrixXd::Identity(nb_joints, nb_joints),
                                             Eigen::MatrixXd::Identity(nb_joints, nb_joints));
  // set up a desired state6d
  JointState desired_state("test", nb_joints);
  desired_state.set_velocities(Eigen::Vector3d(1, 0, 0));
  // set up a real state
  JointState feedback_state("test", nb_joints);
  feedback_state.set_positions(Eigen::Vector3d(0, 0, 1));
  feedback_state.set_velocities(Eigen::Vector3d(0.5, 0, 0));
  // check command
  JointTorques command = impedance_controller.compute_command(desired_state, feedback_state);
  // expect some non null data
  EXPECT_TRUE(command.data().norm() > 0.);
}

TEST(ImpedanceControllerTest, TestCartesianToJointImpedance) {
  Impedance<CartesianState> impedance_controller(Eigen::MatrixXd::Identity(6, 6),
                                                 Eigen::MatrixXd::Identity(6, 6),
                                                 Eigen::MatrixXd::Identity(6, 6));
  // set up a desired state6d
  CartesianState desired_state("test");
  desired_state.set_linear_velocity(Eigen::Vector3d(1, 0, 0));
  // set up a real state
  CartesianState feedback_state("test");
  feedback_state.set_orientation(Eigen::Quaterniond(0, 0, 1, 1));
  feedback_state.set_linear_velocity(Eigen::Vector3d(0.5, 0, 0));
  // set a Jacobian matrix
  Jacobian jac = Jacobian::Random("test_robot", 3, "test");
  // check command
  JointTorques command = impedance_controller.compute_command(desired_state, feedback_state, jac);
  // expect some non null data
  EXPECT_TRUE(command.data().norm() > 0.);
}
