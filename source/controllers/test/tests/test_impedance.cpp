#include <gtest/gtest.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "controllers/ControllerFactory.hpp"
#include "state_representation/space/joint/JointState.hpp"
#include "state_representation/space/joint/JointTorques.hpp"
#include "state_representation/space/cartesian/CartesianState.hpp"
#include "state_representation/space/cartesian/CartesianWrench.hpp"


using namespace controllers;
using namespace state_representation;

TEST(ImpedanceControllerTest, TestCartesianImpedance) {
  auto controller = CartesianControllerFactory::create_controller(CONTROLLER_TYPE::IMPEDANCE);

  // set up a desired state6d
  CartesianState desired_state("test");
  desired_state.set_linear_velocity(Eigen::Vector3d(1, 0, 0));
  // set up a real state
  CartesianState feedback_state("test");
  feedback_state.set_orientation(Eigen::Quaterniond(0, 0, 1, 1));
  feedback_state.set_linear_velocity(Eigen::Vector3d(0.5, 0, 0));
  // check command
  CartesianWrench command = controller->compute_command(desired_state, feedback_state);
  // expect some non null data
  EXPECT_TRUE(command.data().norm() > 0.);
}

TEST(ImpedanceControllerTest, TestJointImpedance) {
  int nb_joints = 3;
  auto controller = JointControllerFactory::create_controller(CONTROLLER_TYPE::IMPEDANCE, nb_joints);

  // set up a desired state6d
  JointState desired_state("test", nb_joints);
  desired_state.set_velocities(Eigen::Vector3d(1, 0, 0));
  // set up a real state
  JointState feedback_state("test", nb_joints);
  feedback_state.set_positions(Eigen::Vector3d(0, 0, 1));
  feedback_state.set_velocities(Eigen::Vector3d(0.5, 0, 0));
  // check command
  JointTorques command = controller->compute_command(desired_state, feedback_state);
  // expect some non null data
  EXPECT_TRUE(command.data().norm() > 0.);
}

TEST(ImpedanceControllerTest, TestCartesianToJointImpedance) {
  auto controller = CartesianControllerFactory::create_controller(CONTROLLER_TYPE::IMPEDANCE);

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
  JointTorques command = controller->compute_command(desired_state, feedback_state, jac);
  // expect some non null data
  EXPECT_TRUE(command.data().norm() > 0.);
}
