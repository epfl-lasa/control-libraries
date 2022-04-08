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

TEST(ImpedanceControllerTest, TestCartesianImpedanceLimits) {
  auto controller = CartesianControllerFactory::create_controller(CONTROLLER_TYPE::IMPEDANCE);

  auto desired_state = CartesianState::Identity("test");
  auto feedback_state = desired_state;

  Eigen::VectorXd twist(6);
  twist << 1.0, -2.0, 3.0, -4.0, 5.0, -6.0;
  feedback_state.set_twist(twist);

  // with default unit gains, expect some equivalent response data
  CartesianWrench command = controller->compute_command(desired_state, feedback_state);
  for (int index = 0; index < 6; ++index) {
    EXPECT_NEAR(abs(command.data()(index)), abs(twist(index)), 1e-6);
  }

  // set a scalar force limit
  double limit = 1.0;
  EXPECT_NO_THROW(controller->set_parameter_value("force_limit", limit));
  // expect all degrees of freedom to have the same force limit
  command = controller->compute_command(desired_state, feedback_state);
  for (int index = 0; index < 6; ++index) {
    EXPECT_LE(abs(command.data()(index)), limit);
  }

  // set a force limit on each degree of freedom
  std::vector<double> limits = {0.5, 1.0, 1.5, 2.0, 3.5, 4.0};
  EXPECT_NO_THROW(controller->set_parameter_value("force_limit", limits));

  // expect all degrees of freedom to respect the individual force limits
  command = controller->compute_command(desired_state, feedback_state);
  for (int index = 0; index < 6; ++index) {
    EXPECT_LE(abs(command.data()(index)), limits.at(index));
  }

  // ensure the limit must match the degrees of freedom
  EXPECT_THROW(controller->set_parameter_value("force_limit", std::vector<double>({1.0, 2.0})),
               state_representation::exceptions::IncompatibleSizeException);
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

TEST(ImpedanceControllerTest, TestJointImpedanceLimits) {
  int nb_joints = 3;
  auto controller = JointControllerFactory::create_controller(CONTROLLER_TYPE::IMPEDANCE, nb_joints);

  auto desired_state = JointState::Zero("test", nb_joints);
  auto feedback_state = desired_state;

  Eigen::VectorXd positions(3);
  positions << 1.0, -2.0, 3.0;
  feedback_state.set_positions(positions);

  // with default unit gains, expect some equivalent response data
  JointTorques command = controller->compute_command(desired_state, feedback_state);
  for (int index = 0; index < nb_joints; ++index) {
    EXPECT_NEAR(abs(command.data()(index)), abs(positions(index)), 1e-6);
  }

  // set a scalar force limit
  double limit = 1.0;
  EXPECT_NO_THROW(controller->set_parameter_value("force_limit", limit));
  // expect all degrees of freedom to have the same force limit
  command = controller->compute_command(desired_state, feedback_state);
  for (int index = 0; index < nb_joints; ++index) {
    EXPECT_LE(abs(command.data()(index)), limit);
  }

  // set a force limit on each degree of freedom
  std::vector<double> limits = {0.5, 1.0, 1.5};
  EXPECT_NO_THROW(controller->set_parameter_value("force_limit", limits));

  // expect all degrees of freedom to respect the individual force limits
  command = controller->compute_command(desired_state, feedback_state);
  for (int index = 0; index < nb_joints; ++index) {
    EXPECT_LE(abs(command.data()(index)), limits.at(index));
  }

  // ensure the limit must match the degrees of freedom
  EXPECT_THROW(controller->set_parameter_value("force_limit", std::vector<double>({1.0, 2.0})),
               state_representation::exceptions::IncompatibleSizeException);
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
