#include <gtest/gtest.h>

#include "controllers/ControllerFactory.hpp"
#include "state_representation/space/joint/JointState.hpp"
#include "state_representation/space/joint/JointTorques.hpp"
#include "state_representation/space/cartesian/CartesianState.hpp"
#include "state_representation/space/cartesian/CartesianWrench.hpp"

using namespace controllers;
using namespace state_representation;

TEST(VelocityImpedanceControllerTest, TestCartesianImpedance) {
  auto controller = CartesianControllerFactory::create_controller(CONTROLLER_TYPE::VELOCITY_IMPEDANCE);

  // set up a desired state6d
  CartesianState desired_state("test");
  desired_state.set_linear_velocity(Eigen::Vector3d(1, 0, 0));
  // set up a real state
  CartesianState feedback_state("test");
  feedback_state.set_linear_velocity(Eigen::Vector3d(0.5, 0, 0));
  // check command
  CartesianWrench command = controller->compute_command(desired_state, feedback_state);
  // expect some non null data
  EXPECT_GT(command.data().norm(), 0.);
  // as opposed to the normal impedance controller, even if the two velocities are the same
  // we still expect a command as the desired pose is non null
  feedback_state.set_linear_velocity(desired_state.get_linear_velocity());
  command = controller->compute_command(desired_state, feedback_state);
  EXPECT_GT(command.data().norm(), 0.);
}

TEST(VelocityImpedanceControllerTest, TestJointImpedance) {
  int nb_joints = 3;
  auto controller = JointControllerFactory::create_controller(CONTROLLER_TYPE::VELOCITY_IMPEDANCE, nb_joints);
  // set up a desired state6d
  JointState desired_state("test", nb_joints);
  desired_state.set_velocities(Eigen::Vector3d(1, 0, 0));
  // set up a real state
  JointState feedback_state("test", nb_joints);
  feedback_state.set_velocities(Eigen::Vector3d(0.5, 0, 0));
  // check command
  JointTorques command = controller->compute_command(desired_state, feedback_state);
  // expect some non null data
  EXPECT_GT(command.data().norm(), 0.);
  // as opposed to the normal impedance controller, even if the two velocities are the same
  // we still expect a command as the desired positions are non null
  feedback_state.set_velocities(desired_state.get_velocities());
  command = controller->compute_command(desired_state, feedback_state);
  EXPECT_GT(command.data().norm(), 0.);
}

TEST(VelocityImpedanceControllerTest, TestCartesianToJointImpedance) {
  auto controller = CartesianControllerFactory::create_controller(CONTROLLER_TYPE::VELOCITY_IMPEDANCE);

  // set up a desired state6d
  CartesianState desired_state("test");
  desired_state.set_linear_velocity(Eigen::Vector3d(1, 0, 0));
  // set up a real state
  CartesianState feedback_state("test");
  feedback_state.set_linear_velocity(Eigen::Vector3d(0.5, 0, 0));
  // set a Jacobian matrix
  Jacobian jacobian = Jacobian::Random("test_robot", 3, "test");
  // check command
  JointTorques command = controller->compute_command(desired_state, feedback_state, jacobian);
  // expect some non null data
  EXPECT_GT(command.data().norm(), 0.);
  // as opposed to the normal impedance controller, even if the two velocities are the same
  // we still expect a command as the desired pose is non null
  feedback_state.set_linear_velocity(desired_state.get_linear_velocity());
  command = controller->compute_command(desired_state, feedback_state, jacobian);
  EXPECT_GT(command.data().norm(), 0.);
}
