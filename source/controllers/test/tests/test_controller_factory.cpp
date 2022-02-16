#include <gtest/gtest.h>

#include "controllers/ControllerFactory.hpp"
#include "controllers/exceptions/InvalidControllerException.hpp"
#include "controllers/exceptions/NoRobotModelException.hpp"
#include "state_representation/parameters/Parameter.hpp"

using namespace state_representation;
using namespace controllers;

TEST(ControllerFactoryTest, CreateCartesianController) {
  std::shared_ptr<IController<CartesianState>> ctrl;
  EXPECT_EQ(ctrl, nullptr);

  auto command_state = CartesianState::Identity("command");
  auto feedback_state = CartesianState::Identity("feedback");

  ctrl = CartesianControllerFactory::create_controller(CONTROLLER_TYPE::NONE);
  EXPECT_EQ(ctrl, nullptr);
}

TEST(ControllerFactoryTest, CreateJointController) {
  unsigned int dim = 3;
  std::shared_ptr<IController<JointState>> ctrl;
  EXPECT_EQ(ctrl, nullptr);

  auto command_state = JointState::Zero("robot", dim);
  auto feedback_state = JointState::Zero("robot", dim);

  ctrl = JointControllerFactory::create_controller(CONTROLLER_TYPE::NONE, dim);
  EXPECT_EQ(ctrl, nullptr);
}

TEST(ControllerFactoryTest, CreateControllerWithParams) {
  std::list<std::shared_ptr<state_representation::ParameterInterface>> parameters;

  auto ctrl = CartesianControllerFactory::create_controller(CONTROLLER_TYPE::NONE, parameters);
  ASSERT_EQ(ctrl, nullptr);
}

TEST(ControllerFactoryTest, CreateControllerWithRobot) {
  std::string robot_name = "robot";
  std::string urdf_path = std::string(TEST_FIXTURES) + "panda_arm.urdf";
  auto robot = robot_model::Model(robot_name, urdf_path);

  auto command_state = CartesianState::Identity(robot.get_frames().back(), robot.get_base_frame());
  auto feedback_state = CartesianState::Identity(robot.get_frames().back(), robot.get_base_frame());
  auto joint_state = JointState::Zero(robot_name, robot.get_number_of_joints());

  EXPECT_THROW(CartesianControllerFactory::create_controller(CONTROLLER_TYPE::NONE, robot),
               controllers::exceptions::InvalidControllerException);

  EXPECT_THROW(JointControllerFactory::create_controller(CONTROLLER_TYPE::NONE, robot),
               controllers::exceptions::InvalidControllerException);
}

TEST(ControllerFactoryTest, CreateControllerWithRobotAndParams) {
  std::list<std::shared_ptr<state_representation::ParameterInterface>> parameters;
  auto robot = robot_model::Model("robot", std::string(TEST_FIXTURES) + "panda_arm.urdf");

  EXPECT_THROW(JointControllerFactory::create_controller(CONTROLLER_TYPE::NONE, parameters, robot),
               controllers::exceptions::InvalidControllerException);
}