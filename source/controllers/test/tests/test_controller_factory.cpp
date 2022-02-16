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

  ctrl = CartesianControllerFactory::create_controller(CONTROLLER_TYPE::IMPEDANCE);
  ASSERT_NE(ctrl, nullptr);
  {
    auto parameters = ctrl->get_parameters();
    EXPECT_EQ(parameters.count("damping"), 1);
    EXPECT_EQ(parameters.count("stiffness"), 1);
    EXPECT_EQ(parameters.count("inertia"), 1);
  }
  EXPECT_EQ(ctrl->get_parameter_value<Eigen::MatrixXd>("damping").size(), 6 * 6);
  EXPECT_EQ(ctrl->get_parameter_value<Eigen::MatrixXd>("stiffness").size(), 6 * 6);
  EXPECT_EQ(ctrl->get_parameter_value<Eigen::MatrixXd>("inertia").size(), 6 * 6);
  EXPECT_NO_THROW(ctrl->compute_command(command_state, feedback_state));

  ctrl = CartesianControllerFactory::create_controller(CONTROLLER_TYPE::VELOCITY_IMPEDANCE);
  ASSERT_NE(ctrl, nullptr);
  {
    auto parameters = ctrl->get_parameters();
    EXPECT_EQ(parameters.count("damping"), 1);
    EXPECT_EQ(parameters.count("stiffness"), 1);
  }
  EXPECT_EQ(ctrl->get_parameter_value<Eigen::MatrixXd>("damping").size(), 6 * 6);
  EXPECT_EQ(ctrl->get_parameter_value<Eigen::MatrixXd>("stiffness").size(), 6 * 6);
  EXPECT_NO_THROW(ctrl->compute_command(command_state, feedback_state));

  ctrl = CartesianControllerFactory::create_controller(CONTROLLER_TYPE::DISSIPATIVE);
  ASSERT_NE(ctrl, nullptr);
  {
    auto parameters = ctrl->get_parameters();
    EXPECT_EQ(parameters.count("stiffness"), 1);
  }
  EXPECT_EQ(ctrl->get_parameter_value<Eigen::MatrixXd>("stiffness").size(), 6 * 6);
  EXPECT_NO_THROW(ctrl->compute_command(command_state, feedback_state));

  ctrl = CartesianControllerFactory::create_controller(CONTROLLER_TYPE::DISSIPATIVE_LINEAR);
  ASSERT_NE(ctrl, nullptr);
  {
    auto parameters = ctrl->get_parameters();
    EXPECT_EQ(parameters.count("stiffness"), 1);
  }
  EXPECT_EQ(ctrl->get_parameter_value<Eigen::MatrixXd>("stiffness").size(), 6 * 6);
  EXPECT_NO_THROW(ctrl->compute_command(command_state, feedback_state));

  ctrl = CartesianControllerFactory::create_controller(CONTROLLER_TYPE::DISSIPATIVE_ANGULAR);
  {
    auto parameters = ctrl->get_parameters();
    EXPECT_EQ(parameters.count("stiffness"), 1);
  }
  EXPECT_EQ(ctrl->get_parameter_value<Eigen::MatrixXd>("stiffness").size(), 6 * 6);
  EXPECT_NO_THROW(ctrl->compute_command(command_state, feedback_state));

  ctrl = CartesianControllerFactory::create_controller(CONTROLLER_TYPE::DISSIPATIVE_DECOUPLED);
  ASSERT_NE(ctrl, nullptr);
  {
    auto parameters = ctrl->get_parameters();
    EXPECT_EQ(parameters.count("stiffness"), 1);
  }
  EXPECT_EQ(ctrl->get_parameter_value<Eigen::MatrixXd>("stiffness").size(), 6 * 6);
  EXPECT_NO_THROW(ctrl->compute_command(command_state, feedback_state));

  ctrl = CartesianControllerFactory::create_controller(CONTROLLER_TYPE::COMPLIANT_TWIST);
  ASSERT_NE(ctrl, nullptr);
  {
    auto parameters = ctrl->get_parameters();
    EXPECT_EQ(parameters.count("linear_principle_damping"), 1);
    EXPECT_EQ(parameters.count("linear_orthogonal_damping"), 1);
    EXPECT_EQ(parameters.count("angular_stiffness"), 1);
    EXPECT_EQ(parameters.count("angular_damping"), 1);
  }
  EXPECT_NO_THROW(ctrl->compute_command(command_state, feedback_state));

  EXPECT_THROW(ctrl->compute_command(command_state, feedback_state, JointState::Zero("robot", 3)),
               controllers::exceptions::NoRobotModelException);
}

TEST(ControllerFactoryTest, CreateJointController) {
  unsigned int dim = 3;
  std::shared_ptr<IController<JointState>> ctrl;
  EXPECT_EQ(ctrl, nullptr);

  auto command_state = JointState::Zero("robot", dim);
  auto feedback_state = JointState::Zero("robot", dim);

  ctrl = JointControllerFactory::create_controller(CONTROLLER_TYPE::NONE, dim);
  EXPECT_EQ(ctrl, nullptr);

  ctrl = JointControllerFactory::create_controller(CONTROLLER_TYPE::IMPEDANCE, dim);
  ASSERT_NE(ctrl, nullptr);
  {
    auto parameters = ctrl->get_parameters();
    EXPECT_EQ(parameters.count("damping"), 1);
    EXPECT_EQ(parameters.count("stiffness"), 1);
    EXPECT_EQ(parameters.count("inertia"), 1);
  }
  EXPECT_EQ(ctrl->get_parameter_value<Eigen::MatrixXd>("damping").size(), dim * dim);
  EXPECT_EQ(ctrl->get_parameter_value<Eigen::MatrixXd>("stiffness").size(), dim * dim);
  EXPECT_EQ(ctrl->get_parameter_value<Eigen::MatrixXd>("inertia").size(), dim * dim);
  EXPECT_NO_THROW(ctrl->compute_command(command_state, feedback_state));

  ctrl = JointControllerFactory::create_controller(CONTROLLER_TYPE::VELOCITY_IMPEDANCE, dim);
  ASSERT_NE(ctrl, nullptr);
  {
    auto parameters = ctrl->get_parameters();
    EXPECT_EQ(parameters.count("damping"), 1);
    EXPECT_EQ(parameters.count("stiffness"), 1);
  }
  EXPECT_EQ(ctrl->get_parameter_value<Eigen::MatrixXd>("damping").size(), dim * dim);
  EXPECT_EQ(ctrl->get_parameter_value<Eigen::MatrixXd>("stiffness").size(), dim * dim);
  EXPECT_NO_THROW(ctrl->compute_command(command_state, feedback_state));

  ctrl = JointControllerFactory::create_controller(CONTROLLER_TYPE::DISSIPATIVE, dim);
  ASSERT_NE(ctrl, nullptr);
  {
    auto parameters = ctrl->get_parameters();
    EXPECT_EQ(parameters.count("stiffness"), 1);
  }
  EXPECT_EQ(ctrl->get_parameter_value<Eigen::MatrixXd>("stiffness").size(), dim * dim);
  EXPECT_NO_THROW(ctrl->compute_command(command_state, feedback_state));

  EXPECT_THROW(JointControllerFactory::create_controller(CONTROLLER_TYPE::DISSIPATIVE_LINEAR),
               controllers::exceptions::InvalidControllerException);
  EXPECT_THROW(JointControllerFactory::create_controller(CONTROLLER_TYPE::DISSIPATIVE_ANGULAR),
               controllers::exceptions::InvalidControllerException);
  EXPECT_THROW(JointControllerFactory::create_controller(CONTROLLER_TYPE::DISSIPATIVE_DECOUPLED),
               controllers::exceptions::InvalidControllerException);
  EXPECT_THROW(JointControllerFactory::create_controller(CONTROLLER_TYPE::COMPLIANT_TWIST),
               controllers::exceptions::InvalidControllerException);
}

TEST(ControllerFactoryTest, CreateControllerWithParams) {
  std::list<std::shared_ptr<state_representation::ParameterInterface>> parameters;
  parameters.emplace_back(make_shared_parameter("damping", 0.0));
  parameters.emplace_back(make_shared_parameter("stiffness", 5.0));
  parameters.emplace_back(make_shared_parameter("inertia", 10.0));

  auto ctrl = CartesianControllerFactory::create_controller(CONTROLLER_TYPE::IMPEDANCE, parameters);
  ASSERT_NE(ctrl, nullptr);
  EXPECT_EQ(ctrl->get_parameter_value<Eigen::MatrixXd>("damping").size(), 6 * 6);
  EXPECT_EQ(ctrl->get_parameter_value<Eigen::MatrixXd>("stiffness").size(), 6 * 6);
  EXPECT_EQ(ctrl->get_parameter_value<Eigen::MatrixXd>("inertia").size(), 6 * 6);

  EXPECT_EQ(ctrl->get_parameter_value<Eigen::MatrixXd>("damping").sum(), 0.0 * 6);
  EXPECT_EQ(ctrl->get_parameter_value<Eigen::MatrixXd>("stiffness").sum(), 5.0 * 6);
  EXPECT_EQ(ctrl->get_parameter_value<Eigen::MatrixXd>("inertia").sum(), 10.0 * 6);
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

  auto cart_ctrl = CartesianControllerFactory::create_controller(CONTROLLER_TYPE::IMPEDANCE, robot);
  ASSERT_NE(cart_ctrl, nullptr);
  EXPECT_NO_THROW(cart_ctrl->compute_command(command_state, feedback_state));
  EXPECT_NO_THROW(cart_ctrl->compute_command(command_state, feedback_state, joint_state));
  EXPECT_NO_THROW(cart_ctrl->compute_command(command_state, feedback_state, robot.compute_jacobian(joint_state)));

  auto joint_ctrl = JointControllerFactory::create_controller(CONTROLLER_TYPE::IMPEDANCE, robot);
  ASSERT_NE(joint_ctrl, nullptr);
  EXPECT_NO_THROW(joint_ctrl->compute_command(joint_state, joint_state));
  EXPECT_EQ(joint_ctrl->get_parameter_value<Eigen::MatrixXd>("stiffness").size(),
            robot.get_number_of_joints() * robot.get_number_of_joints());
}

TEST(ControllerFactoryTest, CreateControllerWithRobotAndParams) {
  std::list<std::shared_ptr<state_representation::ParameterInterface>> parameters;
  parameters.emplace_back(make_shared_parameter("stiffness", 5.0));
  auto robot = robot_model::Model("robot", std::string(TEST_FIXTURES) + "panda_arm.urdf");

  EXPECT_THROW(JointControllerFactory::create_controller(CONTROLLER_TYPE::NONE, parameters, robot),
               controllers::exceptions::InvalidControllerException);

  auto ctrl = JointControllerFactory::create_controller(CONTROLLER_TYPE::DISSIPATIVE, parameters, robot);
  ASSERT_NE(ctrl, nullptr);
  EXPECT_EQ(ctrl->get_parameter_value<Eigen::MatrixXd>("stiffness").size(),
            robot.get_number_of_joints() * robot.get_number_of_joints());

  EXPECT_EQ(ctrl->get_parameter_value<Eigen::MatrixXd>("stiffness").sum(), 5.0 * robot.get_number_of_joints());
}