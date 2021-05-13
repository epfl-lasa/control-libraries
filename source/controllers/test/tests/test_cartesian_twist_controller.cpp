#include <gtest/gtest.h>

#include "controllers/impedance/CartesianTwistController.hpp"
#include "state_representation/space/cartesian/CartesianWrench.hpp"
#include "state_representation/space/cartesian/CartesianTwist.hpp"
#include "state_representation/robot/JointTorques.hpp"

using namespace controllers::impedance;
using namespace state_representation;


TEST(CartesianTwistControllerTest, CartesianWrench) {
  CartesianTwistController controller(100, 100, 5, 5);

  auto desired_twist = CartesianTwist::Identity("test");
  auto feedback_twist = CartesianTwist::Identity("test");

  // check command when desired state == feedback state
  CartesianWrench command = controller.compute_command(desired_twist, feedback_twist);
  EXPECT_NEAR(command.data().norm(), 0.0, 1e-5);

  // check command when there is linear velocity error
  desired_twist.set_linear_velocity(Eigen::Vector3d::Random());
  command = controller.compute_command(desired_twist, feedback_twist);
  EXPECT_GT(command.get_force().norm(), 0.0);
  EXPECT_NEAR(command.get_torque().norm(), 0.0, 1e-5);

  // check command when there is angular velocity error
  desired_twist.set_linear_velocity(Eigen::Vector3d::Zero());
  desired_twist.set_angular_velocity(Eigen::Vector3d::Random());
  command = controller.compute_command(desired_twist, feedback_twist);
  // expect some non-null data for force
  EXPECT_NEAR(command.get_force().norm(), 0.0, 1e-5);
  EXPECT_GT(command.get_torque().norm(), 0.0);
}

TEST(CartesianTwistControllerTest, JointTorques) {
  CartesianTwistController controller(100, 100, 5, 5);

  auto desired_twist = CartesianTwist::Random("test");
  auto feedback_twist = CartesianTwist::Identity("test");
  auto jacobian = Jacobian::Random("test_robot", 3, "test");

  JointTorques command = controller.compute_command(desired_twist, feedback_twist, jacobian);
  // expect some non-null data
  EXPECT_GT(command.data().norm(), 0.0);
}

TEST(CartesianTwistControllerTest, ParametersAndSetters) {
  CartesianTwistController controller(1, 2, 3, 4);

  auto params = controller.get_parameters();
  EXPECT_EQ(params.size(), 4);
  for (auto& param_interface : params) {
    auto param = std::dynamic_pointer_cast<Parameter<double>>(param_interface);
    if (param->get_name() == "linear_principle_damping") {
      EXPECT_NEAR(param->get_value(), 1, 1e-5);
      EXPECT_NEAR(controller.get_gains()(0), 1, 1e-5);
      param->set_value(11);
      EXPECT_NEAR(controller.get_gains()(0), 11, 1e-5);
      controller.set_linear_principle_damping(21);
      EXPECT_NEAR(param->get_value(), 21, 1e-5);
    } else if (param->get_name() == "linear_orthogonal_damping") {
      EXPECT_NEAR(param->get_value(), 2, 1e-5);
      EXPECT_NEAR(controller.get_gains()(1), 2, 1e-5);
      param->set_value(12);
      EXPECT_NEAR(controller.get_gains()(1), 12, 1e-5);
      controller.set_linear_orthogonal_damping(22);
      EXPECT_NEAR(param->get_value(), 22, 1e-5);
    } else if (param->get_name() == "angular_stiffness") {
      EXPECT_NEAR(param->get_value(), 3, 1e-5);
      EXPECT_NEAR(controller.get_gains()(2), 3, 1e-5);
      param->set_value(13);
      EXPECT_NEAR(controller.get_gains()(2), 13, 1e-5);
      controller.set_angular_stiffness(23);
      EXPECT_NEAR(param->get_value(), 23, 1e-5);
    } else if (param->get_name() == "angular_damping") {
      EXPECT_NEAR(param->get_value(), 4, 1e-5);
      EXPECT_NEAR(controller.get_gains()(3), 4, 1e-5);
      param->set_value(14);
      EXPECT_NEAR(controller.get_gains()(3), 14, 1e-5);
      controller.set_angular_damping(24);
      EXPECT_NEAR(param->get_value(), 24, 1e-5);
    }
  }
}