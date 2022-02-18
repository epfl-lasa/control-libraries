#include <gtest/gtest.h>

#include "controllers/ControllerFactory.hpp"
#include "state_representation/space/cartesian/CartesianWrench.hpp"
#include "state_representation/space/cartesian/CartesianTwist.hpp"
#include "state_representation/space/joint/JointTorques.hpp"

using namespace controllers;
using namespace state_representation;

class CompliantTwistControllerTest : public ::testing::Test {
protected:
  CompliantTwistControllerTest() :
      command_twist_(CartesianTwist::Identity("test")), feedback_twist_(CartesianTwist::Identity("test")) {
    controller_ = controllers::CartesianControllerFactory::create_controller(CONTROLLER_TYPE::COMPLIANT_TWIST);
  }

  void set_gains(double lpd, double lod, double as, double ad) {
    controller_->set_parameter_value("linear_principle_damping", lpd);
    controller_->set_parameter_value("linear_orthogonal_damping", lod);
    controller_->set_parameter_value("angular_stiffness", as);
    controller_->set_parameter_value("angular_damping", ad);
  }

  std::shared_ptr<IController<CartesianState>> controller_;

  CartesianTwist command_twist_;
  CartesianTwist feedback_twist_;
};

TEST_F(CompliantTwistControllerTest, CartesianWrench) {
  this->set_gains(100, 100, 5, 5);

  // check command when desired state == feedback state
  CartesianWrench command = controller_->compute_command(command_twist_, feedback_twist_);
  EXPECT_NEAR(command.data().norm(), 0.0, 1e-5);

  // check command when there is linear velocity error
  command_twist_.set_linear_velocity(Eigen::Vector3d::Random());
  command = controller_->compute_command(command_twist_, feedback_twist_);
  EXPECT_GT(command.get_force().norm(), 0.0);
  EXPECT_NEAR(command.get_torque().norm(), 0.0, 1e-5);

  // check command when there is angular velocity error
  command_twist_.set_linear_velocity(Eigen::Vector3d::Zero());
  command_twist_.set_angular_velocity(Eigen::Vector3d::Random());
  // expect some non-null data for force
  command = controller_->compute_command(command_twist_, feedback_twist_);
  EXPECT_NEAR(command.get_force().norm(), 0.0, 1e-5);
  EXPECT_GT(command.get_torque().norm(), 0.0);
}

TEST_F(CompliantTwistControllerTest, GetAndSetParameters) {
  this->set_gains(1, 2, 3, 4);
  auto parameter_map = controller_->get_parameters();
  EXPECT_EQ(parameter_map.size(), 4);
  for (auto& param_pair : parameter_map) {
    auto param = std::dynamic_pointer_cast<Parameter<double>>(param_pair.second);
    if (param->get_name() == "linear_principle_damping") {
      EXPECT_NEAR(param->get_value(), 1, 1e-5);
      EXPECT_NEAR(controller_->get_parameter_value<double>("linear_principle_damping"), 1, 1e-5);
      param->set_value(11);
      EXPECT_NEAR(controller_->get_parameter_value<double>("linear_principle_damping"), 11, 1e-5);
      controller_->set_parameter_value<double>("linear_principle_damping", 21);
      EXPECT_NEAR(param->get_value(), 21, 1e-5);
    } else if (param->get_name() == "linear_orthogonal_damping") {
      EXPECT_NEAR(param->get_value(), 2, 1e-5);
      EXPECT_NEAR(controller_->get_parameter_value<double>("linear_orthogonal_damping"), 2, 1e-5);
      param->set_value(12);
      EXPECT_NEAR(controller_->get_parameter_value<double>("linear_orthogonal_damping"), 12, 1e-5);
      controller_->set_parameter_value<double>("linear_orthogonal_damping", 22);
      EXPECT_NEAR(param->get_value(), 22, 1e-5);
    } else if (param->get_name() == "angular_stiffness") {
      EXPECT_NEAR(param->get_value(), 3, 1e-5);
      EXPECT_NEAR(controller_->get_parameter_value<double>("angular_stiffness"), 3, 1e-5);
      param->set_value(13);
      EXPECT_NEAR(controller_->get_parameter_value<double>("angular_stiffness"), 13, 1e-5);
      controller_->set_parameter_value<double>("angular_stiffness", 23);
      EXPECT_NEAR(param->get_value(), 23, 1e-5);
    } else if (param->get_name() == "angular_damping") {
      EXPECT_NEAR(param->get_value(), 4, 1e-5);
      EXPECT_NEAR(controller_->get_parameter_value<double>("angular_damping"), 4, 1e-5);
      param->set_value(14);
      EXPECT_NEAR(controller_->get_parameter_value<double>("angular_damping"), 14, 1e-5);
      controller_->set_parameter_value<double>("angular_damping", 24);
      EXPECT_NEAR(param->get_value(), 24, 1e-5);
    }
  }
}