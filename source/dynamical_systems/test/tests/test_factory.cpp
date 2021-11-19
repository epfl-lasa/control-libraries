#include <gtest/gtest.h>

#include "dynamical_systems/DynamicalSystemFactory.hpp"
#include "dynamical_systems/exceptions/InvalidParameterException.hpp"

#include "state_representation/parameters/Parameter.hpp"
#include "state_representation/robot/JointState.hpp"

using namespace state_representation;

TEST(DSFactoryTest, CreateDS) {
  std::list<std::shared_ptr<state_representation::ParameterInterface>> param_list;
  param_list.emplace_back(make_shared_parameter("test", 1.0));

  auto cart_ds = dynamical_systems::DynamicalSystemFactory<CartesianState>::create_dynamical_system(
      dynamical_systems::DynamicalSystemFactory<CartesianState>::DYNAMICAL_SYSTEM::NONE
  );
  cart_ds->set_base_frame(CartesianState::Identity("A"));
  ASSERT_NO_THROW(auto res = cart_ds->evaluate(CartesianState::Identity("C", "A")));
  EXPECT_TRUE(cart_ds->evaluate(CartesianState::Identity("C", "A")).is_empty());
  EXPECT_EQ(cart_ds->get_parameters().size(), 0);
  EXPECT_THROW(cart_ds->set_parameters(param_list), dynamical_systems::exceptions::InvalidParameterException);

  auto joint_ds = dynamical_systems::DynamicalSystemFactory<JointState>::create_dynamical_system(
      dynamical_systems::DynamicalSystemFactory<JointState>::DYNAMICAL_SYSTEM::NONE
  );
  joint_ds->set_base_frame(JointState::Zero("robot", 3));
  ASSERT_NO_THROW(auto res = joint_ds->evaluate(JointState::Random("robot", 3)));
  EXPECT_TRUE(joint_ds->evaluate(JointState::Random("robot", 3)).is_empty());
  EXPECT_EQ(joint_ds->get_parameters().size(), 0);
  EXPECT_THROW(joint_ds->set_parameters(param_list), dynamical_systems::exceptions::InvalidParameterException);
}
