#include <gtest/gtest.h>

#include "dynamical_systems/DynamicalSystemFactory.hpp"
#include "state_representation/exceptions/InvalidParameterException.hpp"

#include "state_representation/space/cartesian/CartesianState.hpp"
#include "state_representation/space/joint/JointState.hpp"
#include "state_representation/parameters/Parameter.hpp"

using namespace state_representation;

TEST(DSFactoryTest, CreateDS) {
  std::list<std::shared_ptr<state_representation::ParameterInterface>> param_list;
  param_list.emplace_back(make_shared_parameter("test", 1.0));

  auto cart_ds = dynamical_systems::CartesianDynamicalSystemFactory::create_dynamical_system(
      dynamical_systems::DYNAMICAL_SYSTEM_TYPE::NONE
  );
  cart_ds->set_base_frame(CartesianState::Identity("A"));
  ASSERT_NO_THROW(auto res = cart_ds->evaluate(CartesianState::Identity("C", "A")));
  EXPECT_TRUE(cart_ds->evaluate(CartesianState::Identity("C", "A")).is_empty());
  EXPECT_EQ(cart_ds->get_parameters().size(), 0);
  EXPECT_THROW(cart_ds->set_parameters(param_list), exceptions::InvalidParameterException);

  auto joint_ds = dynamical_systems::JointDynamicalSystemFactory::create_dynamical_system(
      dynamical_systems::DYNAMICAL_SYSTEM_TYPE::NONE
  );
  ASSERT_NO_THROW(auto res = joint_ds->evaluate(JointState::Random("robot", 3)));
  EXPECT_TRUE(joint_ds->evaluate(JointState::Random("robot", 3)).is_empty());
  EXPECT_EQ(joint_ds->get_parameters().size(), 0);
  EXPECT_THROW(joint_ds->set_parameters(param_list), exceptions::InvalidParameterException);
}
