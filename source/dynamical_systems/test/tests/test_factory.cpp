#include <gtest/gtest.h>

#include "dynamical_systems/DynamicalSystemFactory.hpp"

#include "state_representation/parameters/Parameter.hpp"
#include "state_representation/robot/JointState.hpp"

using namespace state_representation;

TEST(DSFactoryTest, CreateDS) {
  auto cart_ds = dynamical_systems::DynamicalSystemFactory<CartesianState>::create_dynamical_system(
      dynamical_systems::DynamicalSystemFactory<CartesianState>::NONE
  );
  cart_ds->set_base_frame(CartesianState::Identity("A"));
  cart_ds->set_parameter(make_shared_parameter("attractor", CartesianPose::Identity("B", "A")));
  ASSERT_NO_THROW(cart_ds->evaluate(CartesianState::Identity("C", "A")));
  EXPECT_TRUE(cart_ds->evaluate(CartesianState::Identity("C", "A")).is_empty());
  EXPECT_EQ(cart_ds->get_parameters().size(), 0);

  auto joint_ds = dynamical_systems::DynamicalSystemFactory<JointState>::create_dynamical_system(
      dynamical_systems::DynamicalSystemFactory<JointState>::NONE
  );
  joint_ds->set_base_frame(JointState::Zero("robot", 3));
  joint_ds->set_parameter(make_shared_parameter("attractor", JointState::Zero("robot", 3)));
  ASSERT_NO_THROW(joint_ds->evaluate(JointState::Random("robot", 3)));
  EXPECT_TRUE(joint_ds->evaluate(JointState::Random("robot", 3)).is_empty());
  EXPECT_EQ(joint_ds->get_parameters().size(), 0);
}
