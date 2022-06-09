#include <gtest/gtest.h>

#include "state_representation/space/SpatialState.hpp"

using namespace state_representation;

TEST(SpatialStateTest, Constructors) {
  SpatialState state0;
  EXPECT_EQ(state0.get_type(), StateType::SPATIAL_STATE);
  EXPECT_EQ(state0.get_name(), "");
  EXPECT_EQ(state0.get_reference_frame(), "world");
  EXPECT_TRUE(state0.is_empty());
  
  SpatialState state1(StateType::JOINT_STATE);
  EXPECT_EQ(state1.get_type(), StateType::JOINT_STATE);
  EXPECT_EQ(state1.get_name(), "");
  EXPECT_EQ(state1.get_reference_frame(), "world");
  EXPECT_TRUE(state1.is_empty());

  SpatialState state2(StateType::CARTESIAN_STATE, "test", "robot", false);
  EXPECT_EQ(state2.get_type(), StateType::CARTESIAN_STATE);
  EXPECT_EQ(state2.get_name(), "test");
  EXPECT_EQ(state2.get_reference_frame(), "robot");
  EXPECT_FALSE(state2.is_empty());

  SpatialState state3(state2);
  EXPECT_EQ(state3.get_type(), StateType::CARTESIAN_STATE);
  EXPECT_EQ(state3.get_name(), "test");
  EXPECT_EQ(state3.get_reference_frame(), "robot");
  EXPECT_FALSE(state3.is_empty());
}

TEST(SpatialStateTest, Compatibility) {
  SpatialState state1(StateType::CARTESIAN_STATE, "test", "robot", true);
  SpatialState state2(StateType::CARTESIAN_STATE, "robot");
  EXPECT_FALSE(state1.is_compatible(state2));
  state2.set_reference_frame("robot");
  EXPECT_FALSE(state1.is_compatible(state2));
  state2.set_name("test");
  EXPECT_TRUE(state1.is_compatible(state2));
}

TEST(SpatialStateTest, Swap) {
  SpatialState state1(StateType::CARTESIAN_STATE, "cartesian");
  SpatialState state2(StateType::JOINT_STATE, "joint", "robot", false);
  swap(state1, state2);
  EXPECT_EQ(state1.get_type(), StateType::JOINT_STATE);
  EXPECT_EQ(state1.get_name(), "joint");
  EXPECT_EQ(state1.get_reference_frame(), "robot");
  EXPECT_FALSE(state1.is_empty());

  EXPECT_EQ(state2.get_type(), StateType::CARTESIAN_STATE);
  EXPECT_EQ(state2.get_name(), "cartesian");
  EXPECT_EQ(state2.get_reference_frame(), "world");
  EXPECT_TRUE(state2.is_empty());
}