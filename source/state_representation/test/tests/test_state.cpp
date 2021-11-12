#include <chrono>
#include <gtest/gtest.h>
#include <thread>

#include "state_representation/State.hpp"

using namespace state_representation;

TEST(StateTest, Constructors) {
  State empty1;
  EXPECT_EQ(empty1.get_type(), StateType::STATE);
  EXPECT_EQ(empty1.get_name(), "");
  EXPECT_TRUE(empty1.is_empty());

  State empty2(StateType::JOINTSTATE);
  EXPECT_EQ(empty2.get_type(), StateType::JOINTSTATE);
  EXPECT_EQ(empty2.get_name(), "");
  EXPECT_TRUE(empty2.is_empty());

  State empty3(StateType::CARTESIANSTATE, "test", true);
  EXPECT_EQ(empty3.get_type(), StateType::CARTESIANSTATE);
  EXPECT_EQ(empty3.get_name(), "test");
  EXPECT_TRUE(empty3.is_empty());
  empty3.set_filled();
  EXPECT_FALSE(empty3.is_empty());

  State state(empty3);
  EXPECT_EQ(state.get_type(), StateType::CARTESIANSTATE);
  EXPECT_EQ(state.get_name(), "test");
  EXPECT_FALSE(state.is_empty());
  state.set_empty();
  EXPECT_TRUE(state.is_empty());
}

TEST(StateTest, Compatibility) {
  State state1;
  state1.set_name("test");
  EXPECT_EQ(state1.get_name(), "test");

  State state2(StateType::STATE, "test", false);
  EXPECT_TRUE(state1.is_compatible(state2));
  state2.set_name("world");
  EXPECT_FALSE(state1.is_compatible(state2));

  state2.initialize();
  EXPECT_TRUE(state2.is_empty());
}

TEST(StateTest, Timestamp) {
  State state(StateType::STATE, "test", false);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_TRUE(state.is_deprecated(std::chrono::milliseconds(100)));
  state.reset_timestamp();
  EXPECT_FALSE(state.is_deprecated(std::chrono::milliseconds(100)));
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_TRUE(state.is_deprecated(std::chrono::milliseconds(100)));
  state.set_timestamp(std::chrono::steady_clock::now());
  EXPECT_FALSE(state.is_deprecated(std::chrono::milliseconds(100)));
}

TEST(StateTest, Swap) {
  State state1(StateType::CARTESIANSTATE, "cartesian", true);
  State state2(StateType::STATE, "state", false);
  swap(state1, state2);
  EXPECT_EQ(state1.get_type(), StateType::STATE);
  EXPECT_EQ(state1.get_name(), "state");
  EXPECT_FALSE(state1.is_empty());
  EXPECT_EQ(state2.get_type(), StateType::CARTESIANSTATE);
  EXPECT_EQ(state2.get_name(), "cartesian");
  EXPECT_TRUE(state2.is_empty());
}