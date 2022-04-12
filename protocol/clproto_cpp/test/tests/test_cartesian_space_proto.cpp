#include <gtest/gtest.h>

#include <state_representation/space/SpatialState.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <state_representation/space/cartesian/CartesianTwist.hpp>
#include <state_representation/space/cartesian/CartesianAcceleration.hpp>
#include <state_representation/space/cartesian/CartesianWrench.hpp>

#include "clproto.h"

using namespace state_representation;


TEST(CartesianProtoTest, EncodeDecodeSpatialState) {
  auto send_state = SpatialState(StateType::SPATIAL_STATE, "A", "B", false);
  std::string msg = clproto::encode(send_state);
  EXPECT_TRUE(clproto::is_valid(msg));
  EXPECT_TRUE(clproto::check_message_type(msg) == clproto::SPATIAL_STATE_MESSAGE);

  SpatialState recv_state(StateType::STATE);
  EXPECT_NO_THROW(clproto::decode<SpatialState>(msg));
  EXPECT_TRUE(clproto::decode(msg, recv_state));

  EXPECT_EQ(send_state.is_empty(), recv_state.is_empty());
  EXPECT_STREQ(send_state.get_name().c_str(), recv_state.get_name().c_str());
  EXPECT_STREQ(send_state.get_reference_frame().c_str(), recv_state.get_reference_frame().c_str());
}

TEST(CartesianProtoTest, EncodeDecodeCartesianState) {
  auto send_state = CartesianState::Random("A", "B");
  std::string msg = clproto::encode(send_state);
  EXPECT_TRUE(clproto::is_valid(msg));
  EXPECT_TRUE(clproto::check_message_type(msg) == clproto::CARTESIAN_STATE_MESSAGE);

  CartesianState recv_state;
  EXPECT_NO_THROW(clproto::decode<CartesianState>(msg));
  EXPECT_TRUE(clproto::decode(msg, recv_state));
  EXPECT_FALSE(recv_state.is_empty());

  EXPECT_EQ(send_state.get_type(), recv_state.get_type());
  EXPECT_STREQ(send_state.get_name().c_str(), recv_state.get_name().c_str());
  EXPECT_STREQ(send_state.get_reference_frame().c_str(), recv_state.get_reference_frame().c_str());
  EXPECT_NEAR(send_state.dist(recv_state), 0, 1e-5);
}

TEST(CartesianProtoTest, EncodeDecodeCartesianPose) {
  auto send_state = CartesianPose::Random("A", "B");
  std::string msg = clproto::encode(send_state);
  EXPECT_TRUE(clproto::is_valid(msg));
  EXPECT_TRUE(clproto::check_message_type(msg) == clproto::CARTESIAN_POSE_MESSAGE);

  CartesianPose recv_state;
  EXPECT_NO_THROW(clproto::decode<CartesianPose>(msg));
  EXPECT_TRUE(clproto::decode(msg, recv_state));
  EXPECT_FALSE(recv_state.is_empty());

  EXPECT_EQ(send_state.get_type(), recv_state.get_type());
  EXPECT_STREQ(send_state.get_name().c_str(), recv_state.get_name().c_str());
  EXPECT_STREQ(send_state.get_reference_frame().c_str(), recv_state.get_reference_frame().c_str());
  EXPECT_NEAR(send_state.dist(recv_state), 0, 1e-5);
}

TEST(CartesianProtoTest, EncodeDecodeCartesianTwist) {
  auto send_state = CartesianTwist::Random("A", "B");
  std::string msg = clproto::encode(send_state);
  EXPECT_TRUE(clproto::is_valid(msg));
  EXPECT_TRUE(clproto::check_message_type(msg) == clproto::CARTESIAN_TWIST_MESSAGE);

  CartesianTwist recv_state;
  EXPECT_NO_THROW(clproto::decode<CartesianTwist>(msg));
  EXPECT_TRUE(clproto::decode(msg, recv_state));
  EXPECT_FALSE(recv_state.is_empty());

  EXPECT_EQ(send_state.get_type(), recv_state.get_type());
  EXPECT_STREQ(send_state.get_name().c_str(), recv_state.get_name().c_str());
  EXPECT_STREQ(send_state.get_reference_frame().c_str(), recv_state.get_reference_frame().c_str());
  EXPECT_NEAR(send_state.dist(recv_state), 0, 1e-5);
}

TEST(CartesianProtoTest, EncodeDecodeCartesianAcceleration) {
  auto send_state = CartesianAcceleration::Random("A", "B");
  std::string msg = clproto::encode(send_state);
  EXPECT_TRUE(clproto::is_valid(msg));
  EXPECT_TRUE(clproto::check_message_type(msg) == clproto::CARTESIAN_ACCELERATION_MESSAGE);

  CartesianAcceleration recv_state;
  EXPECT_NO_THROW(clproto::decode<CartesianAcceleration>(msg));
  EXPECT_TRUE(clproto::decode(msg, recv_state));
  EXPECT_FALSE(recv_state.is_empty());

  EXPECT_EQ(send_state.get_type(), recv_state.get_type());
  EXPECT_STREQ(send_state.get_name().c_str(), recv_state.get_name().c_str());
  EXPECT_STREQ(send_state.get_reference_frame().c_str(), recv_state.get_reference_frame().c_str());
  EXPECT_NEAR(send_state.dist(recv_state), 0, 1e-5);
}

TEST(CartesianProtoTest, EncodeDecodeCartesianWrench) {
  auto send_state = CartesianWrench::Random("A", "B");
  std::string msg = clproto::encode(send_state);
  EXPECT_TRUE(clproto::is_valid(msg));
  EXPECT_TRUE(clproto::check_message_type(msg) == clproto::CARTESIAN_WRENCH_MESSAGE);

  CartesianWrench recv_state;
  EXPECT_NO_THROW(clproto::decode<CartesianWrench>(msg));
  EXPECT_TRUE(clproto::decode(msg, recv_state));
  EXPECT_FALSE(recv_state.is_empty());

  EXPECT_EQ(send_state.get_type(), recv_state.get_type());
  EXPECT_STREQ(send_state.get_name().c_str(), recv_state.get_name().c_str());
  EXPECT_STREQ(send_state.get_reference_frame().c_str(), recv_state.get_reference_frame().c_str());
  EXPECT_NEAR(send_state.dist(recv_state), 0, 1e-5);
}

TEST(CartesianProtoTest, EncodeDecodeEmptyCartesianState) {
  CartesianState empty_state;
  EXPECT_TRUE(empty_state.is_empty());
  std::string msg;
  EXPECT_NO_THROW(msg = clproto::encode(empty_state));

  CartesianState recv_state;
  EXPECT_NO_THROW(recv_state = clproto::decode<CartesianState>(msg));
  EXPECT_TRUE(recv_state.is_empty());
}

TEST(CartesianProtoTest, EncodeDecodeEmptyCartesianPose) {
  CartesianPose empty_state;
  EXPECT_TRUE(empty_state.is_empty());
  std::string msg;
  EXPECT_NO_THROW(msg = clproto::encode(empty_state));

  CartesianPose recv_state;
  EXPECT_NO_THROW(recv_state = clproto::decode<CartesianPose>(msg));
  EXPECT_TRUE(recv_state.is_empty());
}

TEST(CartesianProtoTest, EncodeDecodeEmptyCartesianTwist) {
  CartesianTwist empty_state;
  EXPECT_TRUE(empty_state.is_empty());
  std::string msg;
  EXPECT_NO_THROW(msg = clproto::encode(empty_state));

  CartesianTwist recv_state;
  EXPECT_NO_THROW(recv_state = clproto::decode<CartesianTwist>(msg));
  EXPECT_TRUE(recv_state.is_empty());
}

TEST(CartesianProtoTest, EncodeDecodeEmptyCartesianAcceleration) {
  CartesianAcceleration empty_state;
  EXPECT_TRUE(empty_state.is_empty());
  std::string msg;
  EXPECT_NO_THROW(msg = clproto::encode(empty_state));

  CartesianAcceleration recv_state;
  EXPECT_NO_THROW(recv_state = clproto::decode<CartesianAcceleration>(msg));
  EXPECT_TRUE(recv_state.is_empty());
}

TEST(CartesianProtoTest, EncodeDecodeEmptyCartesianWrench) {
  CartesianWrench empty_state;
  EXPECT_TRUE(empty_state.is_empty());
  std::string msg;
  EXPECT_NO_THROW(msg = clproto::encode(empty_state));

  CartesianWrench recv_state;
  EXPECT_NO_THROW(recv_state = clproto::decode<CartesianWrench>(msg));
  EXPECT_TRUE(recv_state.is_empty());
}