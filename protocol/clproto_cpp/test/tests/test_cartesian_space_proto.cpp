#include <gtest/gtest.h>

#include <state_representation/space/SpatialState.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <state_representation/space/cartesian/CartesianTwist.hpp>
#include <state_representation/space/cartesian/CartesianAcceleration.hpp>
#include <state_representation/space/cartesian/CartesianWrench.hpp>

#include "clproto.h"

using namespace state_representation;

template<typename T>
static void test_cart_state_equal(const T& send_state, const T& recv_state) {
  EXPECT_EQ(send_state.get_type(), recv_state.get_type());
  EXPECT_STREQ(send_state.get_name().c_str(), recv_state.get_name().c_str());
  EXPECT_STREQ(send_state.get_reference_frame().c_str(), recv_state.get_reference_frame().c_str());
  EXPECT_NEAR(send_state.dist(recv_state), 0, 1e-5);
}

template<typename T>
static void test_encode_decode_cartesian(const T& send_state, clproto::MessageType type) {
  std::string msg = clproto::encode(send_state);
  EXPECT_TRUE(clproto::is_valid(msg));
  EXPECT_EQ(clproto::check_message_type(msg), type);

  T recv_state;
  EXPECT_NO_THROW(clproto::decode<T>(msg));
  EXPECT_TRUE(clproto::decode(msg, recv_state));
  EXPECT_FALSE(recv_state.is_empty());

  test_cart_state_equal(send_state, recv_state);

  auto send_state_ptr = make_shared_state(send_state);
  msg = clproto::encode(send_state_ptr);
  EXPECT_TRUE(clproto::is_valid(msg));
  EXPECT_EQ(clproto::check_message_type(msg), type);

  T recv_state_2;
  auto recv_state_ptr = make_shared_state(recv_state_2);
  EXPECT_NO_THROW(clproto::decode<std::shared_ptr<State>>(msg));
  EXPECT_TRUE(clproto::decode(msg, recv_state_ptr));

  recv_state_2 = *std::dynamic_pointer_cast<T>(recv_state_ptr);
  test_cart_state_equal(send_state, recv_state_2);
}

template<typename T>
static void test_encode_decode_empty_cartesian(const T& state) {
  EXPECT_TRUE(state.is_empty());
  std::string msg;
  EXPECT_NO_THROW(msg = clproto::encode(state));

  T recv_state;
  EXPECT_NO_THROW(recv_state = clproto::decode<T>(msg));
  EXPECT_TRUE(recv_state.is_empty());
}

TEST(CartesianProtoTest, EncodeDecodeSpatialState) {
  auto send_state = SpatialState(StateType::SPATIAL_STATE, "A", "B", false);
  std::string msg = clproto::encode(send_state);
  EXPECT_TRUE(clproto::is_valid(msg));
  EXPECT_TRUE(clproto::check_message_type(msg) == clproto::SPATIAL_STATE_MESSAGE);

  SpatialState recv_state(StateType::SPATIAL_STATE);
  EXPECT_NO_THROW(clproto::decode<SpatialState>(msg));
  EXPECT_TRUE(clproto::decode(msg, recv_state));

  EXPECT_EQ(send_state.get_type(), recv_state.get_type());
  EXPECT_EQ(send_state.is_empty(), recv_state.is_empty());
  EXPECT_STREQ(send_state.get_name().c_str(), recv_state.get_name().c_str());
  EXPECT_STREQ(send_state.get_reference_frame().c_str(), recv_state.get_reference_frame().c_str());

  std::shared_ptr<State> send_state_ptr = std::make_shared<SpatialState>(send_state);
  msg = clproto::encode(send_state_ptr);
  EXPECT_TRUE(clproto::is_valid(msg));
  EXPECT_TRUE(clproto::check_message_type(msg) == clproto::SPATIAL_STATE_MESSAGE);

  SpatialState recv_state_2(StateType::SPATIAL_STATE);
  auto recv_state_ptr = make_shared_state(recv_state_2);
  EXPECT_NO_THROW(clproto::decode<std::shared_ptr<State>>(msg));
  EXPECT_TRUE(clproto::decode(msg, recv_state_ptr));

  recv_state_2 = *std::dynamic_pointer_cast<SpatialState>(recv_state_ptr);
  EXPECT_EQ(send_state.get_type(), recv_state_2.get_type());
  EXPECT_EQ(send_state.is_empty(), recv_state_2.is_empty());
  EXPECT_STREQ(send_state.get_name().c_str(), recv_state_2.get_name().c_str());
  EXPECT_STREQ(send_state.get_reference_frame().c_str(), recv_state_2.get_reference_frame().c_str());
}

TEST(CartesianProtoTest, EncodeDecodeRandomCartesian) {
  test_encode_decode_cartesian(CartesianState::Random("A", "B"), clproto::CARTESIAN_STATE_MESSAGE);
  test_encode_decode_cartesian(CartesianPose::Random("A", "B"), clproto::CARTESIAN_POSE_MESSAGE);
  test_encode_decode_cartesian(CartesianTwist::Random("A", "B"), clproto::CARTESIAN_TWIST_MESSAGE);
  test_encode_decode_cartesian(CartesianAcceleration::Random("A", "B"), clproto::CARTESIAN_ACCELERATION_MESSAGE);
  test_encode_decode_cartesian(CartesianWrench::Random("A", "B"), clproto::CARTESIAN_WRENCH_MESSAGE);
}

TEST(CartesianProtoTest, EncodeDecodeEmptyCartesian) {
  test_encode_decode_empty_cartesian(CartesianState());
  test_encode_decode_empty_cartesian(CartesianPose());
  test_encode_decode_empty_cartesian(CartesianTwist());
  test_encode_decode_empty_cartesian(CartesianAcceleration());
  test_encode_decode_empty_cartesian(CartesianWrench());
}
