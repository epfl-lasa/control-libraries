#include <gtest/gtest.h>

#include <state_representation/State.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/space/cartesian/CartesianPose.hpp>

#include "clproto.h"

using namespace state_representation;

TEST(MessageProtoTest, EncodeDecodeState) {
  auto send_state = State(StateType::STATE, "A", false);
  std::string msg = clproto::encode(send_state);
  EXPECT_TRUE(clproto::is_valid(msg));
  EXPECT_TRUE(clproto::check_message_type(msg) == clproto::STATE_MESSAGE);

  State recv_state(StateType::PARAMETER_MATRIX);
  EXPECT_NO_THROW(clproto::decode<State>(msg));
  EXPECT_TRUE(clproto::decode(msg, recv_state));

  EXPECT_EQ(send_state.is_empty(), recv_state.is_empty());
  EXPECT_EQ(send_state.get_type(), recv_state.get_type());
  EXPECT_STREQ(send_state.get_name().c_str(), recv_state.get_name().c_str());
  EXPECT_EQ(send_state.get_timestamp().time_since_epoch().count(),
            recv_state.get_timestamp().time_since_epoch().count());
}

TEST(MessageProtoTest, DecodeInvalidString) {
  std::string dummy_msg = "hello world";

  EXPECT_FALSE(clproto::is_valid(dummy_msg));
  EXPECT_EQ(clproto::check_message_type(dummy_msg), clproto::UNKNOWN_MESSAGE);

  State obj(StateType::STATE);
  EXPECT_NO_THROW(clproto::decode(dummy_msg, obj));
  EXPECT_FALSE(clproto::decode(dummy_msg, obj));

  EXPECT_THROW(clproto::decode<State>(dummy_msg), clproto::DecodingException);
}

TEST(MessageProtoTest, DecodeParallelTypes) {
  auto state = CartesianState::Random("A", "B");
  auto pose = CartesianPose::Random("C", "D");
  auto encoded_state = clproto::encode(state);
  auto encoded_pose = clproto::encode(pose);

  EXPECT_THROW(clproto::decode<CartesianState>(encoded_pose), clproto::DecodingException);
  EXPECT_THROW(clproto::decode<CartesianPose>(encoded_state), clproto::DecodingException);
}

/* If an encode / decode template is invoked that is not implemented in clproto,
 * there will be a linker error "undefined reference" at compile time.
 * Of course, it's not really possible to test this at run-time.
PSEUDO_TEST(CartesianProtoTest, EncodeInvalidObject) {
  foo::Object invalid_object;

  EXPECT_LINKER_ERROR(clproto::encode(invalid_object));
}
*/