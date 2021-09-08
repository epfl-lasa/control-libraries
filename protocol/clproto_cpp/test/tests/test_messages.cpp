#include <gtest/gtest.h>

#include <state_representation/State.hpp>

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
  EXPECT_EQ(recv_state.get_type(), StateType::STATE);
  EXPECT_STREQ(send_state.get_name().c_str(), recv_state.get_name().c_str());
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

/* If an encode / decode template is invoked that is not implemented in clproto,
 * there will be a linker error "undefined reference" at compile time.
 * Of course, it's not really possible to test this at run-time.
PSEUDO_TEST(CartesianProtoTest, EncodeInvalidObject) {
  foo::Object invalid_object;

  EXPECT_LINKER_ERROR(clproto::encode(invalid_object));
}
*/