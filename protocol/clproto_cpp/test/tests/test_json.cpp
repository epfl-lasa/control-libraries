#include <gtest/gtest.h>

#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/space/joint/JointState.hpp>

#include "clproto.h"

using namespace state_representation;

TEST(JsonProtoTest, JsonToFromBinary) {
  auto send_state = CartesianState::Random("A", "B");
  std::string msg = clproto::encode(send_state);
  ASSERT_TRUE(clproto::is_valid(msg));
  ASSERT_TRUE(clproto::check_message_type(msg) == clproto::CARTESIAN_STATE_MESSAGE);

  auto json = clproto::to_json(msg);
  EXPECT_GT(json.size(), 2);  // empty JSON would be "{}"
  auto msg2 = clproto::from_json(json);

  CartesianState recv_state;
  EXPECT_NO_THROW(clproto::decode<CartesianState>(msg));
  EXPECT_TRUE(clproto::decode(msg, recv_state));

  EXPECT_STREQ(send_state.get_name().c_str(), recv_state.get_name().c_str());
  EXPECT_STREQ(send_state.get_reference_frame().c_str(), recv_state.get_reference_frame().c_str());
  EXPECT_NEAR(send_state.dist(recv_state), 0, 1e-5);
}

TEST(JsonProtoTest, JsonToFromObject) {
  auto send_state = CartesianState::Random("A", "B");

  auto json = clproto::to_json(send_state);
  EXPECT_GT(json.size(), 2);  // empty JSON would be "{}"
  auto recv_state = clproto::from_json<CartesianState>(json);

  EXPECT_STREQ(send_state.get_name().c_str(), recv_state.get_name().c_str());
  EXPECT_STREQ(send_state.get_reference_frame().c_str(), recv_state.get_reference_frame().c_str());
  EXPECT_NEAR(send_state.dist(recv_state), 0, 1e-5);
}

TEST(JsonProtoTest, JsonToFromInvalid) {
  auto json = clproto::to_json(CartesianState::Random("A", "B"));

  EXPECT_NO_THROW(clproto::from_json<CartesianState>(json));
  EXPECT_THROW(clproto::from_json<CartesianState>("definitely not json"), clproto::JsonParsingException);
  EXPECT_THROW(clproto::from_json("definitely not json"), clproto::JsonParsingException);

  // valid JSON of the wrong state type should instead throw a decoding exception
  EXPECT_THROW(clproto::from_json<JointState>(json), clproto::DecodingException);

  EXPECT_THROW(clproto::to_json(std::string("definitely not binary encoded data")), clproto::JsonParsingException);
}

/* If a to_json template is invoked that is not implemented in clproto,
 * there will be a linker error "undefined reference" at compile time.
 * Of course, it's not really possible to test this at run-time.
PSEUDO_TEST(JsonProtoTest, JsonToInvalidObject) {
  foo::Object invalid_object;

  EXPECT_LINKER_ERROR(clproto::to_json(invalid_object));
}
*/