#include <gtest/gtest.h>

#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/space/Jacobian.hpp>
#include <state_representation/robot/JointState.hpp>

#include "clproto.h"

using namespace state_representation;

class PackUnpackTest : public ::testing::Test {
protected:
  void SetUp() override {
    cart_state_ = CartesianState::Random("robot_ee", "robot_base");
    joint_state_ = JointState::Random("robot", 7);
    jacobian_ = Jacobian("robot", 7, "robot_ee", "robot_base");
  }

  std::vector<std::string> combine_state_message() {
    std::vector<std::string> message_fields;
    message_fields.emplace_back(clproto::encode(cart_state_));
    message_fields.emplace_back(clproto::encode(joint_state_));
    message_fields.emplace_back(clproto::encode(jacobian_));
    return message_fields;
  }

  void validate_state_messages(const std::vector<std::string>& message_fields) {
    ASSERT_EQ(message_fields.size(), nfields_);

    CartesianState decoded_cart_state;
    EXPECT_TRUE(clproto::is_valid(message_fields.at(0)));
    EXPECT_TRUE(clproto::check_message_type(message_fields.at(0)) == clproto::CARTESIAN_STATE_MESSAGE);
    EXPECT_NO_THROW(decoded_cart_state = clproto::decode<CartesianState>(message_fields.at(0)));
    EXPECT_STREQ(decoded_cart_state.get_name().c_str(), cart_state_.get_name().c_str());

    JointState decoded_joint_state;
    EXPECT_TRUE(clproto::is_valid(message_fields.at(1)));
    EXPECT_TRUE(clproto::check_message_type(message_fields.at(1)) == clproto::JOINT_STATE_MESSAGE);
    EXPECT_NO_THROW(decoded_joint_state = clproto::decode<JointState>(message_fields.at(1)));
    EXPECT_STREQ(decoded_joint_state.get_name().c_str(), decoded_joint_state.get_name().c_str());

    Jacobian decoded_jacobian;
    EXPECT_TRUE(clproto::is_valid(message_fields.at(2)));
    EXPECT_TRUE(clproto::check_message_type(message_fields.at(2)) == clproto::JACOBIAN_MESSAGE);
    EXPECT_NO_THROW(decoded_jacobian = clproto::decode<Jacobian>(message_fields.at(2)));
    EXPECT_STREQ(decoded_jacobian.get_name().c_str(), jacobian_.get_name().c_str());
  }

private:
  const int nfields_ = 3;
  CartesianState cart_state_;
  JointState joint_state_;
  Jacobian jacobian_;
};

TEST_F(PackUnpackTest, PackUnpackDataBuffer) {
  // combine some messages into a vector
  auto message_fields = combine_state_message();

  // pack the message fields into a raw char buffer
  char data_buffer[3 * CLPROTO_PACKING_MAX_FIELD_LENGTH];
  clproto::pack_fields(message_fields, data_buffer);

  // unpack message
  auto unpacked_fields = clproto::unpack_fields(data_buffer);
  validate_state_messages(unpacked_fields);
}

TEST_F(PackUnpackTest, PackUnpackStringBuffer) {
  // combine some messages into a vector
  auto message_fields = combine_state_message();

  // pack the message fields into a std string
  std::string encoded_packet;
  encoded_packet.reserve(2 * CLPROTO_PACKING_MAX_FIELD_LENGTH);
  clproto::pack_fields(message_fields, encoded_packet.data());

  // unpack and validate message
  auto unpacked_fields = clproto::unpack_fields(encoded_packet.c_str());
  validate_state_messages(unpacked_fields);
}