#include <gtest/gtest.h>

#include <state_representation/space/Jacobian.hpp>
#include <state_representation/robot/JointState.hpp>
#include <state_representation/robot/JointPositions.hpp>
#include <state_representation/robot/JointVelocities.hpp>
#include <state_representation/robot/JointAccelerations.hpp>
#include <state_representation/robot/JointTorques.hpp>

#include "clproto.h"

using namespace state_representation;

TEST(JointProtoTest, EncodeDecodeJacobian) {
  auto send_state = Jacobian::Random("robot", {"one", "two", "three"}, "A", "B");
  std::string msg = clproto::encode(send_state);
  EXPECT_TRUE(clproto::is_valid(msg));
  EXPECT_TRUE(clproto::check_message_type(msg) == clproto::JACOBIAN_MESSAGE);

  Jacobian recv_state;
  EXPECT_NO_THROW(clproto::decode<Jacobian>(msg));
  EXPECT_TRUE(clproto::decode(msg, recv_state));
  EXPECT_FALSE(recv_state.is_empty());

  EXPECT_STREQ(send_state.get_name().c_str(), recv_state.get_name().c_str());
  EXPECT_STREQ(send_state.get_frame().c_str(), recv_state.get_frame().c_str());
  EXPECT_STREQ(send_state.get_reference_frame().c_str(), recv_state.get_reference_frame().c_str());
  EXPECT_EQ(send_state.rows(), recv_state.rows());
  EXPECT_EQ(send_state.cols(), recv_state.cols());
  ASSERT_EQ(send_state.get_joint_names().size(), recv_state.get_joint_names().size());
  for (std::size_t ind = 0; ind < send_state.get_joint_names().size(); ++ind) {
    EXPECT_STREQ(send_state.get_joint_names().at(ind).c_str(), recv_state.get_joint_names().at(ind).c_str());
  }
  EXPECT_NEAR(send_state.data().norm(), recv_state.data().norm(), 1e-5);
}

TEST(JointProtoTest, EncodeDecodeJointState) {
  std::vector<std::string> joint_names = {"apple", "orange", "banana", "prune"};
  auto send_state = JointState::Random("zeiss", joint_names);
  std::string msg = clproto::encode(send_state);
  EXPECT_TRUE(clproto::is_valid(msg));
  EXPECT_TRUE(clproto::check_message_type(msg) == clproto::JOINT_STATE_MESSAGE);

  JointState recv_state;
  EXPECT_NO_THROW(clproto::decode<JointState>(msg));
  EXPECT_TRUE(clproto::decode(msg, recv_state));
  EXPECT_FALSE(recv_state.is_empty());

  EXPECT_STREQ(send_state.get_name().c_str(), recv_state.get_name().c_str());
  ASSERT_EQ(send_state.get_size(), recv_state.get_size());
  for (std::size_t ind = 0; ind < send_state.get_size(); ++ind) {
    EXPECT_STREQ(send_state.get_names().at(ind).c_str(), recv_state.get_names().at(ind).c_str());
  }
  EXPECT_NEAR(send_state.dist(recv_state), 0, 1e-5);
}

TEST(JointProtoTest, EncodeDecodeJointPositions) {
  auto send_state = JointPositions::Random("robot", 3);
  std::string msg = clproto::encode(send_state);
  EXPECT_TRUE(clproto::is_valid(msg));
  EXPECT_TRUE(clproto::check_message_type(msg) == clproto::JOINT_POSITIONS_MESSAGE);

  JointPositions recv_state;
  EXPECT_NO_THROW(clproto::decode<JointPositions>(msg));
  EXPECT_TRUE(clproto::decode(msg, recv_state));
  EXPECT_FALSE(recv_state.is_empty());

  EXPECT_STREQ(send_state.get_name().c_str(), recv_state.get_name().c_str());
  ASSERT_EQ(send_state.get_size(), recv_state.get_size());
  for (std::size_t ind = 0; ind < send_state.get_size(); ++ind) {
    EXPECT_STREQ(send_state.get_names().at(ind).c_str(), recv_state.get_names().at(ind).c_str());
  }
  EXPECT_NEAR(send_state.dist(recv_state), 0, 1e-5);
}

TEST(JointProtoTest, EncodeDecodeJointVelocities) {
  auto send_state = JointVelocities::Random("robot", 3);
  std::string msg = clproto::encode(send_state);
  EXPECT_TRUE(clproto::is_valid(msg));
  EXPECT_TRUE(clproto::check_message_type(msg) == clproto::JOINT_VELOCITIES_MESSAGE);

  JointVelocities recv_state;
  EXPECT_NO_THROW(clproto::decode<JointVelocities>(msg));
  EXPECT_TRUE(clproto::decode(msg, recv_state));
  EXPECT_FALSE(recv_state.is_empty());

  EXPECT_STREQ(send_state.get_name().c_str(), recv_state.get_name().c_str());
  ASSERT_EQ(send_state.get_size(), recv_state.get_size());
  for (std::size_t ind = 0; ind < send_state.get_size(); ++ind) {
    EXPECT_STREQ(send_state.get_names().at(ind).c_str(), recv_state.get_names().at(ind).c_str());
  }
  EXPECT_NEAR(send_state.dist(recv_state), 0, 1e-5);
}

TEST(JointProtoTest, EncodeDecodeJointAccelerations) {
  auto send_state = JointAccelerations::Random("robot", 3);
  std::string msg = clproto::encode(send_state);
  EXPECT_TRUE(clproto::is_valid(msg));
  EXPECT_TRUE(clproto::check_message_type(msg) == clproto::JOINT_ACCELERATIONS_MESSAGE);

  JointAccelerations recv_state;
  EXPECT_NO_THROW(clproto::decode<JointAccelerations>(msg));
  EXPECT_TRUE(clproto::decode(msg, recv_state));
  EXPECT_FALSE(recv_state.is_empty());

  EXPECT_STREQ(send_state.get_name().c_str(), recv_state.get_name().c_str());
  ASSERT_EQ(send_state.get_size(), recv_state.get_size());
  for (std::size_t ind = 0; ind < send_state.get_size(); ++ind) {
    EXPECT_STREQ(send_state.get_names().at(ind).c_str(), recv_state.get_names().at(ind).c_str());
  }
  EXPECT_NEAR(send_state.dist(recv_state), 0, 1e-5);
}

TEST(JointProtoTest, EncodeDecodeJointTorques) {
  auto send_state = JointTorques::Random("robot", 3);
  std::string msg = clproto::encode(send_state);
  EXPECT_TRUE(clproto::is_valid(msg));
  EXPECT_TRUE(clproto::check_message_type(msg) == clproto::JOINT_TORQUES_MESSAGE);

  JointTorques recv_state;
  EXPECT_NO_THROW(clproto::decode<JointTorques>(msg));
  EXPECT_TRUE(clproto::decode(msg, recv_state));
  EXPECT_FALSE(recv_state.is_empty());

  EXPECT_STREQ(send_state.get_name().c_str(), recv_state.get_name().c_str());
  ASSERT_EQ(send_state.get_size(), recv_state.get_size());
  for (std::size_t ind = 0; ind < send_state.get_size(); ++ind) {
    EXPECT_STREQ(send_state.get_names().at(ind).c_str(), recv_state.get_names().at(ind).c_str());
  }
  EXPECT_NEAR(send_state.dist(recv_state), 0, 1e-5);
}

TEST(JointProtoTest, EncodeDecodeEmptyJacobian) {
  Jacobian empty_state;
  EXPECT_TRUE(empty_state.is_empty());
  std::string msg;
  EXPECT_NO_THROW(msg = clproto::encode(empty_state));

  Jacobian recv_state;
  EXPECT_NO_THROW(recv_state = clproto::decode<Jacobian>(msg));
  EXPECT_TRUE(recv_state.is_empty());
}

TEST(JointProtoTest, EncodeDecodeEmptyJointPositions) {
  JointPositions empty_state;
  EXPECT_TRUE(empty_state.is_empty());
  std::string msg;
  EXPECT_NO_THROW(msg = clproto::encode(empty_state));

  JointPositions recv_state;
  EXPECT_NO_THROW(recv_state = clproto::decode<JointPositions>(msg));
  EXPECT_TRUE(recv_state.is_empty());
}

TEST(JointProtoTest, EncodeDecodeEmptyJointState) {
  JointState empty_state;
  EXPECT_TRUE(empty_state.is_empty());
  std::string msg;
  EXPECT_NO_THROW(msg = clproto::encode(empty_state));

  JointState recv_state;
  EXPECT_NO_THROW(recv_state = clproto::decode<JointState>(msg));
  EXPECT_TRUE(recv_state.is_empty());
}

TEST(JointProtoTest, EncodeDecodeEmptyJointVelocities) {
  JointVelocities empty_state;
  EXPECT_TRUE(empty_state.is_empty());
  std::string msg;
  EXPECT_NO_THROW(msg = clproto::encode(empty_state));

  JointVelocities recv_state;
  EXPECT_NO_THROW(recv_state = clproto::decode<JointVelocities>(msg));
  EXPECT_TRUE(recv_state.is_empty());
}

TEST(JointProtoTest, EncodeDecodeEmptyJointAccelerations) {
  JointAccelerations empty_state;
  EXPECT_TRUE(empty_state.is_empty());
  std::string msg;
  EXPECT_NO_THROW(msg = clproto::encode(empty_state));

  JointAccelerations recv_state;
  EXPECT_NO_THROW(recv_state = clproto::decode<JointAccelerations>(msg));
  EXPECT_TRUE(recv_state.is_empty());
}

TEST(JointProtoTest, EncodeDecodeEmptyJointTorques) {
  JointTorques empty_state;
  EXPECT_TRUE(empty_state.is_empty());
  std::string msg;
  EXPECT_NO_THROW(msg = clproto::encode(empty_state));

  JointTorques recv_state;
  EXPECT_NO_THROW(recv_state = clproto::decode<JointTorques>(msg));
  EXPECT_TRUE(recv_state.is_empty());
}
