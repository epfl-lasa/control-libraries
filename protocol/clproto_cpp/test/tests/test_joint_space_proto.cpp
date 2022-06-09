#include <gtest/gtest.h>

#include <state_representation/space/Jacobian.hpp>
#include <state_representation/space/joint/JointState.hpp>
#include <state_representation/space/joint/JointPositions.hpp>
#include <state_representation/space/joint/JointVelocities.hpp>
#include <state_representation/space/joint/JointAccelerations.hpp>
#include <state_representation/space/joint/JointTorques.hpp>

#include "clproto.h"

using namespace state_representation;

template<typename T>
static void test_joint_state_equal(const T& send_state, const T& recv_state) {
  EXPECT_EQ(send_state.get_type(), recv_state.get_type());
  EXPECT_STREQ(send_state.get_name().c_str(), recv_state.get_name().c_str());
  ASSERT_EQ(send_state.get_size(), recv_state.get_size());
  for (std::size_t ind = 0; ind < send_state.get_size(); ++ind) {
    EXPECT_STREQ(send_state.get_names().at(ind).c_str(), recv_state.get_names().at(ind).c_str());
  }
  EXPECT_NEAR(send_state.dist(recv_state), 0, 1e-5);
}

static void test_jacobian_equal(const Jacobian& send_state, const Jacobian& recv_state) {
  EXPECT_EQ(send_state.get_type(), recv_state.get_type());
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

template<typename T>
static void test_encode_decode(const T& send_state, clproto::MessageType type) {
  std::string msg = clproto::encode(send_state);
  EXPECT_TRUE(clproto::is_valid(msg));
  EXPECT_EQ(clproto::check_message_type(msg), type);

  T recv_state;
  EXPECT_NO_THROW(clproto::decode<T>(msg));
  EXPECT_TRUE(clproto::decode(msg, recv_state));
  EXPECT_FALSE(recv_state.is_empty());

  test_joint_state_equal(send_state, recv_state);

  auto send_state_ptr = make_shared_state(send_state);
  msg = clproto::encode(send_state_ptr);
  EXPECT_TRUE(clproto::is_valid(msg));
  EXPECT_EQ(clproto::check_message_type(msg), type);

  T recv_state_2;
  auto recv_state_ptr = make_shared_state(recv_state_2);
  EXPECT_NO_THROW(clproto::decode<std::shared_ptr<State>>(msg));
  EXPECT_TRUE(clproto::decode(msg, recv_state_ptr));

  recv_state_2 = *std::dynamic_pointer_cast<T>(recv_state_ptr);
  test_joint_state_equal(send_state, recv_state_2);
}

template<typename T>
static void test_encode_decode_empty_joint(const T& state) {
  EXPECT_TRUE(state.is_empty());
  std::string msg;
  EXPECT_NO_THROW(msg = clproto::encode(state));

  T recv_state;
  EXPECT_NO_THROW(recv_state = clproto::decode<T>(msg));
  EXPECT_TRUE(recv_state.is_empty());
}

TEST(JointProtoTest, EncodeDecodeRandomJoint) {
  std::vector<std::string> joint_names = {"apple", "orange", "banana", "prune"};
  auto send_state = JointState::Random("zeiss", joint_names);
  test_encode_decode(send_state, clproto::JOINT_STATE_MESSAGE);
  test_encode_decode(JointPositions::Random("robot", 3), clproto::JOINT_POSITIONS_MESSAGE);
  test_encode_decode(JointVelocities::Random("robot", 3), clproto::JOINT_VELOCITIES_MESSAGE);
  test_encode_decode(JointAccelerations::Random("robot", 3), clproto::JOINT_ACCELERATIONS_MESSAGE);
  test_encode_decode(JointTorques::Random("robot", 3), clproto::JOINT_TORQUES_MESSAGE);
}

TEST(CartesianProtoTest, EncodeDecodeEmptyJoint) {
  test_encode_decode_empty_joint(JointState());
  test_encode_decode_empty_joint(JointPositions());
  test_encode_decode_empty_joint(JointVelocities());
  test_encode_decode_empty_joint(JointAccelerations());
  test_encode_decode_empty_joint(JointTorques());
}

TEST(JointProtoTest, EncodeDecodeJacobian) {
  auto send_state = Jacobian::Random("robot", {"one", "two", "three"}, "A", "B");
  std::string msg = clproto::encode(send_state);
  EXPECT_TRUE(clproto::is_valid(msg));
  EXPECT_EQ(clproto::check_message_type(msg), clproto::JACOBIAN_MESSAGE);

  Jacobian recv_state;
  EXPECT_NO_THROW(clproto::decode<Jacobian>(msg));
  EXPECT_TRUE(clproto::decode(msg, recv_state));
  EXPECT_FALSE(recv_state.is_empty());

  test_jacobian_equal(send_state, recv_state);

  auto send_state_ptr = make_shared_state(send_state);
  msg = clproto::encode(send_state_ptr);
  EXPECT_TRUE(clproto::is_valid(msg));
  EXPECT_EQ(clproto::check_message_type(msg), clproto::JACOBIAN_MESSAGE);

  Jacobian recv_state_2;
  auto recv_state_ptr = make_shared_state(recv_state_2);
  EXPECT_NO_THROW(clproto::decode<std::shared_ptr<State>>(msg));
  EXPECT_TRUE(clproto::decode(msg, recv_state_ptr));

  recv_state_2 = *std::dynamic_pointer_cast<Jacobian>(recv_state_ptr);
  test_jacobian_equal(send_state, recv_state_2);
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
