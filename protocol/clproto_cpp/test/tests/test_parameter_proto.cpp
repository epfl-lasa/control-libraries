#include <gtest/gtest.h>

#include <state_representation/parameters/Parameter.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>

#include "clproto.h"

using namespace state_representation;

template<typename T>
static void test_parameter_equal(const T& send_state, const T& recv_state) {
  EXPECT_STREQ(send_state.get_name().c_str(), recv_state.get_name().c_str());
  EXPECT_EQ(send_state.get_value(), recv_state.get_value());
  EXPECT_EQ(send_state.get_type(), recv_state.get_type());
}

template<typename T>
static void test_encode_decode_parameter(
    const T& send_state, clproto::MessageType msg_type, clproto::ParameterMessageType param_type
) {
  std::string msg = clproto::encode(send_state);
  EXPECT_TRUE(clproto::is_valid(msg));
  EXPECT_EQ(clproto::check_message_type(msg), msg_type);
  EXPECT_EQ(clproto::check_parameter_message_type(msg), param_type);

  T recv_state("");
  EXPECT_NO_THROW(clproto::decode<T>(msg));
  EXPECT_TRUE(clproto::decode(msg, recv_state));
  EXPECT_FALSE(recv_state.is_empty());

  test_parameter_equal(send_state, recv_state);

  auto send_state_ptr = make_shared_state(send_state);
  msg = clproto::encode(send_state_ptr);
  EXPECT_TRUE(clproto::is_valid(msg));
  EXPECT_EQ(clproto::check_message_type(msg), msg_type);
  EXPECT_EQ(clproto::check_parameter_message_type(msg), param_type);

  T recv_state_2("");
  auto recv_state_ptr = make_shared_state(recv_state_2);
  EXPECT_NO_THROW(clproto::decode<std::shared_ptr<State>>(msg));
  EXPECT_TRUE(clproto::decode(msg, recv_state_ptr));

  recv_state_2 = *std::dynamic_pointer_cast<T>(recv_state_ptr);
  test_parameter_equal(send_state, recv_state_2);
}

template<typename T>
static void test_encode_decode_empty_parameter(const T& state) {
  EXPECT_TRUE(state.is_empty());
  std::string msg;
  EXPECT_NO_THROW(msg = clproto::encode(state));

  T recv_state("");
  EXPECT_NO_THROW(recv_state = clproto::decode<T>(msg));
  EXPECT_TRUE(recv_state.is_empty());
}

TEST(ParameterProtoTest, EncodeDecodeParameterInt) {
  test_encode_decode_parameter(
      Parameter<int>("A", 1), clproto::MessageType::PARAMETER_MESSAGE, clproto::ParameterMessageType::INT
  );
  test_encode_decode_parameter(
      Parameter<std::vector<int>>("A", {1, 2, 3}), clproto::MessageType::PARAMETER_MESSAGE,
      clproto::ParameterMessageType::INT_ARRAY
  );
  test_encode_decode_parameter(
      Parameter<double>("A", 1.0), clproto::MessageType::PARAMETER_MESSAGE, clproto::ParameterMessageType::DOUBLE
  );
  test_encode_decode_parameter(
      Parameter<std::vector<double>>("A", {1.0, 2.0, 3.0}), clproto::MessageType::PARAMETER_MESSAGE,
      clproto::ParameterMessageType::DOUBLE_ARRAY
  );
  test_encode_decode_parameter(
      Parameter<bool>("A", true), clproto::MessageType::PARAMETER_MESSAGE, clproto::ParameterMessageType::BOOL
  );
  test_encode_decode_parameter(
      Parameter<std::vector<bool>>("A", {true, false, true, false}), clproto::MessageType::PARAMETER_MESSAGE,
      clproto::ParameterMessageType::BOOL_ARRAY
  );
  test_encode_decode_parameter(
      Parameter<std::string>("A", "value"), clproto::MessageType::PARAMETER_MESSAGE,
      clproto::ParameterMessageType::STRING
  );
  test_encode_decode_parameter(
      Parameter<std::vector<std::string>>("A", {"value 1", "value 2", "value 3"}),
      clproto::MessageType::PARAMETER_MESSAGE, clproto::ParameterMessageType::STRING_ARRAY
  );
  Eigen::VectorXd vector_value(5);
  vector_value << 1.0, 2.0, 3.0, 4.0, 5.0;
  test_encode_decode_parameter(
      Parameter<Eigen::VectorXd>("A", vector_value), clproto::MessageType::PARAMETER_MESSAGE,
      clproto::ParameterMessageType::VECTOR
  );
  Eigen::MatrixXd matrix_value(2, 3);
  matrix_value << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
  test_encode_decode_parameter(
      Parameter<Eigen::MatrixXd>("A", matrix_value), clproto::MessageType::PARAMETER_MESSAGE,
      clproto::ParameterMessageType::MATRIX
  );
}

TEST(ParameterProtoTest, EncodeDecodeEmptyParameter) {
  test_encode_decode_empty_parameter(Parameter<int>(""));
  test_encode_decode_empty_parameter(Parameter<std::vector<int>>(""));
  test_encode_decode_empty_parameter(Parameter<double>(""));
  test_encode_decode_empty_parameter(Parameter<std::vector<double>>(""));
  test_encode_decode_empty_parameter(Parameter<bool>(""));
  test_encode_decode_empty_parameter(Parameter<std::vector<bool>>(""));
  test_encode_decode_empty_parameter(Parameter<std::string>(""));
  test_encode_decode_empty_parameter(Parameter<std::vector<std::string>>(""));
  test_encode_decode_empty_parameter(Parameter<Eigen::VectorXd>(""));
  test_encode_decode_empty_parameter(Parameter<Eigen::MatrixXd>(""));
}

TEST(MessageProtoTest, EncodeDecodeInvalidParameter) {
  auto send_state = Parameter<CartesianState>("state", CartesianState::Random("state"));
  auto send_state_ptr = make_shared_state(send_state);
  EXPECT_THROW(clproto::encode(send_state_ptr), std::invalid_argument);

  auto send_state_2 = State(StateType::STATE, "A", false);
  std::string msg = clproto::encode(send_state_2);

  Parameter<CartesianState> recv_state("");
  auto recv_state_ptr = make_shared_state(recv_state);
  EXPECT_FALSE(clproto::decode(msg, recv_state_ptr));
}
