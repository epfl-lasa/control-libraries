#include <gtest/gtest.h>

#include <state_representation/parameters/Parameter.hpp>

#include "clproto.h"

using namespace state_representation;

TEST(ParameterProtoTest, EncodeDecodeParameterInt) {
  int value = 1;

  auto send_state = Parameter("A", value);
  std::string msg = clproto::encode(send_state);
  EXPECT_TRUE(clproto::is_valid(msg));
  EXPECT_TRUE(clproto::check_message_type(msg) == clproto::MessageType::PARAMETER_MESSAGE);
  EXPECT_TRUE(clproto::check_parameter_message_type(msg) == clproto::ParameterMessageType::INT);

  Parameter<int> recv_state("");
  EXPECT_NO_THROW(clproto::decode<Parameter<int>>(msg));
  EXPECT_TRUE(clproto::decode(msg, recv_state));

  EXPECT_STREQ(send_state.get_name().c_str(), recv_state.get_name().c_str());
  EXPECT_EQ(send_state.get_value(), recv_state.get_value());
  EXPECT_EQ(send_state.get_type(), recv_state.get_type());
}

TEST(ParameterProtoTest, EncodeDecodeParameterIntArray) {
  std::vector<int> value = {1, 2, 3};

  auto send_state = Parameter("A", value);
  std::string msg = clproto::encode(send_state);
  EXPECT_TRUE(clproto::is_valid(msg));
  EXPECT_TRUE(clproto::check_message_type(msg) == clproto::MessageType::PARAMETER_MESSAGE);
  EXPECT_TRUE(clproto::check_parameter_message_type(msg) == clproto::ParameterMessageType::INT_ARRAY);

  Parameter<std::vector<int>> recv_state("");
  EXPECT_NO_THROW(clproto::decode<Parameter<std::vector<int>>>(msg));
  EXPECT_TRUE(clproto::decode(msg, recv_state));

  EXPECT_STREQ(send_state.get_name().c_str(), recv_state.get_name().c_str());
  EXPECT_EQ(send_state.get_value().size(), recv_state.get_value().size());
  for(std::size_t index = 0; index < send_state.get_value().size(); ++index) {
    EXPECT_EQ(send_state.get_value().at(index), recv_state.get_value().at(index));
  }
  EXPECT_EQ(send_state.get_type(), recv_state.get_type());
}

TEST(ParameterProtoTest, EncodeDecodeParameterDouble) {
  double value = 1.0;

  auto send_state = Parameter("A", value);
  std::string msg = clproto::encode(send_state);
  EXPECT_TRUE(clproto::is_valid(msg));
  EXPECT_TRUE(clproto::check_message_type(msg) == clproto::MessageType::PARAMETER_MESSAGE);
  EXPECT_TRUE(clproto::check_parameter_message_type(msg) == clproto::ParameterMessageType::DOUBLE);

  Parameter<double> recv_state("");
  EXPECT_NO_THROW(clproto::decode<Parameter<double>>(msg));
  EXPECT_TRUE(clproto::decode(msg, recv_state));

  EXPECT_STREQ(send_state.get_name().c_str(), recv_state.get_name().c_str());
  EXPECT_EQ(send_state.get_value(), recv_state.get_value());
  EXPECT_EQ(send_state.get_type(), recv_state.get_type());
}

TEST(ParameterProtoTest, EncodeDecodeParameterDoubleArray) {
  std::vector<double> value = {1.0, 2.0, 3.0};

  auto send_state = Parameter("A", value);
  std::string msg = clproto::encode(send_state);
  EXPECT_TRUE(clproto::is_valid(msg));
  EXPECT_TRUE(clproto::check_message_type(msg) == clproto::MessageType::PARAMETER_MESSAGE);
  EXPECT_TRUE(clproto::check_parameter_message_type(msg) == clproto::ParameterMessageType::DOUBLE_ARRAY);

  Parameter<std::vector<double>> recv_state("");
  EXPECT_NO_THROW(clproto::decode<Parameter<std::vector<double>>>(msg));
  EXPECT_TRUE(clproto::decode(msg, recv_state));

  EXPECT_STREQ(send_state.get_name().c_str(), recv_state.get_name().c_str());
  EXPECT_EQ(send_state.get_value().size(), recv_state.get_value().size());
  for(std::size_t index = 0; index < send_state.get_value().size(); ++index) {
    EXPECT_EQ(send_state.get_value().at(index), recv_state.get_value().at(index));
  }
  EXPECT_EQ(send_state.get_type(), recv_state.get_type());
}

TEST(ParameterProtoTest, EncodeDecodeParameterBool) {
  bool value = true;

  auto send_state = Parameter("A", value);
  std::string msg = clproto::encode(send_state);
  EXPECT_TRUE(clproto::is_valid(msg));
  EXPECT_TRUE(clproto::check_message_type(msg) == clproto::MessageType::PARAMETER_MESSAGE);
  EXPECT_TRUE(clproto::check_parameter_message_type(msg) == clproto::ParameterMessageType::BOOL);

  Parameter<bool> recv_state("");
  EXPECT_NO_THROW(clproto::decode<Parameter<bool>>(msg));
  EXPECT_TRUE(clproto::decode(msg, recv_state));

  EXPECT_STREQ(send_state.get_name().c_str(), recv_state.get_name().c_str());
  EXPECT_EQ(send_state.get_value(), recv_state.get_value());
  EXPECT_EQ(send_state.get_type(), recv_state.get_type());
}

TEST(ParameterProtoTest, EncodeDecodeParameterBoolArray) {
  std::vector<bool> value = {true, false, true, false};

  auto send_state = Parameter("A", value);
  std::string msg = clproto::encode(send_state);
  EXPECT_TRUE(clproto::is_valid(msg));
  EXPECT_TRUE(clproto::check_message_type(msg) == clproto::MessageType::PARAMETER_MESSAGE);
  EXPECT_TRUE(clproto::check_parameter_message_type(msg) == clproto::ParameterMessageType::BOOL_ARRAY);

  Parameter<std::vector<bool>> recv_state("");
  EXPECT_NO_THROW(clproto::decode<Parameter<std::vector<bool>>>(msg));
  EXPECT_TRUE(clproto::decode(msg, recv_state));

  EXPECT_STREQ(send_state.get_name().c_str(), recv_state.get_name().c_str());
  EXPECT_EQ(send_state.get_value().size(), recv_state.get_value().size());
  for(std::size_t index = 0; index < send_state.get_value().size(); ++index) {
    EXPECT_EQ(send_state.get_value().at(index), recv_state.get_value().at(index));
  }
  EXPECT_EQ(send_state.get_type(), recv_state.get_type());
}

TEST(ParameterProtoTest, EncodeDecodeParameterString) {
  std::string value = "value";

  auto send_state = Parameter("A", value);
  std::string msg = clproto::encode(send_state);
  EXPECT_TRUE(clproto::is_valid(msg));
  EXPECT_TRUE(clproto::check_message_type(msg) == clproto::MessageType::PARAMETER_MESSAGE);
  EXPECT_TRUE(clproto::check_parameter_message_type(msg) == clproto::ParameterMessageType::STRING);

  Parameter<std::string> recv_state("");
  EXPECT_NO_THROW(clproto::decode<Parameter<std::string>>(msg));
  EXPECT_TRUE(clproto::decode(msg, recv_state));

  EXPECT_STREQ(send_state.get_name().c_str(), recv_state.get_name().c_str());
  EXPECT_STREQ(send_state.get_value().c_str(), recv_state.get_value().c_str());
  EXPECT_EQ(send_state.get_type(), recv_state.get_type());
}

TEST(ParameterProtoTest, EncodeDecodeParameterStringArray) {
  std::vector<std::string> value = {"value 1", "value 2", "value 3"};

  auto send_state = Parameter("A", value);
  std::string msg = clproto::encode(send_state);
  EXPECT_TRUE(clproto::is_valid(msg));
  EXPECT_TRUE(clproto::check_message_type(msg) == clproto::MessageType::PARAMETER_MESSAGE);
  EXPECT_TRUE(clproto::check_parameter_message_type(msg) == clproto::ParameterMessageType::STRING_ARRAY);

  Parameter<std::vector<std::string>> recv_state("");
  EXPECT_NO_THROW(clproto::decode<Parameter<std::vector<std::string>>>(msg));
  EXPECT_TRUE(clproto::decode(msg, recv_state));

  EXPECT_STREQ(send_state.get_name().c_str(), recv_state.get_name().c_str());
  EXPECT_EQ(send_state.get_value().size(), recv_state.get_value().size());
  for(std::size_t index = 0; index < send_state.get_value().size(); ++index) {
    EXPECT_STREQ(send_state.get_value().at(index).c_str(), recv_state.get_value().at(index).c_str());
  }
  EXPECT_EQ(send_state.get_type(), recv_state.get_type());
}

TEST(ParameterProtoTest, EncodeDecodeParameterVector) {
  Eigen::VectorXd value(5);
  value << 1.0, 2.0, 3.0, 4.0, 5.0;

  auto send_state = Parameter("A", value);
  std::string msg = clproto::encode(send_state);
  EXPECT_TRUE(clproto::is_valid(msg));
  EXPECT_TRUE(clproto::check_message_type(msg) == clproto::MessageType::PARAMETER_MESSAGE);
  EXPECT_TRUE(clproto::check_parameter_message_type(msg) == clproto::ParameterMessageType::VECTOR);

  Parameter<Eigen::VectorXd> recv_state("");
  EXPECT_NO_THROW(clproto::decode<Parameter<Eigen::VectorXd>>(msg));
  EXPECT_TRUE(clproto::decode(msg, recv_state));

  EXPECT_STREQ(send_state.get_name().c_str(), recv_state.get_name().c_str());
  EXPECT_EQ(send_state.get_value().size(), recv_state.get_value().size());
  for(std::size_t index = 0; index < send_state.get_value().size(); ++index) {
    EXPECT_EQ(send_state.get_value()(index), recv_state.get_value()(index));
  }
  EXPECT_EQ(send_state.get_type(), recv_state.get_type());
}

TEST(ParameterProtoTest, EncodeDecodeParameterMatrix) {
  Eigen::MatrixXd value(2, 3);
  value << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;

  auto send_state = Parameter("A", value);
  std::string msg = clproto::encode(send_state);
  EXPECT_TRUE(clproto::is_valid(msg));
  EXPECT_TRUE(clproto::check_message_type(msg) == clproto::MessageType::PARAMETER_MESSAGE);
  EXPECT_TRUE(clproto::check_parameter_message_type(msg) == clproto::ParameterMessageType::MATRIX);

  Parameter<Eigen::MatrixXd> recv_state("");
  EXPECT_NO_THROW(clproto::decode<Parameter<Eigen::MatrixXd>>(msg));
  EXPECT_TRUE(clproto::decode(msg, recv_state));

  EXPECT_STREQ(send_state.get_name().c_str(), recv_state.get_name().c_str());
  EXPECT_EQ(send_state.get_value().size(), recv_state.get_value().size());
  EXPECT_EQ(send_state.get_value().rows(), recv_state.get_value().rows());
  EXPECT_EQ(send_state.get_value().cols(), recv_state.get_value().cols());
  for(std::size_t index = 0; index < send_state.get_value().size(); ++index) {
    EXPECT_EQ(send_state.get_value()(index), recv_state.get_value()(index));
  }
  EXPECT_EQ(send_state.get_type(), recv_state.get_type());
}

TEST(ParameterProtoTest, EncodeDecodeEmptyParameterDouble) {
  Parameter<double> empty_state("");
  EXPECT_TRUE(empty_state.is_empty());
  std::string msg;
  EXPECT_NO_THROW(msg = clproto::encode(empty_state));

  Parameter<double> recv_state("");
  EXPECT_NO_THROW(recv_state = clproto::decode<Parameter<double>>(msg));
  EXPECT_TRUE(recv_state.is_empty());
}

TEST(ParameterProtoTest, EncodeDecodeEmptyParameterBool) {
  Parameter<bool> empty_state("");
  EXPECT_TRUE(empty_state.is_empty());
  std::string msg;
  EXPECT_NO_THROW(msg = clproto::encode(empty_state));

  Parameter<bool> recv_state("");
  EXPECT_NO_THROW(recv_state = clproto::decode<Parameter<bool>>(msg));
  EXPECT_TRUE(recv_state.is_empty());
}

TEST(ParameterProtoTest, EncodeDecodeEmptyParameterString) {
  Parameter<std::string> empty_state("");
  EXPECT_TRUE(empty_state.is_empty());
  std::string msg;
  EXPECT_NO_THROW(msg = clproto::encode(empty_state));

  Parameter<std::string> recv_state("");
  EXPECT_NO_THROW(recv_state = clproto::decode<Parameter<std::string>>(msg));
  EXPECT_TRUE(recv_state.is_empty());
}