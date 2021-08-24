#include "clproto/decoders.h"

#include "state_representation/state_message.pb.h"

using namespace state_representation;

namespace clproto {

DecoderNotImplementedException::DecoderNotImplementedException(const std::string& msg) : DecodingException(msg) {}

template<>
std::vector<bool> decoder(const google::protobuf::RepeatedField<bool>& message) {
  // explicit construction is needed for the bool vector due to stl optimisations
  std::vector<bool> vec(message.begin(), message.end());
  return vec;
}

template<>
StateType decoder(const proto::StateType& message) {
  return static_cast<StateType>(message);
}

template<>
Eigen::Vector3d decoder(const proto::Vector3d& message) {
  return {message.x(), message.y(), message.z()};
}

template<>
Eigen::Quaterniond decoder(const proto::Quaterniond& message) {
  return {message.w(), message.vec().x(), message.vec().y(), message.vec().z()};
}

template<>
Parameter<double> decoder(const state_representation::proto::Parameter& message) {
  return Parameter<double>(message.state().name(), message.parameter_value().double_().value());
}
template<>
Parameter<std::vector<double>> decoder(const state_representation::proto::Parameter& message) {
  return Parameter<std::vector<double>>(
      message.state().name(), decoder(message.parameter_value().double_array().value()));
}
template<>
Parameter<bool> decoder(const state_representation::proto::Parameter& message) {
  return Parameter<bool>(message.state().name(), message.parameter_value().bool_().value());
}
template<>
Parameter<std::vector<bool>> decoder(const state_representation::proto::Parameter& message) {
  auto val = decoder(message.parameter_value().bool_array().value());
  return Parameter<std::vector<bool>>(message.state().name(), val);
}
template<>
Parameter<std::string> decoder(const state_representation::proto::Parameter& message) {
  return Parameter<std::string>(message.state().name(), message.parameter_value().string().value());
}
template<>
Parameter<std::vector<std::string>> decoder(const state_representation::proto::Parameter& message) {
  return Parameter<std::vector<std::string>>(
      message.state().name(), decoder(message.parameter_value().string_array().value()));
}
template<>
Parameter<Eigen::VectorXd> decoder(const state_representation::proto::Parameter& message) {
  std::vector<double> elements = decoder(message.parameter_value().vector().value());
  return Parameter<Eigen::VectorXd>(
      message.state().name(), Eigen::Map<Eigen::VectorXd>(
          elements.data(), static_cast<Eigen::Index>(elements.size())));
}
template<>
Parameter<Eigen::MatrixXd> decoder(const state_representation::proto::Parameter& message) {
  std::vector<double> elements = decoder(message.parameter_value().matrix().value());
  return Parameter<Eigen::MatrixXd>(
      message.state().name(), Eigen::Map<Eigen::MatrixXd>(
          elements.data(), message.parameter_value().matrix().rows(), message.parameter_value().matrix().cols()));
}
}