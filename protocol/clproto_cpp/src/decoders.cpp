#include "clproto/decoders.h"

#include <state_representation/State.hpp>

#include "state_representation/state.pb.h"
#include "state_representation/space/cartesian/cartesian_state.pb.h"
#include "state_representation/space/cartesian/cartesian_state.pb.h"

using namespace state_representation;

namespace clproto {

DecoderNotImplementedException::DecoderNotImplementedException(const std::string& msg) : DecodingException(msg) {}

template<>
StateType decoder(const proto::StateType& message) {
  return static_cast<StateType>(message);
}

template<>
Eigen::Vector3d decoder(const proto::Vector3d& message) {
  return Eigen::Vector3d(message.x(), message.y(), message.z());
}

template<>
Eigen::Quaterniond decoder(const proto::Quaterniond& message) {
  return Eigen::Quaterniond(message.w(), message.vec().x(), message.vec().y(), message.vec().z());
}

template<>
std::vector<double> decoder(const google::protobuf::RepeatedField<double>& message) {
  return std::vector<double>(message.begin(), message.end());
}

}