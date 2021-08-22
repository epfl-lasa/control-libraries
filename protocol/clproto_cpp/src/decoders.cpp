#include "clproto/decoders.h"

using namespace state_representation;

namespace clproto {

state_representation::StateType decoder(const proto::StateType& message) {
  switch (message) {
    case proto::StateType::STATE:
      return state_representation::StateType::STATE;
    case proto::StateType::CARTESIANSTATE:
      return state_representation::StateType::CARTESIANSTATE;
    case proto::StateType::DUALQUATERNIONSTATE:
      return state_representation::StateType::DUALQUATERNIONSTATE;
    case proto::StateType::JOINTSTATE:
      return state_representation::StateType::JOINTSTATE;
    case proto::StateType::JACOBIANMATRIX:
      return state_representation::StateType::JACOBIANMATRIX;
    case proto::StateType::TRAJECTORY:
      return state_representation::StateType::TRAJECTORY;
    case proto::StateType::GEOMETRY_SHAPE:
      return state_representation::StateType::GEOMETRY_SHAPE;
    case proto::StateType::GEOMETRY_ELLIPSOID:
      return state_representation::StateType::GEOMETRY_ELLIPSOID;
    default:
      return state_representation::StateType::STATE;
  }
}

Eigen::Vector3d decoder(const proto::Vector3d& message) {
  return Eigen::Vector3d(message.x(), message.y(), message.z());
}

Eigen::Quaterniond decoder(const proto::Quaterniond& message) {
  return Eigen::Quaterniond(message.w(), message.vec().x(), message.vec().y(), message.vec().z());
}

std::vector<double> decoder(const google::protobuf::RepeatedField<double>& message) {
  return std::vector<double>(message.begin(), message.end());
}

}