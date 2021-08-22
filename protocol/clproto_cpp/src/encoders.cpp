#include "clproto/encoders.h"

#include <state_representation/State.hpp>
#include <state_representation/space/SpatialState.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/robot/Jacobian.hpp>
#include <state_representation/robot/JointState.hpp>

#include "state_representation/state.pb.h"
#include "state_representation/space/spatial_state.pb.h"
#include "state_representation/space/cartesian/cartesian_state.pb.h"
#include "state_representation/space/joint/jacobian.pb.h"
#include "state_representation/space/joint/joint_state.pb.h"

using namespace state_representation;

namespace clproto {

EncoderNotImplementedException::EncoderNotImplementedException(const std::string& msg) : std::runtime_error(msg) {}

template<>
proto::StateType encoder(const StateType& type) {
  switch (type) {
    case state_representation::StateType::STATE:
      return proto::StateType::STATE;
    case state_representation::StateType::CARTESIANSTATE:
      return proto::StateType::CARTESIANSTATE;
    case state_representation::StateType::DUALQUATERNIONSTATE:
      return proto::StateType::DUALQUATERNIONSTATE;
    case state_representation::StateType::JOINTSTATE:
      return proto::StateType::JOINTSTATE;
    case state_representation::StateType::JACOBIANMATRIX:
      return proto::StateType::JACOBIANMATRIX;
    case state_representation::StateType::TRAJECTORY:
      return proto::StateType::TRAJECTORY;
    case state_representation::StateType::GEOMETRY_SHAPE:
      return proto::StateType::GEOMETRY_SHAPE;
    case state_representation::StateType::GEOMETRY_ELLIPSOID:
      return proto::StateType::GEOMETRY_ELLIPSOID;
    default:
      return proto::StateType::STATE;
  }
}

template<>
proto::State encoder(const State& state) {
  proto::State message;
  message.set_name(state.get_name());
  message.set_type(encoder<proto::StateType>(state.get_type()));
  message.set_empty(state.is_empty());
  message.set_timestamp(state.get_timestamp().time_since_epoch().count());
  return message;
}

template<>
proto::SpatialState encoder(const SpatialState& spatial_state) {
  proto::SpatialState message;
  *message.mutable_state() = encoder<proto::State>(static_cast<State>(spatial_state));
  message.set_reference_frame(spatial_state.get_reference_frame());
  return message;
}

template<>
proto::Vector3d encoder(const Eigen::Vector3d& vector) {
  proto::Vector3d message;
  message.set_x(vector.x());
  message.set_y(vector.y());
  message.set_z(vector.z());
  return message;
}

template<>
proto::Quaterniond encoder(const Eigen::Quaterniond& quaternion) {
  proto::Quaterniond message;
  message.set_w(quaternion.w());
  *message.mutable_vec() = encoder<proto::Vector3d>(Eigen::Vector3d(quaternion.vec()));
  return message;
}

template<>
google::protobuf::RepeatedField<double> encoder(const double* data, std::size_t size) {
  std::vector<double> a(data, data + size);
  return google::protobuf::RepeatedField<double>({a.begin(), a.end()});
}

template<>
proto::CartesianState encoder(const CartesianState& cartesian_state) {
  proto::CartesianState message;
  *message.mutable_spatial_state() = encoder<proto::SpatialState>(static_cast<SpatialState>(cartesian_state));
  *message.mutable_position() = encoder<proto::Vector3d>(cartesian_state.get_position());
  *message.mutable_orientation() = encoder<proto::Quaterniond>(cartesian_state.get_orientation());
  *message.mutable_linear_velocity() = encoder<proto::Vector3d>(cartesian_state.get_linear_velocity());
  *message.mutable_angular_velocity() = encoder<proto::Vector3d>(cartesian_state.get_angular_velocity());
  *message.mutable_linear_acceleration() = encoder<proto::Vector3d>(cartesian_state.get_linear_acceleration());
  *message.mutable_angular_acceleration() = encoder<proto::Vector3d>(cartesian_state.get_angular_acceleration());
  *message.mutable_force() = encoder<proto::Vector3d>(cartesian_state.get_force());
  *message.mutable_torque() = encoder<proto::Vector3d>(cartesian_state.get_torque());
  return message;
}

template<>
proto::Jacobian encoder(const Jacobian& jacobian) {
  proto::Jacobian message;
  *message.mutable_state() = encoder<proto::State>(static_cast<State>(jacobian));
  *message.mutable_joint_names() = {jacobian.get_joint_names().begin(), jacobian.get_joint_names().end()};
  message.set_frame(jacobian.get_frame());
  message.set_reference_frame(jacobian.get_reference_frame());
  message.set_rows(jacobian.rows());
  message.set_cols(jacobian.cols());
  *message.mutable_data() = encoder(jacobian.data().data(), jacobian.data().size());
  return message;
}

template<>
proto::JointState encoder(const JointState& joint_state) {
  proto::JointState message;
  *message.mutable_state() = encoder<proto::State>(static_cast<State>(joint_state));
  *message.mutable_joint_names() = {joint_state.get_names().begin(), joint_state.get_names().end()};
  *message.mutable_positions() = encoder(joint_state.get_positions().data(), joint_state.get_positions().size());
  *message.mutable_velocities() = encoder(joint_state.get_velocities().data(), joint_state.get_velocities().size());
  *message.mutable_accelerations() =
      encoder(joint_state.get_accelerations().data(), joint_state.get_accelerations().size());
  *message.mutable_torques() = encoder(joint_state.get_torques().data(), joint_state.get_torques().size());
  return message;
}

}