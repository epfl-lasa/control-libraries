#include "clproto/encoders.h"

#include <state_representation/State.hpp>
#include <state_representation/space/SpatialState.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/robot/Jacobian.hpp>
#include <state_representation/robot/JointState.hpp>

#include "state_representation/state_message.pb.h"

using namespace state_representation;

namespace clproto {

EncoderNotImplementedException::EncoderNotImplementedException(const std::string& msg) : std::runtime_error(msg) {}

google::protobuf::RepeatedField<double> encoder(const Eigen::MatrixXd& matrix) {
  return encoder(std::vector<double>{matrix.data(), matrix.data() + matrix.size()});
}

template<>
proto::StateType encoder(const StateType& type) {
  return static_cast<proto::StateType>(type);
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
  *message.mutable_data() = encoder(jacobian.data());
  return message;
}

template<>
proto::JointState encoder(const JointState& joint_state) {
  proto::JointState message;
  *message.mutable_state() = encoder<proto::State>(static_cast<State>(joint_state));
  *message.mutable_joint_names() = {joint_state.get_names().begin(), joint_state.get_names().end()};
  *message.mutable_positions() = encoder(joint_state.get_positions());
  *message.mutable_velocities() = encoder(joint_state.get_velocities());
  *message.mutable_accelerations() = encoder(joint_state.get_accelerations());
  *message.mutable_torques() = encoder(joint_state.get_torques());
  return message;
}

template<>
state_representation::proto::Parameter encoder(const state_representation::Parameter<double>& parameter) {
  state_representation::proto::Parameter message;
  *message.mutable_state() =
      encoder<state_representation::proto::State>(static_cast<state_representation::State>(parameter));
  message.mutable_parameter_value()->mutable_double_()->set_value(parameter.get_value());
  return message;
}
template<>
state_representation::proto::Parameter encoder(const state_representation::Parameter<std::vector<double>>& parameter) {
  state_representation::proto::Parameter message;
  *message.mutable_state() =
      encoder<state_representation::proto::State>(static_cast<state_representation::State>(parameter));
  *message.mutable_parameter_value()->mutable_double_array()->mutable_value() =
      {parameter.get_value().begin(), parameter.get_value().end()};
  return message;
}
template<>
state_representation::proto::Parameter encoder(const state_representation::Parameter<bool>& parameter) {
  state_representation::proto::Parameter message;
  *message.mutable_state() =
      encoder<state_representation::proto::State>(static_cast<state_representation::State>(parameter));
  message.mutable_parameter_value()->mutable_bool_()->set_value(parameter.get_value());
  return message;
}
template<>
state_representation::proto::Parameter encoder(const state_representation::Parameter<std::vector<bool>>& parameter) {
  state_representation::proto::Parameter message;
  *message.mutable_state() =
      encoder<state_representation::proto::State>(static_cast<state_representation::State>(parameter));
  *message.mutable_parameter_value()->mutable_bool_array()->mutable_value() =
      {parameter.get_value().begin(), parameter.get_value().end()};
  return message;
}
template<>
state_representation::proto::Parameter encoder(const state_representation::Parameter<std::string>& parameter) {
  state_representation::proto::Parameter message;
  *message.mutable_state() =
      encoder<state_representation::proto::State>(static_cast<state_representation::State>(parameter));
  message.mutable_parameter_value()->mutable_string()->set_value(parameter.get_value());
  return message;
}
template<>
state_representation::proto::Parameter
encoder(const state_representation::Parameter<std::vector<std::string>>& parameter) {
  state_representation::proto::Parameter message;
  *message.mutable_state() =
      encoder<state_representation::proto::State>(static_cast<state_representation::State>(parameter));
  *message.mutable_parameter_value()->mutable_string_array()->mutable_value() =
      {parameter.get_value().begin(), parameter.get_value().end()};
  return message;
}
template<>
state_representation::proto::Parameter encoder(const state_representation::Parameter<Eigen::VectorXd>& parameter) {
  state_representation::proto::Parameter message;
  *message.mutable_state() =
      encoder<state_representation::proto::State>(static_cast<state_representation::State>(parameter));
  *message.mutable_parameter_value()->mutable_vector()->mutable_value() = encoder(parameter.get_value());
  return message;
}

template<>
state_representation::proto::Parameter encoder(const state_representation::Parameter<Eigen::MatrixXd>& parameter) {
  state_representation::proto::Parameter message;
  *message.mutable_state() =
      encoder<state_representation::proto::State>(static_cast<state_representation::State>(parameter));
  *message.mutable_parameter_value()->mutable_matrix()->mutable_value() = encoder(parameter.get_value());
  message.mutable_parameter_value()->mutable_matrix()->set_rows(parameter.get_value().rows());
  message.mutable_parameter_value()->mutable_matrix()->set_cols(parameter.get_value().cols());
  return message;
}
}