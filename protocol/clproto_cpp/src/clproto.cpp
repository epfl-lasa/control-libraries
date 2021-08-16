#include "clproto.h"

#include <state_representation/State.hpp>
#include <state_representation/space/SpatialState.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <state_representation/space/cartesian/CartesianTwist.hpp>
#include <state_representation/space/cartesian/CartesianWrench.hpp>
#include <state_representation/robot/Jacobian.hpp>
#include <state_representation/robot/JointState.hpp>
#include <state_representation/robot/JointPositions.hpp>
#include <state_representation/robot/JointVelocities.hpp>
#include <state_representation/robot/JointAccelerations.hpp>
#include <state_representation/robot/JointTorques.hpp>
#include <state_representation/geometry/Shape.hpp>
#include <state_representation/geometry/Ellipsoid.hpp>

#include "state_representation/state_message.pb.h"
#include "state_representation/state.pb.h"
#include "state_representation/space/spatial_state.pb.h"
#include "state_representation/space/cartesian/cartesian_state.pb.h"
#include "state_representation/space/joint/jacobian.pb.h"
#include "state_representation/space/joint/joint_state.pb.h"
#include "state_representation/geometry/shape.pb.h"
#include "state_representation/geometry/ellipsoid.pb.h"

using namespace state_representation;

namespace clproto {

DecodingException::DecodingException(const std::string& msg) : std::runtime_error(msg) {}

bool is_valid(const std::string& msg) {
  return check_message_type(msg) != MessageType::UNKNOWN_MESSAGE;
}

MessageType check_message_type(const std::string& msg) {
  if (proto::StateMessage message; message.ParseFromString(msg)) {
    return static_cast<MessageType>(message.message_type_case());
  }

  /* In theory, sending "raw" messages (message types without the
   * StateMessage wrapper) could also be supported, though it would
   * require manually checking each case as in the following snippet:

  if (proto::State message; message.ParseFromString(msg)) {
    return MessageType::STATE_MESSAGE;
  } else if (proto::SpatialState message; message.ParseFromString(msg)) {
    return MessageType::SPATIAL_STATE_MESSAGE;
  }

   * Because the intention is to encode / decode messages using
   * this library, and all encoded messages use the StateMessage
   * wrapper, raw messages are treated as UNKNOWN at this time.
   */

  return MessageType::UNKNOWN_MESSAGE;
}

/**
 * @brief Local encoding helper for StateType
 * @param type The state representation StateType
 * @return The equivalent proto StateType
 */
static proto::StateType encoder(const StateType& type) {
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

/**
 * @brief Local encoding helper for State
 * @param type The state representation State
 * @return The equivalent proto State
 */
static proto::State encoder(const State& state) {
  proto::State message;
  message.set_name(state.get_name());
  message.set_type(encoder(state.get_type()));
  message.set_empty(state.is_empty());
  message.set_timestamp(state.get_timestamp().time_since_epoch().count());
  return message;
}

/**
 * @brief Local encoding helper for SpatialState
 * @param type The state representation SpatialState
 * @return The equivalent proto SpatialState
 */
static proto::SpatialState encoder(const SpatialState& spatial_state) {
  proto::SpatialState message;
  *message.mutable_state() = encoder(static_cast<State>(spatial_state));
  message.set_reference_frame(spatial_state.get_reference_frame());
  return message;
}

/**
 * @brief Local encoding helper for Vector3d
 * @param type The Eigen Vector3d
 * @return The equivalent proto Vector3d
 */
static proto::Vector3d encoder(const Eigen::Vector3d& vector) {
  proto::Vector3d message;
  message.set_x(vector.x());
  message.set_y(vector.y());
  message.set_z(vector.z());
  return message;
}

/**
 * @brief Local encoding helper for Quaterniond
 * @param type The Eigen Quaterniond
 * @return The equivalent proto Quaterniond
 */
static proto::Quaterniond encoder(const Eigen::Quaterniond& quaternion) {
  proto::Quaterniond message;
  message.set_w(quaternion.w());
  *message.mutable_vec() = encoder(Eigen::Vector3d(quaternion.vec()));
  return message;
}

/**
 * Local encoding helper for generic Eigen data
 * @param data The data buffer as type double
 * @param size The size of the data buffer
 * @return A protobuf RepeatedField<double> type
 */
static google::protobuf::RepeatedField<double> encoder(const double* data, Eigen::Index size) {
  std::vector<double> a(data, data + size);
  return google::protobuf::RepeatedField<double>({a.begin(), a.end()});
}

/**
 * @brief Local encoding helper for CartesianState
 * @param type The state representation CartesianState
 * @return The equivalent proto CartesianState
 */
static proto::CartesianState encoder(const CartesianState& cartesian_state) {
  proto::CartesianState message;
  *message.mutable_spatial_state() = encoder(static_cast<SpatialState>(cartesian_state));
  *message.mutable_position() = encoder(cartesian_state.get_position());
  *message.mutable_orientation() = encoder(cartesian_state.get_orientation());
  *message.mutable_linear_velocity() = encoder(cartesian_state.get_linear_velocity());
  *message.mutable_angular_velocity() = encoder(cartesian_state.get_angular_velocity());
  *message.mutable_linear_acceleration() = encoder(cartesian_state.get_linear_acceleration());
  *message.mutable_angular_acceleration() = encoder(cartesian_state.get_angular_acceleration());
  *message.mutable_force() = encoder(cartesian_state.get_force());
  *message.mutable_torque() = encoder(cartesian_state.get_torque());
  return message;
}

/**
 * @brief Local encoding helper for Jacobian
 * @param type The state representation Jacobian
 * @return The equivalent proto Jacobian
 */
static proto::Jacobian encoder(const Jacobian& jacobian) {
  proto::Jacobian message;
  *message.mutable_state() = encoder(static_cast<State>(jacobian));
  *message.mutable_joint_names() = {jacobian.get_joint_names().begin(), jacobian.get_joint_names().end()};
  message.set_frame(jacobian.get_frame());
  message.set_reference_frame(jacobian.get_reference_frame());
  message.set_rows(jacobian.rows());
  message.set_cols(jacobian.cols());
  *message.mutable_data() = encoder(jacobian.data().data(), jacobian.data().size());
  return message;
}

/**
 * @brief Local encoding helper for CartesianState
 * @param type The state representation CartesianState
 * @return The equivalent proto CartesianState
 */
static proto::JointState encoder(const JointState& joint_state) {
  proto::JointState message;
  *message.mutable_state() = encoder(static_cast<State>(joint_state));
  *message.mutable_joint_names() = {joint_state.get_names().begin(), joint_state.get_names().end()};
  *message.mutable_positions() = encoder(joint_state.get_positions().data(), joint_state.get_positions().size());
  *message.mutable_velocities() = encoder(joint_state.get_velocities().data(), joint_state.get_velocities().size());
  *message.mutable_accelerations() =
      encoder(joint_state.get_accelerations().data(), joint_state.get_accelerations().size());
  *message.mutable_torques() = encoder(joint_state.get_torques().data(), joint_state.get_torques().size());
  return message;
}

/**
 * @brief Local decoding helper for StateType
 * @param type The state representation StateType
 * @return The equivalent proto StateType
 */
static state_representation::StateType decoder(const proto::StateType& message) {
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

/**
 * @brief Local decoding helper for Vector3d
 * @param type The proto Vector3d
 * @return The equivalent Eigen Vector3d
 */
static Eigen::Vector3d decoder(const proto::Vector3d& message) {
  return Eigen::Vector3d(message.x(), message.y(), message.z());
}

/**
 * @brief Local decoding helper for Quaterniond
 * @param type The proto Quaterniond
 * @return The equivalent Eigen Quaterniond
 */
static Eigen::Quaterniond decoder(const proto::Quaterniond& message) {
  return Eigen::Quaterniond(message.w(), message.vec().x(), message.vec().y(), message.vec().z());
}

/**
 * @brief Local decoding helper for repeated double data
 * @param type The protobuf RepeatedField<double> data
 * @return The equivalent STL vector
 */
static std::vector<double> decoder(const google::protobuf::RepeatedField<double>& message) {
  return std::vector<double>(message.begin(), message.end());
}

template<>
std::string encode<State>(const State& obj);
template<>
State decode(const std::string& msg);
template<>
bool decode(const std::string& msg, State& obj);
template<>
std::string encode<State>(const State& obj) {
  proto::StateMessage message;
  *message.mutable_state() = encoder(obj);
  return message.SerializeAsString();
}
template<>
State decode(const std::string& msg) {
  State obj;
  if (!decode(msg, obj)) {
    throw DecodingException("Could not decode the message into a State");
  }
  return obj;
}
template<>
bool decode(const std::string& msg, State& obj) {
  try {
    proto::StateMessage message;
    if (!(message.ParseFromString(msg) && message.message_type_case() == proto::StateMessage::MessageTypeCase::kState)) {
      if (!message.mutable_state()->ParseFromString(msg)) {
        return false;
      }
    }

    auto state = message.state();
    obj = State(decoder(state.type()), state.name(), state.empty());

    //TODO: (maybe) add set_timestamp method to State and add decoder for int to chrono
    //obj.set_timestamp(state.timestamp());

    return true;
  } catch (...) {
    return false;
  }
}

template<>
std::string encode<SpatialState>(const SpatialState& obj);
template<>
SpatialState decode(const std::string& msg);
template<>
bool decode(const std::string& msg, SpatialState& obj);
template<>
std::string encode<SpatialState>(const SpatialState& obj) {
  proto::StateMessage message;
  *message.mutable_spatial_state() = encoder(obj);
  return message.SerializeAsString();
}
template<>
SpatialState decode(const std::string& msg) {
  SpatialState obj(StateType::STATE);
  if (!decode(msg, obj)) {
    throw DecodingException("Could not decode the message into a SpatialState");
  }
  return obj;
}
template<>
bool decode(const std::string& msg, SpatialState& obj) {
  try {
    proto::StateMessage message;
    if (!(message.ParseFromString(msg)
    && message.message_type_case() == proto::StateMessage::MessageTypeCase::kSpatialState)) {
      if (!message.mutable_spatial_state()->ParseFromString(msg)) {
        return false;
      }
    }

    auto spatial_state = message.spatial_state();
    obj = SpatialState(decoder(spatial_state.state().type()),
                       spatial_state.state().name(),
                       spatial_state.reference_frame(),
                       spatial_state.state().empty());

    return true;
  } catch (...) {
    return false;
  }
}

template<>
std::string encode<CartesianState>(const CartesianState& obj);
template<>
CartesianState decode(const std::string& msg);
template<>
bool decode(const std::string& msg, CartesianState& obj);
template<>
std::string encode<CartesianState>(const CartesianState& obj) {
  proto::StateMessage message;
  *message.mutable_cartesian_state() = encoder(obj);
  return message.SerializeAsString();
}
template<>
CartesianState decode(const std::string& msg) {
  CartesianState obj;
  if (!decode(msg, obj)) {
    throw DecodingException("Could not decode the message into a CartesianState");
  }
  return obj;
}
template<>
bool decode(const std::string& msg, CartesianState& obj) {
  try {
    proto::StateMessage message;
    if (!(message.ParseFromString(msg)
        && message.message_type_case() == proto::StateMessage::MessageTypeCase::kCartesianState)) {
      if (!message.mutable_cartesian_state()->ParseFromString(msg)) {
        return false;
      }
    }

    auto state = message.cartesian_state();
    obj.set_name(state.spatial_state().state().name());
    obj.set_reference_frame(state.spatial_state().reference_frame());
    obj.set_position(decoder(state.position()));
    obj.set_orientation(decoder(state.orientation()));
    obj.set_linear_velocity(decoder(state.linear_velocity()));
    obj.set_angular_velocity(decoder(state.angular_velocity()));
    obj.set_linear_acceleration(decoder(state.linear_acceleration()));
    obj.set_angular_acceleration(decoder(state.angular_acceleration()));
    obj.set_force(decoder(state.force()));
    obj.set_torque(decoder(state.torque()));
    obj.set_empty(state.spatial_state().state().empty());
    return true;
  } catch (...) {
    return false;
  }
}

template<>
std::string encode<CartesianPose>(const CartesianPose& obj);
template<>
CartesianPose decode(const std::string& msg);
template<>
bool decode(const std::string& msg, CartesianPose& obj);
template<>
std::string encode<CartesianPose>(const CartesianPose& obj) {
  proto::StateMessage message;
  proto::CartesianState cartesian_state = encoder(static_cast<CartesianState>(obj));
  *message.mutable_cartesian_pose()->mutable_spatial_state() = cartesian_state.spatial_state();
  *message.mutable_cartesian_pose()->mutable_position() = cartesian_state.position();
  *message.mutable_cartesian_pose()->mutable_orientation() = cartesian_state.orientation();
  return message.SerializeAsString();
}
template<>
CartesianPose decode(const std::string& msg) {
  CartesianPose obj;
  if (!decode(msg, obj)) {
    throw DecodingException("Could not decode the message into a CartesianPose");
  }
  return obj;
}
template<>
bool decode(const std::string& msg, CartesianPose& obj) {
  try {
    proto::StateMessage message;
    if (!(message.ParseFromString(msg)
    && message.message_type_case() == proto::StateMessage::MessageTypeCase::kCartesianPose)) {
      if (!message.mutable_cartesian_pose()->ParseFromString(msg)) {
        return false;
      }
    }
    auto pose = message.cartesian_pose();
    obj.set_name(pose.spatial_state().state().name());
    obj.set_reference_frame(pose.spatial_state().reference_frame());
    obj.set_position(decoder(pose.position()));
    obj.set_orientation(decoder(pose.orientation()));
    obj.set_empty(pose.spatial_state().state().empty());
    return true;
  } catch (...) {
    return false;
  }
}

template<>
std::string encode<CartesianTwist>(const CartesianTwist& obj);
template<>
CartesianTwist decode(const std::string& msg);
template<>
bool decode(const std::string& msg, CartesianTwist& obj);
template<>
std::string encode<CartesianTwist>(const CartesianTwist& obj) {
  proto::StateMessage message;
  proto::CartesianState cartesian_state = encoder(static_cast<CartesianState>(obj));
  *message.mutable_cartesian_twist()->mutable_spatial_state() = cartesian_state.spatial_state();
  *message.mutable_cartesian_twist()->mutable_linear_velocity() = cartesian_state.linear_velocity();
  *message.mutable_cartesian_twist()->mutable_angular_velocity() = cartesian_state.angular_velocity();
  return message.SerializeAsString();
}
template<>
CartesianTwist decode(const std::string& msg) {
  CartesianTwist obj;
  if (!decode(msg, obj)) {
    throw DecodingException("Could not decode the message into a CartesianTwist");
  }
  return obj;
}
template<>
bool decode(const std::string& msg, CartesianTwist& obj) {
  try {
    proto::StateMessage message;
    if (!(message.ParseFromString(msg)
    && message.message_type_case() == proto::StateMessage::MessageTypeCase::kCartesianTwist)) {
      if (!message.mutable_cartesian_twist()->ParseFromString(msg)) {
        return false;
      }
    }
    auto twist = message.cartesian_twist();
    obj.set_name(twist.spatial_state().state().name());
    obj.set_reference_frame(twist.spatial_state().reference_frame());
    obj.set_linear_velocity(decoder(twist.linear_velocity()));
    obj.set_angular_velocity(decoder(twist.angular_velocity()));
    obj.set_empty(twist.spatial_state().state().empty());
    return true;
  } catch (...) {
    return false;
  }
}

template<>
std::string encode<CartesianWrench>(const CartesianWrench& obj);
template<>
CartesianWrench decode(const std::string& msg);
template<>
bool decode(const std::string& msg, CartesianWrench& obj);
template<>
std::string encode<CartesianWrench>(const CartesianWrench& obj) {
  proto::StateMessage message;
  proto::CartesianState cartesian_state = encoder(static_cast<CartesianState>(obj));
  *message.mutable_cartesian_wrench()->mutable_spatial_state() = cartesian_state.spatial_state();
  *message.mutable_cartesian_wrench()->mutable_force() = cartesian_state.force();
  *message.mutable_cartesian_wrench()->mutable_torque() = cartesian_state.torque();
  return message.SerializeAsString();
}
template<>
CartesianWrench decode(const std::string& msg) {
  CartesianWrench obj;
  if (!decode(msg, obj)) {
    throw DecodingException("Could not decode the message into a CartesianWrench");
  }
  return obj;
}
template<>
bool decode(const std::string& msg, CartesianWrench& obj) {
  try {
    proto::StateMessage message;
    if (!(message.ParseFromString(msg)
    && message.message_type_case() == proto::StateMessage::MessageTypeCase::kCartesianWrench)) {
      if (!message.mutable_cartesian_wrench()->ParseFromString(msg)) {
        return false;
      }
    }
    auto wrench = message.cartesian_wrench();
    obj.set_name(wrench.spatial_state().state().name());
    obj.set_reference_frame(wrench.spatial_state().reference_frame());
    obj.set_force(decoder(wrench.force()));
    obj.set_torque(decoder(wrench.torque()));
    obj.set_empty(wrench.spatial_state().state().empty());
    return true;
  } catch (...) {
    return false;
  }
}

template<>
std::string encode<Jacobian>(const Jacobian& obj);
template<>
Jacobian decode(const std::string& msg);
template<>
bool decode(const std::string& msg, Jacobian& obj);
template<>
std::string encode<Jacobian>(const Jacobian& obj) {
  proto::StateMessage message;
  *message.mutable_jacobian() = encoder(obj);
  return message.SerializeAsString();
}
template<>
Jacobian decode(const std::string& msg) {
  Jacobian obj;
  if (!decode(msg, obj)) {
    throw DecodingException("Could not decode the message into a Jacobian");
  }
  return obj;
}
template<>
bool decode(const std::string& msg, Jacobian& obj) {
  try {
    proto::StateMessage message;
    if (!(message.ParseFromString(msg)
    && message.message_type_case() == proto::StateMessage::MessageTypeCase::kJacobian)) {
      if (!message.mutable_jacobian()->ParseFromString(msg)) {
        return false;
      }
    }

    auto jacobian = message.jacobian();
    auto raw_data = const_cast<double*>(jacobian.data().data());
    auto data = Eigen::Map<Eigen::MatrixXd>(raw_data, jacobian.rows(), jacobian.cols());
    obj = Jacobian(jacobian.state().name(),
                   {jacobian.joint_names().begin(), jacobian.joint_names().end()},
                   jacobian.frame(),
                   data,
                   jacobian.reference_frame());
    return true;
  } catch (...) {
    return false;
  }
}

template<>
std::string encode<JointState>(const JointState& obj);
template<>
JointState decode(const std::string& msg);
template<>
bool decode(const std::string& msg, JointState& obj);
template<>
std::string encode<JointState>(const JointState& obj) {
  proto::StateMessage message;
  *message.mutable_joint_state() = encoder(obj);
  return message.SerializeAsString();
}
template<>
JointState decode(const std::string& msg) {
  JointState obj;
  if (!decode(msg, obj)) {
    throw DecodingException("Could not decode the message into a JointState");
  }
  return obj;
}
template<>
bool decode(const std::string& msg, JointState& obj) {
  try {
    proto::StateMessage message;
    if (!(message.ParseFromString(msg)
    && message.message_type_case() == proto::StateMessage::MessageTypeCase::kJointState)) {
      if (!message.mutable_joint_state()->ParseFromString(msg)) {
        return false;
      }
    }

    auto state = message.joint_state();
    obj = JointState(state.state().name(), {state.joint_names().begin(), state.joint_names().end()});
    obj.set_positions(decoder(state.positions()));
    obj.set_velocities(decoder(state.velocities()));
    obj.set_accelerations(decoder(state.accelerations()));
    obj.set_torques(decoder(state.torques()));
    obj.set_empty(state.state().empty());
    return true;
  } catch (...) {
    return false;
  }
}

template<>
std::string encode<JointPositions>(const JointPositions& obj);
template<>
JointPositions decode(const std::string& msg);
template<>
bool decode(const std::string& msg, JointPositions& obj);
template<>
std::string encode<JointPositions>(const JointPositions& obj) {
  proto::StateMessage message;
  proto::JointState joint_state = encoder(static_cast<JointState>(obj));
  *message.mutable_joint_positions()->mutable_state() = joint_state.state();
  *message.mutable_joint_positions()->mutable_joint_names() = joint_state.joint_names();
  *message.mutable_joint_positions()->mutable_positions() = joint_state.positions();
  return message.SerializeAsString();
}
template<>
JointPositions decode(const std::string& msg) {
  JointPositions obj;
  if (!decode(msg, obj)) {
    throw DecodingException("Could not decode the message into a JointPositions");
  }
  return obj;
}
template<>
bool decode(const std::string& msg, JointPositions& obj) {
  try {
    proto::StateMessage message;
    if (!(message.ParseFromString(msg)
    && message.message_type_case() == proto::StateMessage::MessageTypeCase::kJointPositions)) {
      if (!message.mutable_joint_positions()->ParseFromString(msg)) {
        return false;
      }
    }

    auto state = message.joint_positions();
    obj = JointState(state.state().name(), {state.joint_names().begin(), state.joint_names().end()});
    obj.set_positions(decoder(state.positions()));
    obj.set_empty(state.state().empty());
    return true;
  } catch (...) {
    return false;
  }
}

template<>
std::string encode<JointVelocities>(const JointVelocities& obj);
template<>
JointVelocities decode(const std::string& msg);
template<>
bool decode(const std::string& msg, JointVelocities& obj);
template<>
std::string encode<JointVelocities>(const JointVelocities& obj) {
  proto::StateMessage message;
  proto::JointState joint_state = encoder(static_cast<JointState>(obj));
  *message.mutable_joint_velocities()->mutable_state() = joint_state.state();
  *message.mutable_joint_velocities()->mutable_joint_names() = joint_state.joint_names();
  *message.mutable_joint_velocities()->mutable_velocities() = joint_state.velocities();
  return message.SerializeAsString();
}
template<>
JointVelocities decode(const std::string& msg) {
  JointVelocities obj;
  if (!decode(msg, obj)) {
    throw DecodingException("Could not decode the message into a JointVelocities");
  }
  return obj;
}
template<>
bool decode(const std::string& msg, JointVelocities& obj) {
  try {
    proto::StateMessage message;
    if (!(message.ParseFromString(msg)
    && message.message_type_case() == proto::StateMessage::MessageTypeCase::kJointVelocities)) {
      if (!message.mutable_joint_velocities()->ParseFromString(msg)) {
        return false;
      }
    }

    auto state = message.joint_velocities();
    obj = JointState(state.state().name(), {state.joint_names().begin(), state.joint_names().end()});
    obj.set_velocities(decoder(state.velocities()));
    obj.set_empty(state.state().empty());
    return true;
  } catch (...) {
    return false;
  }
}

template<>
std::string encode<JointAccelerations>(const JointAccelerations& obj);
template<>
JointAccelerations decode(const std::string& msg);
template<>
bool decode(const std::string& msg, JointAccelerations& obj);
template<>
std::string encode<JointAccelerations>(const JointAccelerations& obj) {
  proto::StateMessage message;
  proto::JointState joint_state = encoder(static_cast<JointState>(obj));
  *message.mutable_joint_accelerations()->mutable_state() = joint_state.state();
  *message.mutable_joint_accelerations()->mutable_joint_names() = joint_state.joint_names();
  *message.mutable_joint_accelerations()->mutable_accelerations() = joint_state.accelerations();
  return message.SerializeAsString();
}
template<>
JointAccelerations decode(const std::string& msg) {
  JointAccelerations obj;
  if (!decode(msg, obj)) {
    throw DecodingException("Could not decode the message into a JointAccelerations");
  }
  return obj;
}
template<>
bool decode(const std::string& msg, JointAccelerations& obj) {
  try {
    proto::StateMessage message;
    if (!(message.ParseFromString(msg)
    && message.message_type_case() == proto::StateMessage::MessageTypeCase::kJointAccelerations)) {
      if (!message.mutable_joint_accelerations()->ParseFromString(msg)) {
        return false;
      }
    }

    auto state = message.joint_accelerations();
    obj = JointState(state.state().name(), {state.joint_names().begin(), state.joint_names().end()});
    obj.set_accelerations(decoder(state.accelerations()));
    obj.set_empty(state.state().empty());
    return true;
  } catch (...) {
    return false;
  }
}

template<>
std::string encode<JointTorques>(const JointTorques& obj);
template<>
JointTorques decode(const std::string& msg);
template<>
bool decode(const std::string& msg, JointTorques& obj);
template<>
std::string encode<JointTorques>(const JointTorques& obj) {
  proto::StateMessage message;
  proto::JointState joint_state = encoder(static_cast<JointState>(obj));
  *message.mutable_joint_torques()->mutable_state() = joint_state.state();
  *message.mutable_joint_torques()->mutable_joint_names() = joint_state.joint_names();
  *message.mutable_joint_torques()->mutable_torques() = joint_state.torques();
  return message.SerializeAsString();
}
template<>
JointTorques decode(const std::string& msg) {
  JointTorques obj;
  if (!decode(msg, obj)) {
    throw DecodingException("Could not decode the message into a JointTorques");
  }
  return obj;
}
template<>
bool decode(const std::string& msg, JointTorques& obj) {
  try {
    proto::StateMessage message;
    if (!(message.ParseFromString(msg)
    && message.message_type_case() == proto::StateMessage::MessageTypeCase::kJointTorques)) {
      if (!message.mutable_joint_torques()->ParseFromString(msg)) {
        return false;
      }
    }

    auto state = message.joint_torques();
    obj = JointState(state.state().name(), {state.joint_names().begin(), state.joint_names().end()});
    obj.set_torques(decoder(state.torques()));
    obj.set_empty(state.state().empty());
    return true;
  } catch (...) {
    return false;
  }
}

/* generic templates for future types:
template<> std::string encode<CartesianPose>(const CartesianPose& obj);
template<> CartesianPose decode(const std::string& msg);
template<> bool decode(const std::string& msg, CartesianPose& obj);
template<> std::string encode<CartesianPose>(const CartesianPose& obj) {
  proto::StateMessage message;
  // encode
  return message.SerializeAsString();
}
template<> CartesianPose decode(const std::string& msg) {
  CartesianPose obj;
  if (!decode(msg, obj)) {
    throw DecodingException("Could not decode the message into a CartesianPose");
  }
  return obj;
}
template<> bool decode(const std::string& msg, CartesianPose& obj) {
  try {
    proto::StateMessage message;
    if (!(message.ParseFromString(msg) && message.message_type_case() == proto::StateMessage::MessageTypeCase::kCartesianPose)) {
      if (!message.mutable_CartesianPose()->ParseFromString(msg)) {
        return false;
      }
    }
    // decode
    return true;
  } catch (...) {
    return false;
  }
}
*/

}
