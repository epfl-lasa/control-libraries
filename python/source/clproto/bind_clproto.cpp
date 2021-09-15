#include "clproto_bindings.h"

#include <string>

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

using namespace clproto;
using namespace state_representation;

template<typename T>
inline py::bytes encode_bytes(const T& object) {
  return py::bytes(encode(object));
}

void message_type(py::module_& m) {
  py::enum_<MessageType>(m, "MessageType")
      .value("UNKNOWN_MESSAGE", MessageType::UNKNOWN_MESSAGE)
      .value("STATE_MESSAGE", MessageType::STATE_MESSAGE)
      .value("SPATIAL_STATE_MESSAGE", MessageType::SPATIAL_STATE_MESSAGE)
      .value("CARTESIAN_STATE_MESSAGE", MessageType::CARTESIAN_STATE_MESSAGE)
      .value("CARTESIAN_POSE_MESSAGE", MessageType::CARTESIAN_POSE_MESSAGE)
      .value("CARTESIAN_TWIST_MESSAGE", MessageType::CARTESIAN_TWIST_MESSAGE)
      .value("CARTESIAN_ACCELERATION_MESSAGE", MessageType::CARTESIAN_ACCELERATION_MESSAGE)
      .value("CARTESIAN_WRENCH_MESSAGE", MessageType::CARTESIAN_WRENCH_MESSAGE)
      .value("JACOBIAN_MESSAGE", MessageType::JACOBIAN_MESSAGE)
      .value("JOINT_STATE_MESSAGE", MessageType::JOINT_STATE_MESSAGE)
      .value("JOINT_POSITIONS_MESSAGE", MessageType::JOINT_POSITIONS_MESSAGE)
      .value("JOINT_VELOCITIES_MESSAGE", MessageType::JOINT_VELOCITIES_MESSAGE)
      .value("JOINT_ACCELERATIONS_MESSAGE", MessageType::JOINT_ACCELERATIONS_MESSAGE)
      .value("JOINT_TORQUES_MESSAGE", MessageType::JOINT_TORQUES_MESSAGE)
      .value("SHAPE_MESSAGE", MessageType::SHAPE_MESSAGE)
      .value("ELLIPSOID_MESSAGE", MessageType::ELLIPSOID_MESSAGE)
      .value("PARAMETER_MESSAGE", MessageType::PARAMETER_MESSAGE)
      .export_values();
}

void parameter_message_type(py::module_& m) {
  py::enum_<ParameterMessageType>(m, "ParameterMessageType")
      .value("UNKNOWN_PARAMETER", ParameterMessageType::UNKNOWN_PARAMETER)
      .value("INT", ParameterMessageType::INT)
      .value("INT_ARRAY", ParameterMessageType::INT_ARRAY)
      .value("DOUBLE", ParameterMessageType::DOUBLE)
      .value("DOUBLE_ARRAY", ParameterMessageType::DOUBLE_ARRAY)
      .value("BOOL", ParameterMessageType::BOOL)
      .value("BOOL_ARRAY", ParameterMessageType::BOOL_ARRAY)
      .value("STRING", ParameterMessageType::STRING)
      .value("STRING_ARRAY", ParameterMessageType::STRING_ARRAY)
      .value("MATRIX", ParameterMessageType::MATRIX)
      .value("VECTOR", ParameterMessageType::VECTOR)
      .export_values();
}

void methods(py::module_& m) {
  m.def("is_valid", &is_valid, "Check if a serialized binary string can be decoded into a support control libraries message type.", "msg"_a);
  m.def("check_message_type", &check_message_type, "Check which control libraries message type a serialized binary string can be decoded as, if at all.", "msg"_a);
  m.def("check_parameter_message_type", &check_parameter_message_type, "Check which control libraries message type a serialized binary string can be decoded as, if at all.", "msg"_a);

  m.def("pack_fields", [](const std::vector<std::string>& encoded_fields) -> py::bytes {
    char data[CLPROTO_PACKING_MAX_FIELDS * CLPROTO_PACKING_MAX_FIELD_LENGTH];
    std::size_t packet_size = sizeof(field_length_t) * (encoded_fields.size() + 1);
    for (const auto& field : encoded_fields) {
      packet_size += field.size();
    }
    pack_fields(encoded_fields, data);
    return std::string(data, packet_size);
  }, "Pack an ordered vector of encoded field messages into a single data array.", "encoded_fields"_a);
  m.def("unpack_fields", [](const std::string& packet) -> std::vector<py::bytes> {
    std::vector<py::bytes> encoded_fields;
    for (std::string field : unpack_fields(packet.c_str())) {
      encoded_fields.emplace_back(field);
    }
    return encoded_fields;
  }, "Unpack a data array into an ordered vector of encoded field messages.", "data"_a);

  m.def("encode", [](const py::object& object, const MessageType& type) -> py::bytes {
    switch (type) {
      case MessageType::STATE_MESSAGE:
        return encode_bytes<State>(object.cast<State>());
      case MessageType::SPATIAL_STATE_MESSAGE:
        return encode_bytes<SpatialState>(object.cast<SpatialState>());
      case MessageType::CARTESIAN_STATE_MESSAGE:
        return encode_bytes<CartesianState>(object.cast<CartesianState>());
      case MessageType::CARTESIAN_POSE_MESSAGE:
        return encode_bytes<CartesianPose>(object.cast<CartesianPose>());
      case MessageType::CARTESIAN_TWIST_MESSAGE:
        return encode_bytes<CartesianTwist>(object.cast<CartesianTwist>());
      case MessageType::CARTESIAN_WRENCH_MESSAGE:
        return encode_bytes<CartesianWrench>(object.cast<CartesianWrench>());
      case MessageType::JACOBIAN_MESSAGE:
        return encode_bytes<Jacobian>(object.cast<Jacobian>());
      case MessageType::JOINT_STATE_MESSAGE:
        return encode_bytes<JointState>(object.cast<JointState>());
      case MessageType::JOINT_POSITIONS_MESSAGE:
        return encode_bytes<JointPositions>(object.cast<JointPositions>());
      case MessageType::JOINT_VELOCITIES_MESSAGE:
        return encode_bytes<JointVelocities>(object.cast<JointVelocities>());
      case MessageType::JOINT_ACCELERATIONS_MESSAGE:
        return encode_bytes<JointAccelerations>(object.cast<JointAccelerations>());
      case MessageType::JOINT_TORQUES_MESSAGE:
        return encode_bytes<JointTorques>(object.cast<JointTorques>());
      default:
        return py::bytes();
    }
  }, "Encode a control libraries object into a serialized binary string representation (wire format).", py::arg("object"), py::arg("type"));

  m.def("decode", [](const std::string& msg) -> py::object {
    switch (check_message_type(msg)) {
      case MessageType::STATE_MESSAGE:
        return py::cast(decode<State>(msg));
      case MessageType::SPATIAL_STATE_MESSAGE:
        return py::cast(decode<SpatialState>(msg));
      case MessageType::CARTESIAN_STATE_MESSAGE:
        return py::cast<CartesianState>(decode<CartesianState>(msg));
      case MessageType::CARTESIAN_POSE_MESSAGE:
        return py::cast(decode<CartesianPose>(msg));
      case MessageType::CARTESIAN_TWIST_MESSAGE:
        return py::cast(decode<CartesianTwist>(msg));
      case MessageType::CARTESIAN_WRENCH_MESSAGE:
        return py::cast(decode<CartesianWrench>(msg));
      case MessageType::JACOBIAN_MESSAGE:
        return py::cast(decode<Jacobian>(msg));
      case MessageType::JOINT_STATE_MESSAGE:
        return py::cast(decode<JointState>(msg));
      case MessageType::JOINT_POSITIONS_MESSAGE:
        return py::cast(decode<JointPositions>(msg));
      case MessageType::JOINT_VELOCITIES_MESSAGE:
        return py::cast(decode<JointVelocities>(msg));
      case MessageType::JOINT_ACCELERATIONS_MESSAGE:
        return py::cast(decode<JointAccelerations>(msg));
      case MessageType::JOINT_TORQUES_MESSAGE:
        return py::cast(decode<JointTorques>(msg));
      default:
        return py::none();
    }
  }, "Decode a serialized binary string from wire format into a control libraries object instance.", "msg"_a);

  m.def("to_json", [](const std::string& msg) { return to_json(msg); }, "Convert a serialized binary string from wire format into a JSON formatted state message description", "msg"_a);
  m.def("from_json", [](const std::string& json) -> py::bytes { return from_json(json); }, "Convert a JSON formatted state message description into a serialized binary string representation (wire format).", "msg"_a);
}

void bind_clproto(py::module_& m) {
  message_type(m);
  parameter_message_type(m);
  methods(m);
}