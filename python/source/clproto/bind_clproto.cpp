#include "clproto_bindings.h"

#include <state_representation/State.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>

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

void state(py::module_& m) {
  m.def("is_valid", &is_valid, "Check if a serialized binary string can be decoded into a support control libraries message type.", "msg"_a);
  m.def("check_message_type", &check_message_type, "Check which control libraries message type a serialized binary string can be decoded as, if at all.", "msg"_a);
  m.def("check_parameter_message_type", &check_parameter_message_type, "Check which control libraries message type a serialized binary string can be decoded as, if at all.", "msg"_a);

  m.def("pack_fields", [](const std::vector<std::string>& fields) {
    char* data;
    pack_fields(fields, data);
    return data;
  }, "Pack an ordered vector of encoded field messages into a single data array.", "fields"_a);
  m.def("unpack_fields", &unpack_fields, "Unpack a data array into an ordered vector of encoded field messages.", "data"_a);

  m.def("encode", [](const state_representation::State& state) {
    return encode(state);
  }, "Encode a control libraries object into a serialized binary string representation (wire format).", "obj"_a);
  m.def("decode", [](const std::string& msg, state_representation::State& obj) {
    return decode(msg, obj);
  }, "Exception safe decoding of a serialized binary string wire format into a control libraries object instance. It modifies the object by reference if the decoding is successful, and leaves it unmodified otherwise.", "msg"_a, "obj"_a);

  m.def("encode", [](const state_representation::CartesianState& state) {
    return encode(state);
  }, "Encode a control libraries object into a serialized binary string representation (wire format).", "obj"_a);
  m.def("decode", [](const std::string& msg, state_representation::CartesianState& obj) {
    return decode(msg, obj);
  }, "Exception safe decoding of a serialized binary string wire format into a control libraries object instance. It modifies the object by reference if the decoding is successful, and leaves it unmodified otherwise.", "msg"_a, "obj"_a);
}

void bind_clproto(py::module_& m) {
  state(m);
}