#pragma once

#include <stdexcept>
#include <string>

namespace clproto {

class DecodingException : public std::runtime_error {
public:
  explicit DecodingException(const std::string& msg);
};

class NotImplementedException : public std::runtime_error {
public:
  explicit NotImplementedException(const std::string& msg);
};

enum MessageType {
  UNKNOWN_MESSAGE = 0,
  STATE_MESSAGE = 1,
  SPATIAL_STATE_MESSAGE = 2,
  CARTESIAN_STATE_MESSAGE = 3,
  CARTESIAN_POSE_MESSAGE = 4,
  CARTESIAN_TWIST_MESSAGE = 5,
  CARTESIAN_WRENCH_MESSAGE = 6,
  JACOBIAN_MESSAGE = 7,
  JOINT_STATE_MESSAGE = 8,
  JOINT_POSITIONS_MESSAGE = 9,
  JOINT_VELOCITIES_MESSAGE = 10,
  JOINT_ACCELERATIONS_MESSAGE = 14,
  JOINT_TORQUES_MESSAGE = 11,
  SHAPE_MESSAGE = 12,
  ELLIPSOID_MESSAGE = 13,
};

/**
 * @brief Check if a serialized binary string can be
 * decoded into a support control libraries message type.
 * @param msg The serialized binary string to check
 * @return True if the message can be decoded, false otherwise
 */
bool is_valid(const std::string& msg);

/**
 * @brief Check which control libraries message type a
 * serialized binary string can be decoded as, if at all.
 * @param msg The serialized binary string to check
 * @return The MessageType of the contained type or UNKNOWN
 */
MessageType check_message_type(const std::string& msg);

/**
 * @brief Encode a control libraries object into
 * a serialized binary string representation (wire format).
 * @tparam T The provided control libraries object type
 * @param obj The control libraries object to encode
 * @return The serialized binary string encoding
 */
template<typename T> std::string encode(const T& obj);

/**
 * @brief Decode a serialized binary string from
 * wire format into a control libraries object instance.
 * Throws an exception if the message cannot be decoded
 * into the desired type.
 * @tparam The desired control libraries object type
 * @param msg The serialized binary string to decode
 * @return A new instance of the control libraries object
 */
template<typename T> T decode(const std::string& msg);

/**
 * @brief Exception safe decoding of a serialized binary string
 * wire format into a control libraries object instance.
 * It modifies the object by reference if the decoding is
 * successful, and leaves it unmodified otherwise.
 * @tparam T The desired control libraries object type
 * @param msg The serialized binary string to decode
 * @param obj A reference to a control libraries object
 * @return A success status boolean
 */
template<typename T> bool decode(const std::string& msg, T& obj);


template<typename T>
std::string encode(const T& obj) {
  throw NotImplementedException("The encoding of the requested type is not implemented");
}

template<typename T>
T decode(const std::string& msg) {
  throw NotImplementedException("The decoding of the requested type is not implemented");
}

template<typename T>
bool decode(const std::string& msg, T& obj) {
  throw NotImplementedException("The decoding of the requested type is not implemented");
}

}
