#pragma once

#include <stdexcept>
#include <string>
#include <vector>

#define CLPROTO_PACKING_MAX_FIELD_LENGTH (4096)
#define CLPROTO_PACKING_MAX_FIELDS (64)

/**
 * @namespace clproto
 * @brief Bindings to encode and decode state objects into serialised binary message
 */
namespace clproto {

/**
 * @typedef field_length_t
 * @brief Size type used to indicate number of fields
 * and field data length in ::pack_fields() and
 * ::unpack_fields() methods
 */
typedef std::size_t field_length_t;

/**
 * @class DecodingException
 * @brief A DecodingException is raised whenever a
 * decoding operation fails due to invalid encoding.
 */
class DecodingException : public std::runtime_error {
public:
  explicit DecodingException(const std::string& msg);
};

/**
 * @enum MessageType
 * @brief The MessageType enumeration contains the possible
 * message types in the clproto.
 * @details The values and order of this enumeration
 * are synchronized with the fields of the protobuf
 * StateMessage type, allowing a one-to-one mapping
 * to the StateMessage type case.
 */
enum MessageType {
  UNKNOWN_MESSAGE = 0,
  STATE_MESSAGE = 1,
  SPATIAL_STATE_MESSAGE = 2,
  CARTESIAN_STATE_MESSAGE = 3,
  CARTESIAN_POSE_MESSAGE = 4,
  CARTESIAN_TWIST_MESSAGE = 5,
  CARTESIAN_ACCELERATION_MESSAGE = 6,
  CARTESIAN_WRENCH_MESSAGE = 7,
  JACOBIAN_MESSAGE = 8,
  JOINT_STATE_MESSAGE = 9,
  JOINT_POSITIONS_MESSAGE = 10,
  JOINT_VELOCITIES_MESSAGE = 11,
  JOINT_ACCELERATIONS_MESSAGE = 12,
  JOINT_TORQUES_MESSAGE = 13,
  SHAPE_MESSAGE = 14,
  ELLIPSOID_MESSAGE = 15,
  PARAMETER_MESSAGE = 16
};

/**
 * @enum ParameterMessageType
 * @brief The ParameterMessageType enumeration contains the
 * possible value types contained in a parameter message.
 * @details The values and order of this enumeration
 * are synchronized with the fields of the protobuf
 * ParameterValue type, allowing a one-to-one mapping
 * to the ParameterValue type case.
 */
enum ParameterMessageType {
  UNKNOWN_PARAMETER = 0,
  DOUBLE = 1,
  DOUBLE_ARRAY = 2,
  BOOL = 3,
  BOOL_ARRAY = 4,
  STRING = 5,
  STRING_ARRAY = 6,
  MATRIX = 7,
  VECTOR = 8
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
 * @brief Check which control libraries parameter type a
 * serialized binary string can be decoded as, if at all.
 * @param msg The serialized binary string to check
 * @return The ParameterMessageType of the contained type or UNKNOWN
 */
ParameterMessageType check_parameter_message_type(const std::string& msg);

/**
 * @brief Encode a control libraries object into
 * a serialized binary string representation (wire format).
 * @tparam T The provided control libraries object type
 * @param obj The control libraries object to encode
 * @return The serialized binary string encoding
 */
template<typename T>
std::string encode(const T& obj);

/**
 * @brief Decode a serialized binary string from
 * wire format into a control libraries object instance.
 * Throws an exception if the message cannot be decoded
 * into the desired type.
 * @tparam The desired control libraries object type
 * @param msg The serialized binary string to decode
 * @return A new instance of the control libraries object
 */
template<typename T>
T decode(const std::string& msg);

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
template<typename T>
bool decode(const std::string& msg, T& obj);

/**
 * @brief Pack an ordered vector of encoded field messages into a single data array.
 * @details To send multiple messages in one packet, there must
 * be some delimiting logic to distinguish the end of one field from the
 * start of the next. This packing function encodes the number of fields (N)
 * as the first data entry in the packet, then the size of each field in the
 * next N data entries, followed by the raw concatenated data of each field.
 * The order of the original vector is preserved. The corresponding
 * ::unpack_fields() method can be used to restore the original vector
 * of fields from the data buffer.
 * @param fields An ordered vector of encoded message fields
 * @param[out] data A raw data array to be packed with the fields
 */
void pack_fields(const std::vector<std::string>& fields, char* data);

/**
 * @brief Unpack a data array into an ordered vector of encoded field messages.
 * @details A buffer of encoded fields serialized by ::pack_fields()
 * can be unpacked by this method. It expects the first data entry
 * in the data buffer to contain the number of fields (N). The next
 * N data entries then must contain the data length of each subsequent
 * field. Finally, the rest of the data is broken into ordered fields
 * based on the interpreted field data lengths.
 * @param data A raw data array that has been packed by ::pack_fields()
 * @return An ordered vector of encoded message fields
 */
std::vector<std::string> unpack_fields(const char* data);

}
