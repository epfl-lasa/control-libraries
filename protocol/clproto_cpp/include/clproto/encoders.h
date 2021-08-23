#pragma once

#include <google/protobuf/repeated_field.h>

namespace clproto {

class EncoderNotImplementedException : public std::runtime_error {
public:
  explicit EncoderNotImplementedException(const std::string& msg);
};

/**
 * @brief Encoding helper method
 * @tparam MsgT The protocol message output type
 * @tparam ObjT The control libraries input type
 * @param object The control libraries object
 * @return The equivalent encoded protocol message object
 */
template<typename MsgT, typename ObjT>
MsgT encoder(const ObjT& object);

/**
 * @brief Encoding helper method for C-style arrays into
 * a RepeatedField message type
 * @tparam FieldT The datatype within the repeated field
 * @param data A C-style array of data
 * @param size The length of the data array
 * @return The encoded RepeatedField protocol message object
 */
template<typename FieldT>
google::protobuf::RepeatedField<FieldT> encoder(const FieldT* data, std::size_t size);

template<typename MsgT, typename ObjT>
MsgT encoder(const ObjT&) {
  throw EncoderNotImplementedException("Templated encoder function not implemented!");
}

template<typename FieldT>
google::protobuf::RepeatedField<FieldT> encoder(const FieldT*, std::size_t) {
  throw EncoderNotImplementedException("Templated encoder function not implemented!");
}

}