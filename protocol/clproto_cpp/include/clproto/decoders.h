#pragma once

#include <google/protobuf/repeated_field.h>
#include <state_representation/parameters/Parameter.hpp>
#include <state_representation/parameters/parameter.pb.h>

#include "clproto.h"

namespace clproto {

class DecoderNotImplementedException : public DecodingException {
public:
  explicit DecoderNotImplementedException(const std::string& msg);
};

/**
 * @brief Decoding helper method.
 * @tparam ObjT The control libraries output type
 * @tparam MsgT The protocol message input type
 * @param message The protocol message object
 * @return The equivalent decoded control libraries object
 */
template<typename ObjT, typename MsgT>
ObjT decoder(const MsgT& message);

/**
 * @brief Decoding helper method for a RepeatedField message
 * into vector data.
 * @tparam FieldT The datatype within the repeated field
 * @param message A RepeatedField message
 * @return The decoded vector of data
 */
template<typename FieldT>
std::vector<FieldT> decoder(const google::protobuf::RepeatedField<FieldT>& message);

/**
 * @brief Decoding helper method for a RepeatedPtrField message
 * into vector data.
 * @tparam FieldT The datatype within the repeated field
 * @param message A RepeatedPtrField message
 * @return The decoded vector of data
 */
template<typename FieldT>
std::vector<FieldT> decoder(const google::protobuf::RepeatedPtrField<FieldT>& message);

/**
 * @brief Decoding helper method for the Parameter type.
 * @tparam ParamT The type contained within the Parameter object
 * @param message The protocol Parameter message object
 * @return The decoded control libraries Parameter object
 */
template<typename ParamT>
state_representation::Parameter<ParamT> decoder(const state_representation::proto::Parameter& message);

template<typename ObjT, typename MsgT>
ObjT decoder(const MsgT&) {
  throw DecoderNotImplementedException("Templated decoder function not implemented!");
}

template<typename FieldT>
std::vector<FieldT> decoder(const google::protobuf::RepeatedField<FieldT>& message) {
  return {message.begin(), message.end()};
}

template<typename FieldT>
std::vector<FieldT> decoder(const google::protobuf::RepeatedPtrField<FieldT>& message) {
  return {message.begin(), message.end()};
}
}