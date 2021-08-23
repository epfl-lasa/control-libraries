#pragma once

#include <google/protobuf/repeated_field.h>
#include <state_representation/parameters/Parameter.hpp>
#include <state_representation/parameters/parameter.pb.h>

namespace clproto {

class EncoderNotImplementedException : public std::runtime_error {
public:
  explicit EncoderNotImplementedException(const std::string& msg);
};

/**
 * @brief Encoding helper method.
 * @tparam MsgT The protocol message output type
 * @tparam ObjT The control libraries input type
 * @param object The control libraries object
 * @return The equivalent encoded protocol message object
 */
template<typename MsgT, typename ObjT>
MsgT encoder(const ObjT& object);

/**
 * @brief Encoding helper method for the Parameter type.
 * @tparam ParamT The type contained within the Parameter object
 * @param parameter The control libraries Parameter object
 * @return The encoded protocol Parameter message object
 */
template<typename ParamT>
state_representation::proto::Parameter encoder(const state_representation::Parameter<ParamT>& parameter);

/**
 * @brief Encoding helper method for vector data into
 * a RepeatedField message type.
 * @tparam FieldT The datatype within the repeated field
 * @param data A vector of data
 * @return The encoded RepeatedField protocol message object
 */
template<typename FieldT>
google::protobuf::RepeatedField<FieldT> encoder(const std::vector<FieldT>& data);

/**
 * @brief Encoding helper method for Eigen data into
 * a RepeatedField message type.
 * @param matrix An Eigen matrix of data
 * @return The encoded RepeatedField protocol message object
 */
google::protobuf::RepeatedField<double> encoder(const Eigen::MatrixXd& matrix);

template<typename MsgT, typename ObjT>
MsgT encoder(const ObjT&) {
  throw EncoderNotImplementedException("Templated encoder function not implemented!");
}

template<typename FieldT>
google::protobuf::RepeatedField<FieldT> encoder(const std::vector<FieldT>& data) {
  return google::protobuf::RepeatedField<FieldT>({data.begin(), data.end()});
}

}