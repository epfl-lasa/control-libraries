#pragma once

#include <google/protobuf/repeated_field.h>

#include <state_representation/State.hpp>
#include <state_representation/parameters/Parameter.hpp>

#include "state_representation/state_message.pb.h"

namespace clproto {

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

/*
 * Declarations for decoding helpers
 */
std::vector<bool> decoder(const google::protobuf::RepeatedField<bool>& message);
state_representation::StateType decoder(const state_representation::proto::StateType& message);
Eigen::Vector3d decoder(const state_representation::proto::Vector3d& message);
Eigen::Quaterniond decoder(const state_representation::proto::Quaterniond& message);

/*
 * Definitions for templated RepeatedField methods
 */
template<typename FieldT>
std::vector<FieldT> decoder(const google::protobuf::RepeatedField<FieldT>& message) {
  return {message.begin(), message.end()};
}

template<typename FieldT>
std::vector<FieldT> decoder(const google::protobuf::RepeatedPtrField<FieldT>& message) {
  return {message.begin(), message.end()};
}

}