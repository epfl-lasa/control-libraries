#pragma once

#include <google/protobuf/repeated_field.h>

#include <state_representation/State.hpp>
#include <state_representation/space/SpatialState.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/space/Jacobian.hpp>
#include <state_representation/space/joint/JointState.hpp>
#include <state_representation/parameters/Parameter.hpp>

#include "clproto.h"
#include "state_representation/state_message.pb.h"

namespace clproto {

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
google::protobuf::RepeatedField<double> matrix_encoder(const Eigen::MatrixXd& matrix);

/*
 * Declarations for encoding helpers
 */
state_representation::proto::StateType encoder(const state_representation::StateType& type);
state_representation::proto::State encoder(const state_representation::State& state);
state_representation::proto::SpatialState encoder(const state_representation::SpatialState& spatial_state);
state_representation::proto::Vector3d encoder(const Eigen::Vector3d& vector);
state_representation::proto::Quaterniond encoder(const Eigen::Quaterniond& quaternion);
state_representation::proto::CartesianState encoder(const state_representation::CartesianState& cartesian_state);
state_representation::proto::Jacobian encoder(const state_representation::Jacobian& jacobian);
state_representation::proto::JointState encoder(const state_representation::JointState& joint_state);

/*
 * Definitions for templated RepeatedField methods
 */
template<typename FieldT>
google::protobuf::RepeatedField<FieldT> encoder(const std::vector<FieldT>& data) {
  return google::protobuf::RepeatedField<FieldT>({data.begin(), data.end()});
}
}