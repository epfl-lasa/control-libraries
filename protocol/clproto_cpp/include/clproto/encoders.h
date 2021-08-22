#pragma once

#include <state_representation/State.hpp>
#include <state_representation/space/SpatialState.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/robot/Jacobian.hpp>
#include <state_representation/robot/JointState.hpp>

#include "state_representation/state.pb.h"
#include "state_representation/space/spatial_state.pb.h"
#include "state_representation/space/cartesian/cartesian_state.pb.h"
#include "state_representation/space/joint/jacobian.pb.h"
#include "state_representation/space/joint/joint_state.pb.h"

using namespace state_representation;

namespace clproto {

/**
 * @brief Local encoding helper for StateType
 * @param type The state representation StateType
 * @return The equivalent proto StateType
 */
proto::StateType encoder(const StateType& type);

/**
 * @brief Local encoding helper for State
 * @param type The state representation State
 * @return The equivalent proto State
 */
proto::State encoder(const State& state);

/**
 * @brief Local encoding helper for SpatialState
 * @param type The state representation SpatialState
 * @return The equivalent proto SpatialState
 */
proto::SpatialState encoder(const SpatialState& spatial_state);

/**
 * @brief Local encoding helper for Vector3d
 * @param type The Eigen Vector3d
 * @return The equivalent proto Vector3d
 */
proto::Vector3d encoder(const Eigen::Vector3d& vector);

/**
 * @brief Local encoding helper for Quaterniond
 * @param type The Eigen Quaterniond
 * @return The equivalent proto Quaterniond
 */
proto::Quaterniond encoder(const Eigen::Quaterniond& quaternion);

/**
 * Local encoding helper for generic Eigen data
 * @param data The data buffer as type double
 * @param size The size of the data buffer
 * @return A protobuf RepeatedField<double> type
 */
google::protobuf::RepeatedField<double> encoder(const double* data, Eigen::Index size);

/**
 * @brief Local encoding helper for CartesianState
 * @param type The state representation CartesianState
 * @return The equivalent proto CartesianState
 */
proto::CartesianState encoder(const CartesianState& cartesian_state);

/**
 * @brief Local encoding helper for Jacobian
 * @param type The state representation Jacobian
 * @return The equivalent proto Jacobian
 */
proto::Jacobian encoder(const Jacobian& jacobian);

/**
 * @brief Local encoding helper for JointState
 * @param type The state representation JointState
 * @return The equivalent proto JointState
 */
proto::JointState encoder(const JointState& joint_state);

}