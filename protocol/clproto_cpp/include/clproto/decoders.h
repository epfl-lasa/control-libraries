#pragma once

#include <state_representation/State.hpp>

#include "state_representation/state.pb.h"
#include "state_representation/space/cartesian/cartesian_state.pb.h"

using namespace state_representation;

namespace clproto {

/**
 * @brief Local decoding helper for StateType
 * @param type The state representation StateType
 * @return The equivalent proto StateType
 */
state_representation::StateType decoder(const proto::StateType& message);

/**
 * @brief Local decoding helper for Vector3d
 * @param type The proto Vector3d
 * @return The equivalent Eigen Vector3d
 */
Eigen::Vector3d decoder(const proto::Vector3d& message);

/**
 * @brief Local decoding helper for Quaterniond
 * @param type The proto Quaterniond
 * @return The equivalent Eigen Quaterniond
 */
Eigen::Quaterniond decoder(const proto::Quaterniond& message);

/**
 * @brief Local decoding helper for repeated double data
 * @param type The protobuf RepeatedField<double> data
 * @return The equivalent STL vector
 */
std::vector<double> decoder(const google::protobuf::RepeatedField<double>& message);

}