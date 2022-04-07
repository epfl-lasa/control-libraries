#pragma once

/**
 * @namespace state_representation
 * @brief Core state variables and objects
 */
namespace state_representation {

/**
 * @enum StateType
 * @brief The class types inheriting from State.
 */
enum class StateType {
  STATE,
  SPATIAL_STATE,
  CARTESIAN_STATE,
  CARTESIAN_POSE,
  CARTESIAN_TWIST,
  CARTESIAN_ACCELERATION,
  CARTESIAN_WRENCH,
  JOINT_STATE,
  JOINT_POSITIONS,
  JOINT_VELOCITIES,
  JOINT_ACCELERATIONS,
  JOINT_TORQUES,
  JACOBIAN,
  PARAMETER,
  GEOMETRY_SHAPE,
  GEOMETRY_ELLIPSOID,
  TRAJECTORY,
#ifdef EXPERIMENTAL_FEATURES
  DUAL_QUATERNION_STATE,
  DUAL_QUATERNION_POSE,
  DUAL_QUATERNION_TWIST
#endif
};
}