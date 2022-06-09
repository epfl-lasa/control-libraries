#pragma once

#include "dynamical_systems/IDynamicalSystem.hpp"
#include "state_representation/parameters/Parameter.hpp"
#include "state_representation/space/cartesian/CartesianPose.hpp"
#include "state_representation/space/cartesian/CartesianState.hpp"
#include <cmath>
#include <vector>

namespace dynamical_systems {

/**
 * @class Ring
 * @brief Represent a Ring dynamical system limit cycle to move around a radius within a fixed width.
 */
class Ring : public IDynamicalSystem<state_representation::CartesianState> {
public:
  /**
   * @brief Empty constructor
  */
  Ring();

  /**
   * @brief Constructor from an initial parameter list
   * @param parameters A parameter list containing initial parameters
   */
  explicit Ring(const std::list<std::shared_ptr<state_representation::ParameterInterface>>& parameters);

  /**
   * @copydoc IDynamicalSystem::set_parameter
   */
  void set_base_frame(const state_representation::CartesianState& base_frame) override;

  /**
   * @copydoc IDynamicalSystem::compute_dynamics
   */
  [[nodiscard]] state_representation::CartesianState
  compute_dynamics(const state_representation::CartesianState& state) const override;

private:
  typedef std::shared_ptr<state_representation::Parameter<double>> ptr_param_double_t;

  /**
   * @copydoc IDynamicalSystem::validate_and_set_parameter
   */
  void validate_and_set_parameter(const std::shared_ptr<state_representation::ParameterInterface>& parameter) override;

  /**
   * @brief Setter of the center as a new value
   * @param center the new center
   */
  void set_center(const state_representation::CartesianPose& center);

  /**
   * @brief Setter of the attractor orientation relative to the circle reference frame
   * @param rotation the rotation quaternion
   */
  void set_rotation_offset(const Eigen::Quaterniond& rotation);

  /**
   * @brief Getter of the rotation offset in the center frame
   * @return the rotation offset as a quaternion
   */
  [[nodiscard]] Eigen::Quaterniond get_rotation_offset() const;

  [[nodiscard]] Eigen::Vector3d calculate_local_linear_velocity(
      const state_representation::CartesianPose& pose, double& local_field_strength
  ) const;

  [[nodiscard]] Eigen::Vector3d calculate_local_angular_velocity(
      const state_representation::CartesianPose& pose, const Eigen::Vector3d& linear_velocity,
      double local_field_strength
  ) const;

  std::shared_ptr<state_representation::Parameter<state_representation::CartesianPose>> center_; ///< ring center
  std::shared_ptr<state_representation::Parameter<state_representation::CartesianPose>>
      rotation_offset_; ///< orientation of attractor in circle frame
  ptr_param_double_t radius_; ///< ring radius [m]
  ptr_param_double_t width_; ///< distance around radius where field rotates [m]
  ptr_param_double_t speed_; ///< desired linear speed when travelling along the circle radius [m/s]
  ptr_param_double_t field_strength_; ///< scale factor for desired speed outside of radius + width
  ptr_param_double_t normal_gain_; ///< scale factor for the speed normal to the circular plane
  ptr_param_double_t angular_gain_; ///< scale factor for angular velocity restitution
};
}// namespace dynamical_systems
