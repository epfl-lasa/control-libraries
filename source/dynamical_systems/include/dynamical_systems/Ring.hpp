#pragma once

#include "dynamical_systems/DynamicalSystem.hpp"
#include "state_representation/parameters/Parameter.hpp"
#include "state_representation/space/cartesian/CartesianPose.hpp"
#include "state_representation/space/cartesian/CartesianState.hpp"
#include <cmath>
#include <vector>

namespace DynamicalSystems {
/**
 * @class Ring
 * @brief Represent a Ring dynamical system limit cycle to move around a radius within a fixed width
 */
class Ring : public DynamicalSystem<state_representation::CartesianState> {
private:
  typedef std::shared_ptr<state_representation::Parameter<double>> ptr_param_double_t;
  std::shared_ptr<state_representation::Parameter<state_representation::CartesianPose>> center_; ///< ring center
  std::shared_ptr<state_representation::Parameter<state_representation::CartesianPose>> rotation_offset_; ///< orientation of attractor in circle frame
  ptr_param_double_t radius_; ///< ring radius [m]
  ptr_param_double_t width_; ///< distance around radius where field rotates [m]
  ptr_param_double_t speed_; ///< desired linear speed when travelling along the circle radius [m/s]
  ptr_param_double_t field_strength_; ///< scale factor for desired speed outside of radius + width
  ptr_param_double_t normal_gain_; ///< scale factor for the speed normal to the circular plane
  ptr_param_double_t angular_gain_; ///< scale factor for angular velocity restitution

  Eigen::Vector3d calculateLocalLinearVelocity(const state_representation::CartesianPose& pose,
                                               double& localFieldStrength) const;
  Eigen::Vector3d calculateLocalAngularVelocity(const state_representation::CartesianPose& pose,
                                                const Eigen::Vector3d& linearVelocity,
                                                double localFieldStrength) const;

protected:
  /**
   * @brief Compute the dynamics of the input state.
   * Internal function, to be redefined based on the
   * type of dynamical system, called by the evaluate
   * function
   * @param state the input state
   * @return the output state
   */
  state_representation::CartesianState compute_dynamics(const state_representation::CartesianState& state) const override;

public:
  /**
   * @brief Default constructor with center and fixed radius
   * @param center the center of the limit cycle
   * @param radius radius of the limit cycle (default=1.)
   */
  explicit Ring(const state_representation::CartesianState& center,
                    double radius = 1.0,
                    double width = 0.5,
                    double speed = 1.0,
                    double field_strength = 1.0,
                    double normal_gain = 1.0,
                    double angular_gain = 1.0);

  /**
   * @brief Setter of the base frame as a new value
   * @param base_frame the new base frame
   */
  void set_base_frame(const state_representation::CartesianState& base_frame) override;

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
  * @brief Setter of the radius of the ring
  * @param radius the radius [m]
  */
  void set_radius(double radius);

  /**
  * @brief Setter of the field width around the ring radius
  * @param width the width [m]
  */
  void set_width(double width);

  /**
  * @brief Setter of the linear speed of the ring DS
  * @param speed the linear speed [m/s]
  */
  void set_speed(double speed);

  /**
  * @brief Setter of the field strength
  * @param field_strength the field strength
  */
  void set_field_strength(double field_strength);

  /**
  * @brief Setter of the gain normal to the ring plane
  * @param normal_gain the normal gain
  */
  void set_normal_gain(double normal_gain);

  /**
  * @brief Setter of the angular restitution gain
  * @param angular_gain the angular gain
  */
  void set_angular_gain(double angular_gain);


  /**
   * @brief Getter of the center
   * @return the center as a const reference
   */
  const state_representation::CartesianPose& get_center() const;

  /**
   * @brief Getter of the rotation offset in the center frame
   * @return the rotation offset as a quaternion
   */
  Eigen::Quaterniond get_rotation_offset() const;

  /**
  * @brief Getter of the radius of the ring
  * @return the radius [m]
  */
  double get_radius() const;

  /**
  * @brief Getter of the field width around the ring radius
  * @return the width [m]
  */
  double get_width() const;

  /**
  * @brief Getter of the linear speed of the ring DS
  * @return the linear speed [m/s]
  */
  double get_speed() const;

  /**
  * @brief Getter of the field strength
  * @return the field strength
  */
  double get_field_strength() const;

  /**
  * @brief Getter of the gain normal to the ring plane
  * @return the normal gain
  */
  double get_normal_gain() const;

  /**
  * @brief Getter of the angular restitution gain
  * @return the angular gain
  */
  double get_angular_gain() const;

  /**
   * @brief Return a list of all the parameters of the dynamical system
   * @return the list of parameters
   */
  std::list<std::shared_ptr<state_representation::ParameterInterface>> get_parameters() const override;
};

}// namespace DynamicalSystems