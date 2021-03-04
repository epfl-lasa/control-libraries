/**
 * @author Baptiste Busch
 * @date 2019/08/05
 */

#pragma once

#include "dynamical_systems/DynamicalSystem.hpp"
#include "state_representation/Geometry/Ellipsoid.hpp"
#include "state_representation/Parameters/Parameter.hpp"
#include "state_representation/Space/Cartesian/CartesianPose.hpp"
#include "state_representation/Space/Cartesian/CartesianState.hpp"
#include "state_representation/Space/Cartesian/CartesianTwist.hpp"
#include <cmath>
#include <vector>

namespace DynamicalSystems {
/**
 * @class Circular
 * @brief Represent a Circular dynamical system to move around an center
 */
class Circular : public DynamicalSystem<state_representation::CartesianState> {
private:
  std::shared_ptr<state_representation::Parameter<state_representation::Ellipsoid>>
      limit_cycle_;///< limit_cycle of the dynamical system
  std::shared_ptr<state_representation::Parameter<double>>
      planar_gain_;///< gain associate to the system in the plane of the circle
  std::shared_ptr<state_representation::Parameter<double>>
      normal_gain_;///< gain associate to the system normal to the plane of the circle
  std::shared_ptr<state_representation::Parameter<double>>
      circular_velocity_;///< velocity at wich to navigate the limit cycle

protected:
  /**
   * @brief Compute the dynamics of the input state.
   * Internal function, to be redefined based on the
   * type of dynamical system, called by the evaluate
   * function
   * @param state the input state
   * @return the output state
   */
  state_representation::CartesianState compute_dynamics(const state_representation::CartesianState& state) const;

public:
  /**
   * @brief Default constructor with center and fixed radius
   * @param center the center of the limit cycle
   * @param radius radius of the limit cycle (default=1.)
   * @param gain gain of the dynamical system (default=1.)
   * @param circular_velocity circular velocity to move around the limit cycle
   */
  explicit Circular(const state_representation::CartesianState& center,
                    double radius = 1.0,
                    double gain = 1.0,
                    double circular_velocity = M_PI / 2);

  /**
   * @brief Cnstructor with an elliptic limit cycle
   * @param limit_cycle the limit cycle as an ellipsoid
   * @param gain gain of the dynamical system (default=1.)
   * @param circular_velocity circular velocity to move around the limit cycle
   */
  explicit Circular(const state_representation::Ellipsoid& limit_cycle,
                    double gain = 1.0,
                    double circular_velocity = M_PI / 2);

  /**
   * @brief Getter of the center
   * @return the center as a const reference
   */
  const state_representation::CartesianPose& get_center() const;

  /**
   * @brief Setter of the center as a new value
   * @param center the new center
   */
  void set_center(const state_representation::CartesianPose& center);

  /**
   * @brief Getter of the planar gain attribute
   * @return The gain value in the plane of the circle
   */
  double get_planar_gain() const;

  /**
   * @brief Getter of the normal gain attribute
   * @return The gain value normal to the plane of the circle
   */
  double get_normal_gain() const;

  /**
   * @brief Setter of the gain attribute
   */
  void set_gain(double gain);

  /**
   * @brief Setter of the planar gain attribute
   */
  void set_planar_gain(double gain);

  /**
   * @brief Setter of the normal gain attribute
   */
  void set_normal_gain(double gain);

  /**
   * @brief Getter of the radiuses of the limit cycle
   * @return the radius value
   */

  double get_rotation_angle() const;

  /**
   * @brief Setter of the rotation angle attribute
   * @return The rotation angle value
   */
  void set_rotation_angle(double angle);

  /**
   * @brief Getter of the rotation angle attribute
   * @param The rotation angle value
   */

  const std::vector<double>& get_radiuses() const;

  /**
   * @brief Setter of the radiuses of the limit cycle
   * @param radiuses the new radiuses values
   */
  void set_radiuses(const std::vector<double>& radiuses);

  /**
   * @brief Setter of the radius of the limit cycle as a single value, i.e. perfect cycle
   * @param radiuses the new radiuses values
   */
  void set_radius(double radius);

  /**
   * @brief Getter of the circular velocity attribute
   * @return the cirular velocity value
   */
  double get_circular_velocity() const;

  /**
   * @brief Setter of the circular_velocity attribute
   * @param circular_velocity the new circular_velocity value
   */
  void set_circular_velocity(double circular_velocity);

  /**
   * @brief Getter of the limit cycle attribute
   * @return the limit cycle
   */
  const state_representation::Ellipsoid& get_limit_cycle() const;

  /**
   * @brief Setter of the limit cycle attribute
   * @param the limit cycle value
   */
  void set_limit_cycle(const state_representation::Ellipsoid& limit_cycle);

  /**
   * @brief Return a list of all the parameters of the dynamical system
   * @return the list of parameters
   */
  std::list<std::shared_ptr<state_representation::ParameterInterface>> get_parameters() const override;
};

inline const state_representation::CartesianPose& Circular::get_center() const {
  return this->limit_cycle_->get_value().get_center_pose();
}

inline void Circular::set_center(const state_representation::CartesianPose& center) {
  this->limit_cycle_->get_value().set_center_pose(center);
}

inline double Circular::get_planar_gain() const {
  return this->planar_gain_->get_value();
}

inline double Circular::get_normal_gain() const {
  return this->normal_gain_->get_value();
}

inline void Circular::set_gain(double gain) {
  this->planar_gain_->set_value(gain);
  this->normal_gain_->set_value(gain);
}

inline void Circular::set_planar_gain(double gain) {
  this->planar_gain_->set_value(gain);
}

inline void Circular::set_normal_gain(double gain) {
  this->normal_gain_->set_value(gain);
}

inline double Circular::get_rotation_angle() const {
  return this->limit_cycle_->get_value().get_rotation_angle();
}

inline void Circular::set_rotation_angle(double angle) {
  this->limit_cycle_->get_value().set_rotation_angle(angle);
}

inline const std::vector<double>& Circular::get_radiuses() const {
  return this->limit_cycle_->get_value().get_axis_lengths();
}

inline void Circular::set_radiuses(const std::vector<double>& radiuses) {
  this->limit_cycle_->get_value().set_axis_lengths(radiuses);
}

inline void Circular::set_radius(double radius) {
  this->limit_cycle_->get_value().set_axis_lengths({radius, radius});
}

inline double Circular::get_circular_velocity() const {
  return this->circular_velocity_->get_value();
}

inline void Circular::set_circular_velocity(double circular_velocity) {
  this->circular_velocity_->set_value(circular_velocity);
}

inline const state_representation::Ellipsoid& Circular::get_limit_cycle() const {
  return this->limit_cycle_->get_value();
}

inline void Circular::set_limit_cycle(const state_representation::Ellipsoid& limit_cycle) {
  this->limit_cycle_->set_value(limit_cycle);
}
}// namespace DynamicalSystems