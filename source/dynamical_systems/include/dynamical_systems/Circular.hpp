#pragma once

#include "dynamical_systems/IDynamicalSystem.hpp"
#include "state_representation/geometry/Ellipsoid.hpp"
#include "state_representation/parameters/Parameter.hpp"

namespace dynamical_systems {

/**
 * @class Circular
 * @brief Represent a Circular dynamical system to move around an center.
 */
class Circular : public IDynamicalSystem<state_representation::CartesianState> {
public:
  /**
   * @brief Empty constructor
   */
  Circular();

  /**
   * @brief Constructor from an initial parameter list
   * @param parameters A parameter list containing initial parameters
   */
  explicit Circular(const std::list<std::shared_ptr<state_representation::ParameterInterface>>& parameters);

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
  /**
   * @copydoc IDynamicalSystem::validate_and_set_parameter
   */
  void validate_and_set_parameter(const std::shared_ptr<state_representation::ParameterInterface>& parameter) override;

  /**
   * @brief Setter of the center as a new value
   * @param center the new center
   */
  void set_limit_cycle(state_representation::Ellipsoid& limit_cycle);

  std::shared_ptr<state_representation::Parameter<state_representation::Ellipsoid>>
      limit_cycle_;///< limit_cycle of the dynamical system
  std::shared_ptr<state_representation::Parameter<double>>
      planar_gain_;///< gain associate to the system in the plane of the circle
  std::shared_ptr<state_representation::Parameter<double>>
      normal_gain_;///< gain associate to the system normal to the plane of the circle
  std::shared_ptr<state_representation::Parameter<double>>
      circular_velocity_;///< velocity at wich to navigate the limit cycle
};
}// namespace dynamical_systems
