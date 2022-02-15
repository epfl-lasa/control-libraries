#pragma once

namespace controllers {

/**
 * @brief Enumeration of the implemented controllers
 */
enum class CONTROLLER_TYPE {
  NONE,
  IMPEDANCE,
  DISSIPATIVE,
  DISSIPATIVE_LINEAR,
  DISSIPATIVE_ANGULAR,
  DISSIPATIVE_DECOUPLED,
  VELOCITY_IMPEDANCE,
  COMPLIANT_TWIST
};

}