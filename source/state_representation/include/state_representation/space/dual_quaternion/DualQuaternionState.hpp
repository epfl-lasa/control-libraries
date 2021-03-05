#pragma once

#include "state_representation/space/SpatialState.hpp"

namespace state_representation {
/**
 * @class DualQuaternionState
 * @brief Class to represent a state in Dual Quaternion space
 */
class DualQuaternionState : public SpatialState {
private:
  Eigen::Quaterniond primary; ///< primary part of the dual quaternion
  Eigen::Quaterniond dual; ///< dual part of the dual quaternion

public:
  /**
    * @brief Empty constructor
     */
  explicit DualQuaternionState();

  /**
    * @brief Constructor with name and reference frame provided
    * @brief name the name of the state
    * @brief reference the name of the reference frame
     */
  explicit DualQuaternionState(const std::string& name, const std::string& reference = "world");

  /**
    * @brief Copy constructor
     */
  DualQuaternionState(const DualQuaternionState& state);

  /**
    * @brief Construct a DualQuaternion from two quaternions
    * @param name the name of the state
    * @param primary the value of the primary quaternion
    * @param dual the value of the dual quaternion
     */
  explicit DualQuaternionState(const std::string& name,
                               const Eigen::Quaterniond& primary,
                               const Eigen::Quaterniond& dual,
                               const std::string& reference = "world");

  /**
    * @brief Getter of the primary attribute
     */
  const Eigen::Quaterniond& get_primary() const;

  /**
    * @brief Getter of the dual attribute
     */
  const Eigen::Quaterniond& get_dual() const;

  /**
    * @brief Setter of the primary attribute
     */
  void set_primary(const Eigen::Quaterniond& primary);

  /**
    * @brief Setter of the dual attribute
     */
  void set_dual(const Eigen::Quaterniond& dual);

  /**
    * @brief Overload the *= operator
    * @param q DualQuaternion to multiply with
    * @return the current DualQuaternion multiply by the DualQuaternion given in argument
     */
  DualQuaternionState& operator*=(const DualQuaternionState& q);

  /**
    * @brief Overload the * operator
    * @param p DualQuaternionState to multiply with
    * @return the current DualQuaternionState multiply by the DualQuaternionState given in argument
     */
  const DualQuaternionState operator*(const DualQuaternionState& p) const;

  /**
   * @brief compute the conjugate of the current DualQuaternion
   * @return the inverse
   */
  const DualQuaternionState conjugate() const;

  /**
    * @brief Initialize the DualQuaternionState to a zero value
     */
  virtual void initialize();

  /**
   * @brief Return a copy of the DualQuaternionState
   * @return the copy
   */
  const DualQuaternionState copy() const;

  /**
    * @brief Overload the * operator with a scalar
    * @param lambda the scalar to multiply with
    * @return the DualQuaternionState provided multiply by lambda
     */
  friend const DualQuaternionState operator*(const float& lambda, const DualQuaternionState& state);

  /**
   * @brief overload exp function
   * @param state the DualQuaternion to operate on
   * @return the exponential of the DualQuaternion provided
   */
  friend const DualQuaternionState exp(const DualQuaternionState& state);

  /**
    * @brief Overload the ostream operator for printing
    * @param os the ostream to happend the string representing the state to
    * @param state the state to print
    * @return the appended ostream
     */
  friend std::ostream& operator<<(std::ostream& os, const DualQuaternionState& state);
};

inline const Eigen::Quaterniond& DualQuaternionState::get_primary() const {
  return this->primary;
}

inline const Eigen::Quaterniond& DualQuaternionState::get_dual() const {
  return this->dual;
}

inline void DualQuaternionState::set_primary(const Eigen::Quaterniond& primary) {
  this->set_filled();
  this->primary = primary;
}

inline void DualQuaternionState::set_dual(const Eigen::Quaterniond& dual) {
  this->set_filled();
  this->dual = dual;
}
}
